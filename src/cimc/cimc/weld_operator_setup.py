"""One-time operator review before the camera/ABB perception chain starts.

This process never publishes robot commands. The camera viewer must exit before
this process can approve, so the ROS camera node can acquire the device.
"""

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import yaml


GROUP_DEFAULTS = {
    'start_transition': (0, 0, 0),
    'end_transition': (0, 0, 0),
    'protruding_left': (2.5, 0, 0),
    'protruding_right': (-0.5, 0, 0),
    'recessed_left': (0, 1, 0),
    'recessed_right': (0, 1, 0),
    'straight_protruding_flat': (0, 0, 0),
    'straight_left_waist': (0, 0, 0),
    'straight_recessed_flat': (0, 0, 0),
    'straight_right_waist': (0, 0, 0),
    'adaptive_contour': (0, 0, 0),
}
ROI_KEYS = ('roi.enable', 'roi.min_x', 'roi.max_x', 'roi.min_y',
            'roi.max_y', 'roi.min_z', 'roi.max_z')
MODES = ('feature_points', 'rounded_features', 'adaptive_contour')


def active_overrides(config_text):
    data = yaml.safe_load(config_text)
    items = data['weld_seam_node']['ros__parameters']['algorithm_overrides']
    result = {}
    for item in items:
        key, value = item.split('=', 1)
        if key in result:
            raise ValueError(f'YAML 存在重复生效参数：{key}')
        result[key] = value
    return result


def replace_active_overrides(config_text, changes):
    """Preserve comments and all unrelated YAML lines; add only changed keys."""
    active_overrides(config_text)
    lines = config_text.splitlines(keepends=True)
    remaining = dict(changes)
    pattern = re.compile(r'^(\s*-\s*["\'])([^= "\']+)=([^"\']*)(["\']\s*)$')
    for index, line in enumerate(lines):
        match = pattern.match(line.rstrip('\r\n'))
        if match and match.group(2) in remaining:
            key = match.group(2)
            ending = '\n' if line.endswith('\n') else ''
            lines[index] = f'{match.group(1)}{key}={remaining.pop(key)}{match.group(4)}{ending}'
    if remaining:
        marker = next((i for i, line in enumerate(lines)
                       if line.strip() == 'algorithm_overrides:'), None)
        if marker is None:
            raise ValueError('YAML 缺少 algorithm_overrides')
        lines[marker + 1:marker + 1] = [
            f'      - "{key}={value}"\n' for key, value in remaining.items()]
    result = ''.join(lines)
    parsed = active_overrides(result)
    if any(parsed[key] != value for key, value in changes.items()):
        raise ValueError('配置写入校验失败')
    return result


def finite_or_infinity(value):
    number = float(value)
    if math.isnan(number):
        raise ValueError('ROI 不能包含 NaN')
    return format(number, '.9g')


def make_highlight_ply(result_ply, row, target_ply):
    """Copy SDK result and add one cyan cross without altering existing vertices."""
    data = result_ply.read_bytes()
    end = data.find(b'end_header\n')
    if end < 0 or end > 16384:
        raise ValueError('SDK 结果 PLY 缺少有效头部')
    head_end = end + len(b'end_header\n')
    header = data[:head_end]
    if b'format binary_little_endian 1.0' not in header:
        raise ValueError('高亮只支持 SDK 的二进制小端 result.ply')
    required = (b'property float x\n', b'property float y\n',
                b'property float z\n', b'property uchar red\n',
                b'property uchar green\n', b'property uchar blue\n')
    if not all(field in header for field in required):
        raise ValueError('SDK 结果 PLY 顶点格式与预期不符，未生成高亮文件')
    match = re.search(rb'(?m)^element vertex (\d+)\r?$', header)
    if not match:
        raise ValueError('SDK 结果 PLY 缺少顶点数量')
    count = int(match.group(1))
    vertex_end = head_end + count * 15  # float32 XYZ + uint8 RGB
    if vertex_end > len(data):
        raise ValueError('SDK 结果 PLY 顶点数据不完整')
    center = tuple(float(row[key]) for key in ('x', 'y', 'z'))
    marker = bytearray()
    # Five-millimetre cyan XYZ cross, sampled densely enough for PLY viewers.
    for axis in range(3):
        for step in range(-20, 21):
            point = list(center)
            point[axis] += step * 0.25
            marker.extend(struct.pack('<fffBBB', *point, 0, 255, 255))
    updated_header = (header[:match.start(1)] +
                      str(count + len(marker) // 15).encode('ascii') +
                      header[match.end(1):])
    target_ply.write_bytes(
        updated_header + data[head_end:vertex_end] + marker + data[vertex_end:])
    return target_ply


class PreviewWindow:
    """Keep point-to-YAML mapping separate from the uncluttered SDK PLY."""

    def __init__(self, parent, result_ply, rows):
        self.result_ply = result_ply
        self.rows = rows
        self.window = tk.Toplevel(parent)
        self.window.title(f'焊缝结果核对：{len(rows)} 个点')
        self.window.geometry('1280x650')
        frame = ttk.Frame(self.window, padding=10)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text=(
            '原始 result.ply 保持算法生成效果；下表列出点序号与 YAML 偏移组。'
            '选中一行可生成仅高亮该点的点云（青色十字）。'),
            wraplength=1200).pack(anchor='w')
        ttk.Label(frame, text=str(result_ply), wraplength=1200).pack(anchor='w')
        controls = ttk.Frame(frame)
        controls.pack(fill='x', pady=6)
        ttk.Label(controls, text='按偏移组筛选：').pack(side='left')
        self.group = tk.StringVar(value='全部')
        groups = ['全部'] + sorted({row['position_offset_group'] for row in rows})
        selector = ttk.Combobox(controls, textvariable=self.group, values=groups,
                                width=26, state='readonly')
        selector.pack(side='left')
        selector.bind('<<ComboboxSelected>>', lambda _event: self.populate())
        ttk.Button(controls, text='在 VS Code 打开原始 result.ply',
                   command=lambda: self.open_ply(result_ply)).pack(side='left', padx=8)
        ttk.Button(controls, text='高亮选中点并在 VS Code 打开',
                   command=self.highlight_selected).pack(side='left', padx=8)
        ttk.Button(controls, text='复制原始 PLY 路径',
                   command=self.copy_path).pack(side='left', padx=8)

        columns = ('point', 'feature', 'group', 'offset', 'source', 'camera', 'workpiece')
        table_frame = ttk.Frame(frame)
        table_frame.pack(fill='both', expand=True)
        self.table = ttk.Treeview(table_frame, columns=columns, show='headings',
                                  selectmode='browse')
        headings = ('点序号', '特征类型', 'YAML 偏移组', '实际偏移 X/Y/Z mm',
                    '点来源', '相机 XYZ mm', '工件 XYZ mm')
        widths = (65, 290, 190, 160, 250, 175, 175)
        for column, title, width in zip(columns, headings, widths):
            self.table.heading(column, text=title)
            self.table.column(column, width=width, minwidth=50, stretch=True)
        horizontal = ttk.Scrollbar(table_frame, orient='horizontal',
                                   command=self.table.xview)
        horizontal.pack(side='bottom', fill='x')
        self.table.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(table_frame, orient='vertical', command=self.table.yview)
        scrollbar.pack(side='right', fill='y')
        self.table.configure(yscrollcommand=scrollbar.set,
                             xscrollcommand=horizontal.set)
        self.table.bind('<<TreeviewSelect>>', self.show_selection)
        self.detail = tk.StringVar(value='选择一行，查看该点对应的 YAML 参数名。')
        ttk.Label(frame, textvariable=self.detail, wraplength=1200).pack(
            fill='x', pady=6)
        self.populate()

    def populate(self):
        self.table.delete(*self.table.get_children())
        for row in self.rows:
            if self.group.get() not in ('全部', row['position_offset_group']):
                continue
            order = int(row['order'])
            values = (
                f'P{order + 1}', row['feature_type'], row['position_offset_group'],
                '/'.join(row[f'applied_offset_workpiece_{axis}'] for axis in 'xyz'),
                row['point_source'],
                '/'.join(row[axis] for axis in 'xyz'),
                '/'.join(row[f'workpiece_{axis}'] for axis in 'xyz'),
            )
            self.table.insert('', 'end', iid=str(order), values=values)

    def selected_row(self):
        selected = self.table.selection()
        if not selected:
            raise ValueError('请先在表格里选中一个点')
        order = int(selected[0])
        return next(row for row in self.rows if int(row['order']) == order)

    def show_selection(self, _event=None):
        try:
            row = self.selected_row()
        except ValueError:
            return
        group = row['position_offset_group']
        self.detail.set(
            f"P{int(row['order']) + 1} → {row['feature_type']}；"
            f"对应 YAML：offset.{group}.x / .y / .z；"
            f"来源：{row['point_source']}。偏移作用于工件坐标系，不是相机 XYZ。")

    def copy_path(self):
        self.window.clipboard_clear()
        self.window.clipboard_append(str(self.result_ply))
        self.detail.set('已复制原始 result.ply 路径；可直接粘贴到 VS Code 打开。')

    def open_ply(self, path):
        code = shutil.which('code')
        if not code:
            messagebox.showinfo('请手动打开 PLY', f'未找到 VS Code 命令，请打开：\n{path}',
                                parent=self.window)
            return
        try:
            subprocess.Popen([code, '--reuse-window', str(path)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            messagebox.showerror('打开 PLY 失败', str(exc), parent=self.window)

    def highlight_selected(self):
        try:
            row = self.selected_row()
            order = int(row['order'])
            target = self.result_ply.with_name(
                f'{self.result_ply.stem}_highlight_P{order + 1}.ply')
            make_highlight_ply(self.result_ply, row, target)
            self.open_ply(target)
            self.detail.set(f'P{order + 1} 已用青色十字高亮；原始 result.ply 未修改。')
        except Exception as exc:
            messagebox.showerror('高亮失败', str(exc), parent=self.window)


class SetupWindow:
    def __init__(self, workspace, config_path, camera_path):
        self.workspace = workspace
        self.config_path = config_path
        self.camera_path = camera_path
        self.original_text = config_path.read_text(encoding='utf-8')
        self.original_hash = hashlib.sha256(self.original_text.encode()).hexdigest()
        self.original = active_overrides(self.original_text)
        self.session = workspace / 'operator_sessions' / time.strftime('%Y%m%d_%H%M%S')
        self.session.mkdir(parents=True, exist_ok=True)
        self.manifest = self.session / 'viewer_snapshot.yaml'
        self.ply = None
        self.viewer_running = False
        self.preview_signature = None
        self.preview_rows = None
        self.preview_window = None
        self.approved = False
        self.root = tk.Tk()
        self.root.title('焊缝任务启动前确认（仅本次启动一次）')
        self.root.protocol('WM_DELETE_WINDOW', self.cancel)
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='ABB TCP 已监听；确认前拍照请求会被拒绝，不会缓存旧位姿。').grid(
            row=0, column=0, columnspan=5, sticky='w')
        ttk.Button(outer, text='打开实时点云/画 ROI（S 保存，Q 退出）',
                   command=self.open_viewer).grid(row=1, column=0, columnspan=2, sticky='w')
        ttk.Button(outer, text='选历史 PLY', command=self.pick_ply).grid(row=1, column=2)
        self.ply_label = ttk.Label(outer, text='未选 PLY；如配置无需修改可直接确认')
        self.ply_label.grid(row=2, column=0, columnspan=5, sticky='w')
        self.vars = {}
        self.vars['roi.enable'] = tk.StringVar(value=self.original.get('roi.enable', 'false'))
        ttk.Label(outer, text='ROI 启用').grid(row=3, column=0, sticky='w')
        ttk.Combobox(outer, textvariable=self.vars['roi.enable'], values=('true', 'false'),
                     width=12, state='readonly').grid(row=3, column=1, sticky='w')
        for idx, key in enumerate(ROI_KEYS[1:], 4):
            ttk.Label(outer, text=key).grid(row=idx, column=0, sticky='w')
            self.vars[key] = tk.StringVar(value=self.original.get(key, '-inf' if 'min_' in key else 'inf'))
            ttk.Entry(outer, textvariable=self.vars[key], width=16).grid(row=idx, column=1, sticky='w')
        ttk.Label(outer, text='算法模式').grid(row=10, column=0, sticky='w')
        self.vars['path.mode'] = tk.StringVar(value=self.original.get('path.mode', 'feature_points'))
        ttk.Combobox(outer, textvariable=self.vars['path.mode'], values=MODES,
                     width=22, state='readonly').grid(row=10, column=1, sticky='w')
        ttk.Label(outer, text='偏移 mm：工件 X / Y / Z；预览表格显示每个点对应的参数组').grid(
            row=11, column=0, columnspan=5, sticky='w')
        for index, (group, defaults) in enumerate(GROUP_DEFAULTS.items(), 12):
            ttk.Label(outer, text=group).grid(row=index, column=0, sticky='w')
            for col, (axis, default) in enumerate(zip('xyz', defaults), 1):
                key = f'offset.{group}.{axis}'
                self.vars[key] = tk.StringVar(value=self.original.get(key, str(default)))
                ttk.Entry(outer, textvariable=self.vars[key], width=12).grid(
                    row=index, column=col, sticky='w')
        ttk.Button(outer, text='离线生成原始结果 PLY 与点位对照表', command=self.preview).grid(
            row=23, column=0, columnspan=2, sticky='w')
        ttk.Button(outer, text='确认配置，启动自动链路', command=self.approve).grid(
            row=23, column=2, columnspan=2, sticky='w')
        self.status = tk.StringVar(value='可直接确认原配置；任何修改须先用 PLY 预览。')
        ttk.Label(outer, textvariable=self.status, wraplength=780).grid(
            row=24, column=0, columnspan=5, sticky='w')

    def changes(self):
        wanted = {}
        for key, var in self.vars.items():
            value = var.get().strip()
            if key.startswith('roi.') and key != 'roi.enable':
                value = finite_or_infinity(value)
            elif key.startswith('offset.'):
                number = float(value)
                if not math.isfinite(number) or abs(number) > 100:
                    raise ValueError(f'{key} 必须是有限数，绝对值不超过 100 mm')
                value = format(number, '.9g')
            wanted[key] = value
        if wanted['roi.enable'] not in ('true', 'false') or wanted['path.mode'] not in MODES:
            raise ValueError('ROI 开关或算法模式无效')
        for axis in 'xyz':
            if float(wanted[f'roi.min_{axis}']) > float(wanted[f'roi.max_{axis}']):
                raise ValueError(f'ROI {axis.upper()} 最小值大于最大值')
        return {key: value for key, value in wanted.items()
                if value != self.original.get(key, value)}

    def open_viewer(self):
        camera = yaml.safe_load(self.camera_path.read_text(encoding='utf-8'))
        params = camera['chishine_camera_node']['ros__parameters']
        exe = self.workspace / 'x86_chishine_live_viewer/build/chishine_live_viewer'
        if not exe.is_file():
            messagebox.showerror('预览程序缺失', f'请先构建：{exe}')
            return
        command = [str(exe), '--network', 'true', '--usb', 'false',
                   '--save-dir', str(self.session), '--snapshot-manifest', str(self.manifest),
                   '--point-size', '2', '--max-display-points', '600000',
                   '--roi', self.vars['roi.enable'].get()]
        for key in ROI_KEYS[1:]:
            value = self.vars[key].get().strip()
            if math.isfinite(float(value)):
                command += ['--' + key.replace('.', '-').replace('_', '-'), value]
        for key, flag in (('depth_min_mm', '--depth-min-mm'),
                          ('depth_max_mm', '--depth-max-mm'), ('depth_gain', '--gain'),
                          ('depth_exposure', '--exposure'),
                          ('depth_frame_time', '--frame-time')):
            if key in params:
                command += [flag, str(params[key])]
        self.status.set('实时预览独占相机：按 S 保存 ROI 点云，再按 Q 退出。')
        self.root.update_idletasks()
        self.viewer_running = True
        try:
            subprocess.run(command, check=True)
            if self.manifest.is_file():
                data = yaml.safe_load(self.manifest.read_text(encoding='utf-8'))
                self.ply = Path(data['ply'])
                self.ply_label.configure(text=f'已保存：{self.ply}')
                for key in ROI_KEYS:
                    if key in data:
                        self.vars[key].set(str(data[key]).lower())
                self.preview_signature = None
                self.status.set('ROI 已带入表单；请预览、核对后确认。')
        except Exception as exc:
            messagebox.showerror('相机预览失败', str(exc))
        finally:
            self.viewer_running = False

    def pick_ply(self):
        name = filedialog.askopenfilename(filetypes=[('PLY 点云', '*.ply')])
        if name:
            self.ply = Path(name)
            self.ply_label.configure(text=f'离线 PLY：{name}')
            self.preview_signature = None

    def signature(self, changes):
        return (str(self.ply), tuple(sorted(changes.items())))

    def preview(self):
        try:
            if self.viewer_running or self.ply is None or not self.ply.is_file():
                raise ValueError('请先选择历史 PLY，或在实时预览中按 S 保存并按 Q 退出')
            changes = self.changes()
            overrides = dict(self.original)
            overrides.update(changes)
            exe = self.workspace / 'x86_weld_seam_test/install/bin/weld_seam_extractor'
            if not exe.is_file():
                raise FileNotFoundError(f'已安装 SDK 离线程序不存在：{exe}')
            prefix = 'operator_preview'
            command = [str(exe), str(self.ply), str(self.session), prefix]
            config_file = yaml.safe_load(self.original_text)[
                'weld_seam_node']['ros__parameters'].get('config_file', '')
            if config_file:
                command += ['--config', str(config_file)]
            for key, value in overrides.items():
                command += ['--set', f'{key}={value}']
            started = time.perf_counter()
            result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            (self.session / 'extract.log').write_text(
                result.stdout + '\n' + result.stderr, encoding='utf-8')
            if result.returncode:
                raise RuntimeError(f'算法失败，见 {self.session / "extract.log"}\n'
                                   + (result.stderr or result.stdout)[-1000:])
            csv_path = self.session / f'{prefix}_features.csv'
            with csv_path.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.DictReader(stream))
            if not 1 <= len(rows) <= 100:
                raise ValueError(f'点数 {len(rows)} 超过 ABB 的 100 点限制或为空')
            result_ply = self.session / f'{prefix}_result.ply'
            if self.preview_window is not None and self.preview_window.window.winfo_exists():
                self.preview_window.window.destroy()
            self.preview_window = PreviewWindow(self.root, result_ply, rows)
            self.preview_signature = self.signature(changes)
            self.preview_rows = rows
            self.status.set(f'预览成功：{len(rows)} 点，耗时 {1000 * (time.perf_counter()-started):.1f} ms；'
                            f'原始 result.ply：{result_ply}；表格可按偏移组筛选并单点高亮。')
            self.preview_window.open_ply(result_ply)
        except Exception as exc:
            self.preview_signature = None
            messagebox.showerror('离线预览失败', str(exc))

    def approve(self):
        try:
            if self.viewer_running:
                raise ValueError('请先按 Q 关闭相机预览，释放相机连接')
            changes = self.changes()
            if (changes or self.ply is not None) and self.preview_signature != self.signature(changes):
                raise ValueError('已选择 PLY 或修改参数；必须针对当前数值成功预览后才能确认')
            current_text = self.config_path.read_text(encoding='utf-8')
            if hashlib.sha256(current_text.encode()).hexdigest() != self.original_hash:
                raise ValueError('YAML 被其他程序修改；为避免覆盖请取消并重新启动')
            if not messagebox.askyesno('最终确认',
                    f'确认启动自动链路？\n本次配置变更 {len(changes)} 项；'
                    '之后多条 ABB 请求将自动处理，不再逐次询问。'):
                return
            if changes:
                updated = replace_active_overrides(current_text, changes)
                (self.session / 'weld_seam.before.yaml').write_text(current_text, encoding='utf-8')
                temporary = self.config_path.with_name(self.config_path.name + '.operator_tmp')
                temporary.write_text(updated, encoding='utf-8')
                os.replace(temporary, self.config_path)
            (self.session / 'approval.json').write_text(json.dumps({
                'approved_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                'config': str(self.config_path), 'changes': changes,
                'preview_ply': str(self.ply) if self.ply else None,
                'preview_points': len(self.preview_rows) if self.preview_rows else None,
            }, ensure_ascii=False, indent=2), encoding='utf-8')
            self.approved = True
            self.root.destroy()
        except Exception as exc:
            messagebox.showerror('不能启动自动链路', str(exc))

    def cancel(self):
        if messagebox.askyesno('取消', '取消人工确认并停止本次 launch？'):
            self.root.destroy()


def main():
    if len(sys.argv) != 4:
        raise SystemExit('用法: weld_operator_setup <workspace> <weld_yaml> <camera_yaml>')
    window = SetupWindow(*(Path(arg).resolve() for arg in sys.argv[1:]))
    window.root.mainloop()
    return 0 if window.approved else 2


if __name__ == '__main__':
    sys.exit(main())
