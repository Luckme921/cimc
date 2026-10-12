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


def plot_preview(result_ply, rows):
    """Visualize sampled cloud/red seam plus ordered, named target points."""
    import numpy as np
    import matplotlib.pyplot as plt

    with result_ply.open('rb') as stream:
        header = bytearray()
        while not header.endswith(b'end_header\n'):
            chunk = stream.readline()
            if not chunk or len(header) > 16384:
                raise ValueError('结果 PLY 头无效')
            header.extend(chunk)
        head = header.decode('ascii')
        if 'format binary_little_endian 1.0' not in head:
            raise ValueError('预览目前只支持 SDK 输出的二进制 PLY')
        count = int(re.search(r'element vertex (\d+)', head).group(1))
        cloud = np.fromfile(stream, dtype=np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('r', 'u1'), ('g', 'u1'), ('b', 'u1')]), count=count)
    fig = plt.figure(figsize=(13, 7))
    ax = fig.add_subplot(111, projection='3d')
    sample = cloud[::max(1, len(cloud) // 25000)]
    ax.scatter(sample['x'], sample['y'], sample['z'], c='0.7', s=0.2, alpha=0.3)
    red = cloud[(cloud['r'] > 150) & (cloud['r'] > cloud['g'] * 1.5)
                & (cloud['r'] > cloud['b'] * 1.5)]
    red = red[::max(1, len(red) // 12000)]
    ax.scatter(red['x'], red['y'], red['z'], c='red', s=1, alpha=0.7)
    for row in rows:
        x, y, z = (float(row[k]) for k in ('x', 'y', 'z'))
        ax.scatter([x], [y], [z], c='magenta', s=35)
        ax.text(x, y, z, f"P{int(row['order']) + 1} {row['position_offset_group']}",
                fontsize=7)
    ax.set(xlabel='camera X / mm', ylabel='camera Y / mm', zlabel='camera Z / mm')
    ax.set_title(f'Offline review: {len(rows)} points; close window to return to setup')
    plt.show(block=False)


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
        ttk.Label(outer, text='偏移 mm：工件 X / Y / Z；预览点标签显示对应组').grid(
            row=11, column=0, columnspan=5, sticky='w')
        for index, (group, defaults) in enumerate(GROUP_DEFAULTS.items(), 12):
            ttk.Label(outer, text=group).grid(row=index, column=0, sticky='w')
            for col, (axis, default) in enumerate(zip('xyz', defaults), 1):
                key = f'offset.{group}.{axis}'
                self.vars[key] = tk.StringVar(value=self.original.get(key, str(default)))
                ttk.Entry(outer, textvariable=self.vars[key], width=12).grid(
                    row=index, column=col, sticky='w')
        ttk.Button(outer, text='离线预览并标出点位/偏移组', command=self.preview).grid(
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
            self.preview_signature = self.signature(changes)
            self.preview_rows = rows
            self.status.set(f'预览成功：{len(rows)} 点，耗时 {1000 * (time.perf_counter()-started):.1f} ms；'
                            f'结果 {self.session}。关闭图窗后可继续调整。')
            plot_preview(self.session / f'{prefix}_result.ply', rows)
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
