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


def camera_save_directory(camera_params):
    """Match the camera ROS node's configured/default PLY directory."""
    configured = str(camera_params.get('output_directory', '')).strip()
    if configured:
        return Path(configured).expanduser()
    data_root = os.environ.get('SCUT_WELD_DATA_ROOT', '').strip()
    return (Path(data_root).expanduser() if data_root else
            Path.home() / 'scut_weld_data') / 'pointclouds'


def viewer_command(executable, save_directory, manifest, camera_params, roi_values):
    """Use the same depth settings as camera.yaml, plus the live ROI controls."""
    command = [str(executable), '--network', 'true', '--usb', 'false',
               '--save-dir', str(save_directory), '--snapshot-manifest', str(manifest),
               '--point-size', '2', '--max-display-points', '600000',
               '--roi', roi_values['roi.enable']]
    for key in ROI_KEYS[1:]:
        value = roi_values[key]
        if math.isfinite(float(value)):
            command += ['--' + key.replace('.', '-').replace('_', '-'), value]
    for key, flag in (('depth_min_mm', '--depth-min-mm'),
                      ('depth_max_mm', '--depth-max-mm'), ('depth_gain', '--gain'),
                      ('depth_exposure', '--exposure'),
                      ('depth_frame_time', '--frame-time')):
        if key in camera_params:
            command += [flag, str(camera_params[key])]
    return command


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
        self.camera_hash = hashlib.sha256(camera_path.read_bytes()).hexdigest()
        self.session = workspace / 'operator_sessions' / time.strftime('%Y%m%d_%H%M%S')
        self.session.mkdir(parents=True, exist_ok=True)
        self.manifest = self.session / 'viewer_snapshot.yaml'
        self.ply = None
        self.viewer_running = False
        self.preview_signature = None
        self.preview_rows = None
        self.preview_warnings = []
        self.preview_window = None
        self.preview_result_ply = None
        self.preview_index = 0
        self.preview_history = []
        self.approved = False
        self.camera_params = yaml.safe_load(camera_path.read_text(encoding='utf-8'))[
            'chishine_camera_node']['ros__parameters']
        self.root = tk.Tk()
        self.root.title('焊缝任务 · 启动前确认')
        self.root.geometry('1060x760')
        self.root.minsize(870, 620)
        self.root.protocol('WM_DELETE_WINDOW', self.cancel)
        style = ttk.Style(self.root)
        if 'clam' in style.theme_names():
            style.theme_use('clam')
        style.configure('TFrame', background='#f5f8fb')
        style.configure('TLabel', background='#f5f8fb', foreground='#1c2b39',
                        font=('Noto Sans CJK SC', 10))
        style.configure('Title.TLabel', font=('Noto Sans CJK SC', 19, 'bold'),
                        foreground='#133a56')
        style.configure('Muted.TLabel', foreground='#526576')
        style.configure('Safety.TLabel', foreground='#8a4b00',
                        font=('Noto Sans CJK SC', 10, 'bold'))
        style.configure('Accent.TButton', font=('Noto Sans CJK SC', 10, 'bold'),
                        foreground='white', background='#167a78', padding=(14, 8))
        style.map('Accent.TButton', background=[('active', '#116a68')])
        style.configure('TButton', padding=(10, 6))
        style.configure('TLabelframe', background='#f5f8fb')
        style.configure('TLabelframe.Label', background='#f5f8fb',
                        foreground='#254b63', font=('Noto Sans CJK SC', 10, 'bold'))
        self.root.configure(background='#f5f8fb')

        self.vars = {}
        self.vars['roi.enable'] = tk.StringVar(value=self.original.get('roi.enable', 'false'))
        for key in ROI_KEYS[1:]:
            self.vars[key] = tk.StringVar(value=self.original.get(
                key, '-inf' if 'min_' in key else 'inf'))
        self.vars['path.mode'] = tk.StringVar(value=self.original.get(
            'path.mode', 'feature_points'))
        for group, defaults in GROUP_DEFAULTS.items():
            for axis, default in zip('xyz', defaults):
                key = f'offset.{group}.{axis}'
                self.vars[key] = tk.StringVar(value=self.original.get(key, str(default)))

        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='焊缝任务启动前确认', style='Title.TLabel').pack(anchor='w')
        ttk.Label(outer, text='① 选择点云 / ROI  →  ② 调整算法与偏移  →  ③ 离线预览  →  ④ 确认后自动处理多条 ABB 请求',
                  style='Muted.TLabel').pack(anchor='w', pady=(3, 8))
        ttk.Label(outer, text='安全边界：ABB TCP 可先连接；确认前拒绝拍照指令，不缓存旧机器人位姿。',
                  style='Safety.TLabel').pack(anchor='w', pady=(0, 12))

        tabs = ttk.Notebook(outer)
        tabs.pack(fill='both', expand=True)
        capture_tab = ttk.Frame(tabs, padding=16)
        offset_tab = ttk.Frame(tabs, padding=16)
        review_tab = ttk.Frame(tabs, padding=16)
        tabs.add(capture_tab, text='  1  点云与 ROI  ')
        tabs.add(offset_tab, text='  2  算法与点位偏移  ')
        tabs.add(review_tab, text='  3  复核与启动  ')

        capture = ttk.LabelFrame(capture_tab, text='点云来源', padding=12)
        capture.pack(fill='x', pady=(0, 12))
        ttk.Button(capture, text='实时预览相机并画 ROI',
                   command=self.open_viewer).grid(row=0, column=0, sticky='w')
        ttk.Label(capture, text='S 保存当前 ROI 点云 → Q 退出预览，释放相机').grid(
            row=0, column=1, padx=14, sticky='w')
        ttk.Button(capture, text='选择历史 PLY', command=self.pick_ply).grid(
            row=1, column=0, sticky='w', pady=(8, 0))
        self.ply_label = ttk.Label(capture, text='尚未选择 PLY；若不改配置，可直接确认沿用已有配置。',
                                   wraplength=740)
        self.ply_label.grid(row=1, column=1, padx=14, pady=(8, 0), sticky='w')
        ttk.Button(capture, text='清除已选 PLY', command=self.clear_ply).grid(
            row=2, column=0, sticky='w', pady=(8, 0))
        camera_box = ttk.LabelFrame(capture_tab, text='实时预览成像参数 · 深度值来自 camera.yaml',
                                    padding=12)
        camera_box.pack(fill='x', pady=(0, 12))
        p = self.camera_params
        camera_text = (
            f"网络 true · USB false · 深度 {p.get('depth_min_mm')}–{p.get('depth_max_mm')} mm · "
            f"增益 {p.get('depth_gain')} · 曝光 {p.get('depth_exposure')} μs · "
            f"帧时间 {p.get('depth_frame_time')} μs\n"
            f"点大小 2 · 最多显示 600000 点 · S 保存到 {camera_save_directory(p)}")
        ttk.Label(camera_box, text=camera_text, wraplength=930).pack(anchor='w')
        ttk.Label(camera_box, text='实时预览固定使用网络发现、关闭 USB；深度参数要到 camera.yaml 修改。本窗口仅编辑焊缝算法的 ROI/模式/点位偏移。',
                  style='Muted.TLabel').pack(anchor='w', pady=(6, 0))
        roi_box = ttk.LabelFrame(capture_tab, text='算法 ROI · 相机坐标系 / mm', padding=12)
        roi_box.pack(fill='x')
        ttk.Label(roi_box, text='启用 ROI').grid(row=0, column=0, sticky='w', pady=4)
        ttk.Combobox(roi_box, textvariable=self.vars['roi.enable'], values=('true', 'false'),
                     width=12, state='readonly').grid(row=0, column=1, sticky='w')
        ttk.Label(roi_box, text='最小值').grid(row=1, column=1, sticky='w')
        ttk.Label(roi_box, text='最大值').grid(row=1, column=2, sticky='w')
        for row, axis in enumerate('xyz', 2):
            ttk.Label(roi_box, text=f'{axis.upper()} 轴').grid(row=row, column=0, sticky='w', pady=5)
            for col, bound in enumerate(('min', 'max'), 1):
                ttk.Entry(roi_box, textvariable=self.vars[f'roi.{bound}_{axis}'],
                          width=18).grid(row=row, column=col, padx=(0, 14), sticky='w')
        ttk.Label(roi_box, text='-inf / inf = 该方向不裁剪。历史 PLY 若已裁剪，也会再次应用这里的算法 ROI。',
                  style='Muted.TLabel').grid(row=5, column=0, columnspan=3, sticky='w', pady=(8, 0))

        mode_box = ttk.LabelFrame(offset_tab, text='路径算法', padding=12)
        mode_box.pack(fill='x', pady=(0, 10))
        ttk.Label(mode_box, text='path.mode').pack(side='left')
        ttk.Combobox(mode_box, textvariable=self.vars['path.mode'], values=MODES,
                     width=24, state='readonly').pack(side='left', padx=12)
        ttk.Label(mode_box, text='feature_points 单角点 / rounded_features 圆角离散 / adaptive_contour 自适应轮廓',
                  style='Muted.TLabel').pack(side='left')
        groups_box = ttk.LabelFrame(offset_tab, text='工件坐标系下的点位偏移 / mm', padding=10)
        groups_box.pack(fill='both', expand=True)
        for column, title in enumerate(('参数组（预览表格会标出）', 'X', 'Y', 'Z')):
            ttk.Label(groups_box, text=title).grid(row=0, column=column, padx=(0, 12), sticky='w')
        for row, (group, defaults) in enumerate(GROUP_DEFAULTS.items(), 1):
            ttk.Label(groups_box, text=group).grid(row=row, column=0, padx=(0, 12), pady=3, sticky='w')
            for column, axis in enumerate('xyz', 1):
                ttk.Entry(groups_box, textvariable=self.vars[f'offset.{group}.{axis}'],
                          width=15).grid(row=row, column=column, padx=(0, 12), pady=3, sticky='w')
        ttk.Label(offset_tab, text='每次离线预览都会用此刻输入框里的最新数值；预览不会写正式 YAML。',
                  style='Muted.TLabel').pack(anchor='w', pady=(10, 0))

        ttk.Label(review_tab, text='复核当前输入', style='Title.TLabel').pack(anchor='w')
        self.pending = tk.StringVar()
        ttk.Label(review_tab, textvariable=self.pending, wraplength=930).pack(
            anchor='w', fill='x', pady=(8, 12))
        self.preview_info = tk.StringVar(value='尚未离线预览。若不修改参数且不选 PLY，可直接确认沿用原配置。')
        ttk.Label(review_tab, textvariable=self.preview_info, wraplength=930).pack(
            anchor='w', fill='x')
        self.warning_info = tk.StringVar(value='')
        ttk.Label(review_tab, textvariable=self.warning_info, style='Safety.TLabel',
                  wraplength=930).pack(anchor='w', fill='x', pady=(7, 0))
        ttk.Label(review_tab, text=(
            '预览成功后：原始 result.ply 会尝试在 VS Code 打开；独立表格可按偏移组筛选，'
            '并高亮单个 P 点。若修改任何输入值，需重新预览。'),
            style='Muted.TLabel', wraplength=930).pack(anchor='w', pady=(12, 0))
        self.auto_open_result = tk.BooleanVar(value=True)
        ttk.Checkbutton(review_tab, text='预览完成后自动在 VS Code 打开原始 result.ply',
                        variable=self.auto_open_result).pack(anchor='w', pady=(12, 5))
        ttk.Label(review_tab, text='本次启动的预览历史（每次结果单独保留，不覆盖上一版）：').pack(
            anchor='w', pady=(12, 3))
        self.history_listbox = tk.Listbox(review_tab, height=7, exportselection=False,
                                          background='white', foreground='#1c2b39',
                                          selectbackground='#d3eeeb')
        self.history_listbox.pack(fill='x')
        self.history_listbox.bind('<Double-Button-1>', lambda _event: self.open_history())
        ttk.Button(review_tab, text='打开选中历史结果', command=self.open_history).pack(
            anchor='w', pady=(8, 0))

        footer = ttk.Frame(outer)
        footer.pack(fill='x', pady=(14, 0))
        ttk.Button(footer, text='取消本次启动', command=self.cancel).pack(side='left')
        self.preview_button = ttk.Button(footer, text='用当前参数离线预览', command=self.preview)
        self.preview_button.pack(side='right', padx=(8, 0))
        self.approve_button = ttk.Button(footer, text='确认配置并启动自动链路',
                                         style='Accent.TButton', command=self.approve)
        self.approve_button.pack(side='right', padx=(8, 0))
        self.status = tk.StringVar(value='当前配置尚未改动；需要调整时先选点云，再预览。')
        ttk.Label(outer, textvariable=self.status, style='Muted.TLabel',
                  wraplength=1000).pack(anchor='w', pady=(8, 0))
        for variable in self.vars.values():
            variable.trace_add('write', self.mark_changed)
        self.update_review()

    def mark_changed(self, *_args):
        self.preview_signature = None
        self.update_review()

    def update_review(self):
        try:
            changes = self.changes()
        except (ValueError, KeyError, OSError) as exc:
            self.pending.set(f'输入尚未有效：{exc}')
            self.approve_button.configure(state='disabled')
            return
        if changes:
            examples = '、'.join(changes.keys())
            self.pending.set(
                f'相对启动时 YAML 有 {len(changes)} 项待写入：{examples}\n'
                f'这些值只在“确认配置”后写入 {self.config_path}。')
        else:
            self.pending.set('当前输入与启动时 YAML 一致；确认时不会重写配置文件。')
        preview_needed = bool(changes or self.ply is not None)
        try:
            preview_current = self.preview_signature == self.signature(changes)
        except OSError as exc:
            self.pending.set(f'已选点云文件不可用：{exc}')
            self.approve_button.configure(state='disabled')
            return
        if preview_needed and not preview_current:
            self.approve_button.configure(state='disabled')
            self.preview_info.set('当前输入尚未通过离线预览；请先用最新值生成结果，再确认。')
            self.warning_info.set('')
        else:
            self.approve_button.configure(state='normal')
            if preview_current and self.preview_rows is not None:
                self.preview_info.set(
                    f'当前输入已预览：{len(self.preview_rows)} 个轨迹点；'
                    f'结果在 {self.preview_result_ply}。')
                self.warning_info.set('\n'.join(self.preview_warnings))
            else:
                self.preview_info.set('无需修改或预览；可直接沿用已有 YAML 启动自动链路。')
                self.warning_info.set('')

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
        changes = {}
        for key, value in wanted.items():
            previous = self.original.get(key, value)
            if key.startswith('offset.') or (key.startswith('roi.') and key != 'roi.enable'):
                # YAML often spells the same number as 1.000 or 188.380. Do not
                # silently rewrite tuned values when the operator did not edit them.
                changed = float(value) != float(previous)
            else:
                changed = value != previous
            if changed:
                changes[key] = value
        return changes

    def open_viewer(self):
        exe = self.workspace / 'x86_chishine_live_viewer/build/chishine_live_viewer'
        if not exe.is_file():
            messagebox.showerror('预览程序缺失', f'请先构建：{exe}')
            return
        try:
            roi_values = {key: self.vars[key].get().strip() for key in ROI_KEYS}
            command = viewer_command(
                exe, camera_save_directory(self.camera_params), self.manifest,
                self.camera_params, roi_values)
            previous_manifest_time = (
                self.manifest.stat().st_mtime_ns if self.manifest.is_file() else None)
            self.status.set('实时预览独占相机：按 S 保存 ROI 点云，再按 Q 退出。')
            self.root.update_idletasks()
            self.viewer_running = True
            subprocess.run(command, check=True)
            if (self.manifest.is_file() and
                    (previous_manifest_time is None or
                     self.manifest.stat().st_mtime_ns > previous_manifest_time)):
                data = yaml.safe_load(self.manifest.read_text(encoding='utf-8'))
                self.ply = Path(data['ply'])
                self.ply_label.configure(text=f'已保存：{self.ply}')
                for key in ROI_KEYS:
                    if key in data:
                        self.vars[key].set(str(data[key]).lower())
                self.preview_signature = None
                self.status.set('ROI 已带入表单；请预览、核对后确认。')
                self.update_review()
            else:
                self.status.set('预览已退出但没有新保存的 PLY；仍使用之前选择的点云。')
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
            self.update_review()

    def clear_ply(self):
        self.ply = None
        self.ply_label.configure(text='未选择 PLY；若未改参数，可直接沿用现有 YAML。')
        self.preview_signature = None
        self.update_review()

    def open_history(self):
        selected = self.history_listbox.curselection()
        if not selected:
            messagebox.showinfo('预览历史', '请先选中一条历史预览。', parent=self.root)
            return
        item = self.preview_history[selected[0]]
        if self.preview_window is not None and self.preview_window.window.winfo_exists():
            self.preview_window.window.destroy()
        self.preview_window = PreviewWindow(self.root, item['result_ply'], item['rows'])
        self.preview_window.open_ply(item['result_ply'])

    def signature(self, changes):
        if self.ply is None:
            file_identity = None
        else:
            stat = self.ply.stat()
            file_identity = (str(self.ply.resolve()), stat.st_size, stat.st_mtime_ns)
        return (file_identity, tuple(sorted(changes.items())))

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
            self.preview_index += 1
            prefix = f'operator_preview_{self.preview_index:03d}'
            command = [str(exe), str(self.ply), str(self.session), prefix]
            config_file = yaml.safe_load(self.original_text)[
                'weld_seam_node']['ros__parameters'].get('config_file', '')
            if config_file:
                command += ['--config', str(config_file)]
            for key, value in overrides.items():
                command += ['--set', f'{key}={value}']
            started = time.perf_counter()
            result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            warnings = [line.strip() for line in (result.stdout + '\n' + result.stderr).splitlines()
                        if line.strip().startswith('WARNING:')]
            log_path = self.session / f'extract_{self.preview_index:03d}.log'
            log_path.write_text(
                result.stdout + '\n' + result.stderr, encoding='utf-8')
            if result.returncode:
                raise RuntimeError(f'算法失败，见 {log_path}\n'
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
            self.preview_warnings = warnings
            self.preview_result_ply = result_ply
            self.preview_history.append({
                'result_ply': result_ply, 'rows': rows,
                'source_ply': self.ply, 'changes': dict(changes),
            })
            self.history_listbox.insert(
                'end', f'{prefix} · {len(rows)} 点 · 变更 {len(changes)} 项 · '
                       f'{self.ply.name}')
            self.history_listbox.selection_clear(0, 'end')
            self.history_listbox.selection_set('end')
            self.status.set(f'预览成功：{len(rows)} 点，耗时 {1000 * (time.perf_counter()-started):.1f} ms；'
                            f'原始 result.ply：{result_ply}；表格可按偏移组筛选并单点高亮。')
            self.update_review()
            if self.auto_open_result.get():
                self.preview_window.open_ply(result_ply)
        except Exception as exc:
            self.preview_signature = None
            self.update_review()
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
            if hashlib.sha256(self.camera_path.read_bytes()).hexdigest() != self.camera_hash:
                raise ValueError('camera.yaml 在窗口打开后被修改；请重新启动以核对成像参数')
            if not messagebox.askyesno('最终确认',
                    f'确认启动自动链路？\n\n'
                    f'写入目标：{self.config_path}\n'
                    f'本次待写入 {len(changes)} 项；'
                    f'{"当前预览点数 " + str(len(self.preview_rows)) if self.preview_rows else "沿用现有配置，跳过离线预览"}。\n\n'
                    + ('算法提示：' + '\n'.join(self.preview_warnings) + '\n\n'
                       if self.preview_warnings else '') +
                    '确认后启动相机、焊缝、手眼、协调节点；'
                    '之后多条 ABB 请求自动处理，不再逐次询问。',
                    parent=self.root):
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
                'preview_result_ply': str(self.preview_result_ply) if self.preview_result_ply else None,
                'preview_points': len(self.preview_rows) if self.preview_rows else None,
                'config_sha256_after': hashlib.sha256(
                    self.config_path.read_bytes()).hexdigest(),
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
