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

import numpy as np
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
CAMERA_FIELDS = (
    ('depth_min_mm', '深度下限 / mm'),
    ('depth_max_mm', '深度上限 / mm'),
    ('depth_gain', '深度增益'),
    ('depth_exposure', '曝光 / μs'),
    ('depth_frame_time', '帧时间 / μs'),
    ('capture_max_attempts', '稀疏帧重拍次数'),
    ('min_valid_depth_ratio', '最小有效深度比例'),
)
WELD_COMMON = (
    ('feature.safe_transition_offset_y', '安全点 +Y / mm', '20'),
    ('feature.safe_transition_offset_z', '安全点 +Z / mm', '20'),
    ('path.rounded_corner_spacing', '内部圆角采样间距 / mm', '3'),
    ('path.rounded_corner_point_count', '内部圆角点数（奇数）', '5'),
    ('orientation.protruding_left.work_angle_deg', '凸角左工作角 / °', '45'),
    ('orientation.protruding_right.work_angle_deg', '凸角右工作角 / °', '45'),
    ('orientation.recessed_left.work_angle_deg', '凹角左工作角 / °', '45'),
    ('orientation.recessed_right.work_angle_deg', '凹角右工作角 / °', '45'),
)
HANDEYE_FIELDS = ('matrix_file', 'matrix_key', 'matrix_direction',
                  'matrix_translation_unit')


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


def replace_camera_parameters(config_text, changes):
    """Update existing ROS scalar keys only, preserving comments and unrelated YAML."""
    original = yaml.safe_load(config_text)['chishine_camera_node']['ros__parameters']
    missing = set(changes) - original.keys()
    if missing:
        raise ValueError(f'camera.yaml 缺少参数：{sorted(missing)}')
    lines = config_text.splitlines(keepends=True)
    found = set()
    for index, line in enumerate(lines):
        match = re.match(r'^(    )([a-z_]+):(.*?)(\r?\n)?$', line)
        if match and match.group(2) in changes:
            key = match.group(2)
            if key in found:
                raise ValueError(f'camera.yaml 参数重复：{key}')
            found.add(key)
            ending = match.group(4) or ''
            lines[index] = f'{match.group(1)}{key}: {changes[key]}{ending}'
    if found != set(changes):
        raise ValueError(f'camera.yaml 未找到参数行：{sorted(set(changes) - found)}')
    result = ''.join(lines)
    parsed = yaml.safe_load(result)['chishine_camera_node']['ros__parameters']
    if any(float(parsed[key]) != float(value) for key, value in changes.items()):
        raise ValueError('相机配置写入校验失败')
    return result


def replace_handeye_parameters(config_text, changes):
    """Edit only selected bridge scalars; the matrix file itself is never written."""
    original = yaml.safe_load(config_text)['handeye_abb_bridge_node']['ros__parameters']
    if set(changes) - (set(HANDEYE_FIELDS) & original.keys()):
        raise ValueError('手眼配置包含未知或缺失的参数')
    lines = config_text.splitlines(keepends=True)
    found = set()
    for index, line in enumerate(lines):
        match = re.match(r'^(    )([a-z_]+):(.*?)(\r?\n)?$', line)
        if match and match.group(2) in changes:
            key = match.group(2)
            if key in found:
                raise ValueError(f'手眼配置参数重复：{key}')
            found.add(key)
            lines[index] = f'{match.group(1)}{key}: {json.dumps(changes[key])}{match.group(4) or ""}'
    if found != set(changes):
        raise ValueError(f'手眼配置未找到参数行：{sorted(set(changes) - found)}')
    result = ''.join(lines)
    parsed = yaml.safe_load(result)['handeye_abb_bridge_node']['ros__parameters']
    if any(parsed[key] != value for key, value in changes.items()):
        raise ValueError('手眼配置写入校验失败')
    return result


def inspect_handeye_matrix(path, key):
    """Use the same rigid-transform validity limits as the bridge node."""
    text = Path(path).read_text(encoding='utf-8')
    match = re.search(
        rf'{re.escape(key)}\s*:\s*!!opencv-matrix.*?data\s*:\s*\[([^]]+)\]',
        text, flags=re.DOTALL)
    if not match:
        raise ValueError(f'矩阵文件中未找到 OpenCV 键 {key}')
    values = [float(value) for value in re.findall(
        r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?', match.group(1))]
    if len(values) != 16:
        raise ValueError(f'矩阵必须有 16 个数，实际 {len(values)} 个')
    matrix = np.asarray(values, dtype=float).reshape(4, 4)
    rotation = matrix[:3, :3]
    if (not np.all(np.isfinite(matrix)) or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1.0e-5) or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=5.0e-3) or
            np.linalg.det(rotation) < 0.99):
        raise ValueError('矩阵不是有效的右手刚体变换；禁止启动手眼桥接')
    return matrix, hashlib.sha256(text.encode()).hexdigest()


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
    def __init__(self, workspace, config_path, camera_path, handeye_path=None):
        self.workspace = workspace
        self.config_path = config_path
        self.camera_path = camera_path
        self.handeye_path = (handeye_path if handeye_path is not None else
                             workspace / 'src/cimc/config/handeye_bridge.yaml')
        self.original_text = config_path.read_text(encoding='utf-8')
        self.original_hash = hashlib.sha256(self.original_text.encode()).hexdigest()
        self.original = active_overrides(self.original_text)
        self.camera_original_text = camera_path.read_text(encoding='utf-8')
        self.camera_hash = hashlib.sha256(self.camera_original_text.encode()).hexdigest()
        self.handeye_original_text = self.handeye_path.read_text(encoding='utf-8')
        self.handeye_hash = hashlib.sha256(self.handeye_original_text.encode()).hexdigest()
        self.handeye_params = yaml.safe_load(self.handeye_original_text)[
            'handeye_abb_bridge_node']['ros__parameters']
        try:
            _, self.initial_matrix_hash = inspect_handeye_matrix(
                self.handeye_params['matrix_file'], self.handeye_params['matrix_key'])
        except (ValueError, OSError):
            # Let the operator select a repaired file in the calibration tab.
            self.initial_matrix_hash = None
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
        self.camera_params = yaml.safe_load(self.camera_original_text)[
            'chishine_camera_node']['ros__parameters']
        self.root = tk.Tk()
        self.root.title('华南理工大学智能焊接实验室 · 焊缝任务启动前确认')
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
        style.configure('Brand.TLabel', font=('Noto Sans CJK SC', 12, 'bold'),
                        foreground='#167a78')
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
        for key, _label, default in WELD_COMMON:
            self.vars[key] = tk.StringVar(value=self.original.get(key, default))
        self.camera_vars = {key: tk.StringVar(value=str(self.camera_params[key]))
                            for key, _label in CAMERA_FIELDS}
        self.handeye_vars = {key: tk.StringVar(value=str(self.handeye_params[key]))
                             for key in HANDEYE_FIELDS}
        self.matrix_verified = tk.BooleanVar(value=False)
        self.verified_matrix_digest = None

        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='华南理工大学智能焊接实验室', style='Brand.TLabel').pack(anchor='w')
        ttk.Label(outer, text='焊缝任务启动前确认', style='Title.TLabel').pack(anchor='w', pady=(2, 0))
        ttk.Label(outer, text='① 点云与相机  →  ② 焊缝参数  →  ③ 手眼标定复核  →  ④ 预览并确认  →  运行状态窗',
                  style='Muted.TLabel').pack(anchor='w', pady=(3, 8))
        ttk.Label(outer, text='安全边界：ABB TCP 可先连接；确认前拒绝拍照指令，不缓存旧机器人位姿。',
                  style='Safety.TLabel').pack(anchor='w', pady=(0, 12))

        tabs = ttk.Notebook(outer)
        tabs.pack(fill='both', expand=True)
        capture_tab = ttk.Frame(tabs, padding=16)
        offset_tab = ttk.Frame(tabs, padding=16)
        handeye_tab = ttk.Frame(tabs, padding=16)
        review_tab = ttk.Frame(tabs, padding=16)
        tabs.add(capture_tab, text='  1  点云与 ROI  ')
        tabs.add(offset_tab, text='  2  算法与点位偏移  ')
        tabs.add(handeye_tab, text='  3  手眼标定  ')
        tabs.add(review_tab, text='  4  复核与启动  ')

        capture_pages = ttk.Notebook(capture_tab)
        capture_pages.pack(fill='both', expand=True)
        roi_page = ttk.Frame(capture_pages, padding=10)
        camera_page = ttk.Frame(capture_pages, padding=10)
        capture_pages.add(roi_page, text='  点云与 ROI  ')
        capture_pages.add(camera_page, text='  相机成像参数  ')
        capture = ttk.LabelFrame(roi_page, text='点云来源', padding=12)
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
        roi_box = ttk.LabelFrame(roi_page, text='算法 ROI · 相机坐标系 / mm', padding=12)
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

        camera_box = ttk.LabelFrame(camera_page, text='ROS 相机配置 · camera.yaml', padding=14)
        camera_box.pack(fill='x')
        for row, (key, label) in enumerate(CAMERA_FIELDS):
            ttk.Label(camera_box, text=label).grid(row=row, column=0, sticky='w', pady=6)
            ttk.Entry(camera_box, textvariable=self.camera_vars[key], width=20).grid(
                row=row, column=1, padx=(15, 0), sticky='w')
            ttk.Label(camera_box, text=key, style='Muted.TLabel').grid(
                row=row, column=2, padx=(18, 0), sticky='w')
        ttk.Label(camera_page, text=(
            '实时预览使用上面当前输入的深度参数，网络 true / USB false、显示点大小 2、最多 600000 点；'
            f'按 S 保存到 {camera_save_directory(self.camera_params)}。最终确认才写入 camera.yaml，ROS 相机节点随后读取。'
            '重拍次数和有效比例仅影响 ROS 拍照节点，不影响实时预览。'),
            style='Muted.TLabel', wraplength=900).pack(anchor='w', pady=(12, 0))

        offset_pages = ttk.Notebook(offset_tab)
        offset_pages.pack(fill='both', expand=True)
        common_page = ttk.Frame(offset_pages, padding=10)
        point_page = ttk.Frame(offset_pages, padding=10)
        offset_pages.add(common_page, text='  算法与常用参数  ')
        offset_pages.add(point_page, text='  分类点位偏移  ')

        mode_box = ttk.LabelFrame(common_page, text='路径算法', padding=12)
        mode_box.pack(fill='x', pady=(0, 10))
        ttk.Label(mode_box, text='path.mode').pack(side='left')
        ttk.Combobox(mode_box, textvariable=self.vars['path.mode'], values=MODES,
                     width=24, state='readonly').pack(side='left', padx=12)
        ttk.Label(mode_box, text='feature_points 单角点 / rounded_features 圆角离散 / adaptive_contour 自适应轮廓',
                  style='Muted.TLabel').pack(side='left')
        common_box = ttk.LabelFrame(common_page, text='现场常用算法参数 · 写入 weld_seam.yaml', padding=12)
        common_box.pack(fill='x')
        for row, (key, label, _default) in enumerate(WELD_COMMON):
            ttk.Label(common_box, text=label).grid(row=row, column=0, sticky='w', pady=4)
            ttk.Entry(common_box, textvariable=self.vars[key], width=15).grid(
                row=row, column=1, padx=(12, 18), sticky='w')
            ttk.Label(common_box, text=key, style='Muted.TLabel').grid(
                row=row, column=2, sticky='w')
        ttk.Label(common_page, text='工具轴方向、手眼矩阵等高风险参数仍在各自 YAML 中，不在本窗口快捷修改。',
                  style='Safety.TLabel').pack(anchor='w', pady=(12, 0))
        groups_box = ttk.LabelFrame(point_page, text='工件坐标系下的点位偏移 / mm', padding=10)
        groups_box.pack(fill='both', expand=True)
        for column, title in enumerate(('参数组（预览表格会标出）', 'X', 'Y', 'Z')):
            ttk.Label(groups_box, text=title).grid(row=0, column=column, padx=(0, 12), sticky='w')
        for row, (group, defaults) in enumerate(GROUP_DEFAULTS.items(), 1):
            ttk.Label(groups_box, text=group).grid(row=row, column=0, padx=(0, 12), pady=3, sticky='w')
            for column, axis in enumerate('xyz', 1):
                ttk.Entry(groups_box, textvariable=self.vars[f'offset.{group}.{axis}'],
                          width=15).grid(row=row, column=column, padx=(0, 12), pady=3, sticky='w')
        ttk.Label(point_page, text='离线算法预览使用当前焊缝参数；相机参数用于实时点云预览。两种预览都不写正式 YAML。',
                  style='Muted.TLabel').pack(anchor='w', pady=(10, 0))

        matrix_box = ttk.LabelFrame(handeye_tab, text='标定文件与变换约定 · handeye_bridge.yaml', padding=12)
        matrix_box.pack(fill='x')
        for row, (key, label) in enumerate((
                ('matrix_file', 'OpenCV 标定矩阵文件'),
                ('matrix_key', '矩阵键名'),
                ('matrix_direction', '矩阵方向'),
                ('matrix_translation_unit', '矩阵平移单位'))):
            ttk.Label(matrix_box, text=label).grid(row=row, column=0, sticky='w', pady=7)
            if key in ('matrix_direction', 'matrix_translation_unit'):
                choices = (('tcp_from_camera', 'camera_from_tcp') if key == 'matrix_direction'
                           else ('mm', 'm'))
                widget = ttk.Combobox(matrix_box, textvariable=self.handeye_vars[key],
                                      values=choices, width=28, state='readonly')
            else:
                widget = ttk.Entry(matrix_box, textvariable=self.handeye_vars[key],
                                   width=64 if key == 'matrix_file' else 28)
            widget.grid(row=row, column=1, sticky='w', padx=(14, 8), pady=7)
            if key == 'matrix_file':
                ttk.Button(matrix_box, text='选择文件', command=self.pick_matrix).grid(
                    row=row, column=2, sticky='w')
        ttk.Label(handeye_tab, text=(
            'tcp_from_camera：TCP ← Camera；camera_from_tcp：Camera ← TCP，桥接节点会求逆。'
            '只改变方向并不会重新标定，位置与姿态都可能明显错误。'),
            style='Safety.TLabel', wraplength=920).pack(anchor='w', pady=(12, 6))
        self.matrix_info = tk.StringVar()
        ttk.Label(handeye_tab, textvariable=self.matrix_info, font=('DejaVu Sans Mono', 10),
                  wraplength=920).pack(anchor='w', fill='x', pady=(5, 8))
        ttk.Label(handeye_tab, text=(
            f'输出坐标系：{self.handeye_params.get("output_frame_id")}；'
            f'发送 ABB：{self.handeye_params.get("send_to_abb")}；'
            f'最多点数：{self.handeye_params.get("max_trajectory_points")}。'
            '这三项及矩阵文件数值在本窗口只读，防止误操作。'),
            style='Muted.TLabel', wraplength=920).pack(anchor='w')
        ttk.Checkbutton(handeye_tab,
                        text='若修改标定文件、方向或单位：我已核对 TCP/相机坐标系和矩阵来源',
                        variable=self.matrix_verified, command=self.verify_matrix).pack(
            anchor='w', pady=(15, 0))

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
        for variable in self.camera_vars.values():
            variable.trace_add('write', self.mark_changed)
        for variable in self.handeye_vars.values():
            variable.trace_add('write', self.mark_handeye_changed)
        self.update_matrix_info()
        self.update_review()

    def mark_changed(self, *_args):
        self.preview_signature = None
        self.update_review()

    def mark_handeye_changed(self, *_args):
        self.matrix_verified.set(False)
        self.verified_matrix_digest = None
        self.preview_signature = None
        self.update_matrix_info()
        self.update_review()

    def verify_matrix(self):
        if self.matrix_verified.get():
            try:
                _values, _matrix, self.verified_matrix_digest = self.handeye_values()
            except (ValueError, OSError) as exc:
                self.matrix_verified.set(False)
                self.verified_matrix_digest = None
                messagebox.showerror('手眼矩阵不可用', str(exc), parent=self.root)
        else:
            self.verified_matrix_digest = None
        self.update_review()

    def pick_matrix(self):
        name = filedialog.askopenfilename(
            title='选择手眼标定 OpenCV YAML',
            filetypes=[('YAML 标定矩阵', '*.yaml *.yml'), ('所有文件', '*.*')],
            parent=self.root)
        if name:
            self.handeye_vars['matrix_file'].set(name)

    def handeye_values(self):
        values = {key: var.get().strip() for key, var in self.handeye_vars.items()}
        if not re.fullmatch(r'[A-Za-z_]\w*', values['matrix_key']):
            raise ValueError('矩阵键名只能使用字母、数字和下划线')
        if values['matrix_direction'] not in ('tcp_from_camera', 'camera_from_tcp'):
            raise ValueError('手眼矩阵方向必须明确选择 TCP←Camera 或 Camera←TCP')
        if values['matrix_translation_unit'] not in ('mm', 'm'):
            raise ValueError('手眼矩阵平移单位只能选择 mm 或 m')
        matrix_path = Path(values['matrix_file']).expanduser()
        if not matrix_path.is_absolute():
            matrix_path = self.workspace / matrix_path
        matrix_path = matrix_path.resolve(strict=True)
        values['matrix_file'] = str(matrix_path)
        matrix, digest = inspect_handeye_matrix(matrix_path, values['matrix_key'])
        return values, matrix, digest

    def handeye_changes(self):
        values, matrix, digest = self.handeye_values()
        changes = {key: value for key, value in values.items()
                   if value != str(self.handeye_params[key])}
        return changes, matrix, digest

    def update_matrix_info(self):
        try:
            values, matrix, _digest = self.handeye_values()
            lines = ['已验证右手刚体矩阵（未改写矩阵文件）：']
            lines.extend('  '.join(f'{number: .6f}' for number in row) for row in matrix)
            lines.append(f'平移 {matrix[:3, 3].tolist()} {values["matrix_translation_unit"]}；'
                         f'解释为 {values["matrix_direction"]}。')
            self.matrix_info.set('\n'.join(lines))
        except (ValueError, OSError) as exc:
            self.matrix_info.set(f'标定矩阵不可用：{exc}')

    def update_review(self):
        try:
            changes = self.changes()
            camera_changes = self.camera_changes()
            handeye_changes, _matrix, matrix_digest = self.handeye_changes()
        except (ValueError, KeyError, OSError) as exc:
            self.pending.set(f'输入尚未有效：{exc}')
            self.approve_button.configure(state='disabled')
            return
        if changes or camera_changes or handeye_changes:
            weld_names = '、'.join(changes) if changes else '无'
            camera_names = '、'.join(camera_changes) if camera_changes else '无'
            self.pending.set(
                f'相机待写入 {len(camera_changes)} 项：{camera_names}\n'
                f'焊缝待写入 {len(changes)} 项：{weld_names}\n'
                f'手眼待写入 {len(handeye_changes)} 项：{", ".join(handeye_changes) if handeye_changes else "无"}\n'
                '只有最终确认后才写入相应 src 配置 YAML。')
        else:
            self.pending.set('当前输入与启动时两份 YAML 一致；确认时不会重写配置文件。')
        preview_needed = bool(changes or self.ply is not None)
        try:
            preview_current = self.preview_signature == self.signature(
                changes, camera_changes, handeye_changes, matrix_digest)
        except OSError as exc:
            self.pending.set(f'已选点云文件不可用：{exc}')
            self.approve_button.configure(state='disabled')
            return
        if preview_needed and not preview_current:
            self.approve_button.configure(state='disabled')
            self.preview_info.set('当前输入尚未通过离线预览；请先用最新值生成结果，再确认。')
            self.warning_info.set('')
        elif not handeye_changes and matrix_digest != self.initial_matrix_hash:
            self.approve_button.configure(state='disabled')
            self.preview_info.set('矩阵文件在窗口打开后发生变化；请重新启动并复核标定。')
            self.warning_info.set('')
        elif handeye_changes and (not self.matrix_verified.get() or
                                 self.verified_matrix_digest != matrix_digest):
            self.approve_button.configure(state='disabled')
            self.preview_info.set('手眼参数已修改；请先核对矩阵文件、方向和单位，再勾选标定确认。')
            self.warning_info.set('离线焊缝预览不验证手眼标定方向或机器人可达性。')
        else:
            self.approve_button.configure(state='normal')
            if preview_current and self.preview_rows is not None:
                self.preview_info.set(
                    f'当前输入已预览：{len(self.preview_rows)} 个轨迹点；'
                    f'结果在 {self.preview_result_ply}。')
                self.warning_info.set('\n'.join(self.preview_warnings))
            else:
                self.preview_info.set(
                    '相机参数已校验，但没有点云可离线验收成像；确认后仍须现场核对。'
                    if camera_changes else
                    '无需修改或预览；可直接沿用已有 YAML 启动自动链路。')
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
            elif key == 'path.rounded_corner_point_count':
                count = int(value)
                if str(count) != value or count < 1 or count > 9 or count % 2 != 1:
                    raise ValueError('内部圆角点数必须是 1–9 内的奇数')
            elif key in {item[0] for item in WELD_COMMON}:
                number = float(value)
                if not math.isfinite(number):
                    raise ValueError(f'{key} 必须是有限数')
                if key.endswith('work_angle_deg') and not 0 < number < 90:
                    raise ValueError(f'{key} 必须在 0–90° 之间（不含端点）')
                if key == 'path.rounded_corner_spacing' and not 0 < number <= 50:
                    raise ValueError('圆角采样间距必须大于 0 且不超过 50 mm')
                if key.startswith('feature.safe_transition_') and not 0 <= number <= 100:
                    raise ValueError('安全点抬离距离必须在 0–100 mm')
                value = format(number, '.9g')
            wanted[key] = value
        if wanted['roi.enable'] not in ('true', 'false') or wanted['path.mode'] not in MODES:
            raise ValueError('ROI 开关或算法模式无效')
        for axis in 'xyz':
            if float(wanted[f'roi.min_{axis}']) > float(wanted[f'roi.max_{axis}']):
                raise ValueError(f'ROI {axis.upper()} 最小值大于最大值')
        if (float(wanted['feature.safe_transition_offset_y']) <= 0 and
                float(wanted['feature.safe_transition_offset_z']) <= 0):
            raise ValueError('安全点 +Y 和 +Z 不可同时为 0')
        changes = {}
        for key, value in wanted.items():
            previous = self.original.get(key, value)
            if (key.startswith('offset.') or
                    (key.startswith('roi.') and key != 'roi.enable') or
                    key in {item[0] for item in WELD_COMMON}):
                # YAML often spells the same number as 1.000 or 188.380. Do not
                # silently rewrite tuned values when the operator did not edit them.
                changed = float(value) != float(previous)
            else:
                changed = value != previous
            if changed:
                changes[key] = value
        return changes

    def current_camera_params(self):
        params = dict(self.camera_params)
        for key, _label in CAMERA_FIELDS:
            value = self.camera_vars[key].get().strip()
            try:
                if key in ('depth_min_mm', 'depth_max_mm', 'capture_max_attempts'):
                    number = int(value)
                    if str(number) != value:
                        raise ValueError()
                else:
                    number = float(value)
                    if not math.isfinite(number):
                        raise ValueError()
            except ValueError as exc:
                raise ValueError(f'相机 {key} 必须输入有效数值') from exc
            params[key] = number
        if not 0 <= params['depth_min_mm'] < params['depth_max_mm']:
            raise ValueError('相机深度范围必须满足 0 ≤ 下限 < 上限')
        if params['depth_gain'] < 0 or params['depth_exposure'] <= 0 or params['depth_frame_time'] <= 0:
            raise ValueError('增益必须 ≥ 0；曝光与帧时间必须 > 0')
        if params['depth_exposure'] > params['depth_frame_time']:
            raise ValueError('曝光时间不能超过帧时间')
        if params['capture_max_attempts'] < 1 or not 0 <= params['min_valid_depth_ratio'] < 1:
            raise ValueError('重拍次数须 ≥ 1；最小有效深度比例须在 [0,1)')
        return params

    def camera_changes(self):
        params = self.current_camera_params()
        return {key: str(params[key]) for key, _label in CAMERA_FIELDS
                if params[key] != self.camera_params[key]}

    def open_viewer(self):
        exe = self.workspace / 'x86_chishine_live_viewer/build/chishine_live_viewer'
        if not exe.is_file():
            messagebox.showerror('预览程序缺失', f'请先构建：{exe}')
            return
        try:
            camera_params = self.current_camera_params()
            roi_values = {key: self.vars[key].get().strip() for key in ROI_KEYS}
            command = viewer_command(
                exe, camera_save_directory(camera_params), self.manifest,
                camera_params, roi_values)
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

    def signature(self, changes, camera_changes=None, handeye_changes=None,
                  matrix_digest=None):
        if self.ply is None:
            file_identity = None
        else:
            stat = self.ply.stat()
            file_identity = (str(self.ply.resolve()), stat.st_size, stat.st_mtime_ns)
        return (file_identity, tuple(sorted(changes.items())),
                tuple(sorted((camera_changes or {}).items())),
                tuple(sorted((handeye_changes or {}).items())), matrix_digest)

    def preview(self):
        try:
            if self.viewer_running or self.ply is None or not self.ply.is_file():
                raise ValueError('请先选择历史 PLY，或在实时预览中按 S 保存并按 Q 退出')
            changes = self.changes()
            camera_changes = self.camera_changes()
            handeye_changes, _matrix, matrix_digest = self.handeye_changes()
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
            self.preview_signature = self.signature(
                changes, camera_changes, handeye_changes, matrix_digest)
            self.preview_rows = rows
            self.preview_warnings = warnings
            self.preview_result_ply = result_ply
            self.preview_history.append({
                'result_ply': result_ply, 'rows': rows,
                'source_ply': self.ply, 'changes': dict(changes),
                'camera_changes': dict(camera_changes),
                'handeye_changes': dict(handeye_changes),
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
            camera_changes = self.camera_changes()
            handeye_changes, _matrix, matrix_digest = self.handeye_changes()
            if ((changes or self.ply is not None) and
                    self.preview_signature != self.signature(
                        changes, camera_changes, handeye_changes, matrix_digest)):
                raise ValueError('已选择 PLY 或修改参数；必须针对当前数值成功预览后才能确认')
            if handeye_changes and (not self.matrix_verified.get() or
                                     self.verified_matrix_digest != matrix_digest):
                raise ValueError('手眼参数已改变：须核对矩阵、方向和单位并勾选标定确认')
            if not handeye_changes and matrix_digest != self.initial_matrix_hash:
                raise ValueError('标定矩阵文件在窗口打开后发生变化；请重新启动以复核')
            current_text = self.config_path.read_text(encoding='utf-8')
            camera_text = self.camera_path.read_text(encoding='utf-8')
            handeye_text = self.handeye_path.read_text(encoding='utf-8')
            if hashlib.sha256(current_text.encode()).hexdigest() != self.original_hash:
                raise ValueError('YAML 被其他程序修改；为避免覆盖请取消并重新启动')
            if hashlib.sha256(camera_text.encode()).hexdigest() != self.camera_hash:
                raise ValueError('camera.yaml 在窗口打开后被修改；请重新启动以核对成像参数')
            if hashlib.sha256(handeye_text.encode()).hexdigest() != self.handeye_hash:
                raise ValueError('handeye_bridge.yaml 在窗口打开后被修改；请重新启动以复核标定')
            proposed_weld = replace_active_overrides(current_text, changes) if changes else current_text
            proposed_camera = replace_camera_parameters(camera_text, camera_changes) if camera_changes else camera_text
            proposed_handeye = (replace_handeye_parameters(handeye_text, handeye_changes)
                                if handeye_changes else handeye_text)
            preview_current = self.preview_signature == self.signature(
                changes, camera_changes, handeye_changes, matrix_digest)
            preview_points = (len(self.preview_rows)
                              if preview_current and self.preview_rows else None)
            warning_text = ('算法提示：' + '\n'.join(self.preview_warnings) + '\n\n'
                            if preview_current and self.preview_warnings else '')
            if not messagebox.askyesno('最终确认',
                    f'确认启动自动链路？\n\n'
                    f'相机配置 {self.camera_path}：{len(camera_changes)} 项\n'
                    f'焊缝配置 {self.config_path}：{len(changes)} 项\n'
                    f'手眼配置 {self.handeye_path}：{len(handeye_changes)} 项\n'
                    f'矩阵：{self.handeye_vars["matrix_file"].get()}\n'
                    f'方向：{self.handeye_vars["matrix_direction"].get()}；'
                    f'平移单位：{self.handeye_vars["matrix_translation_unit"].get()}\n'
                    f'{"当前预览点数 " + str(preview_points) if preview_points else "未使用离线预览，须现场核对"}。\n\n'
                    + warning_text +
                    '确认后启动相机、焊缝、手眼、协调节点；'
                    '之后多条 ABB 请求自动处理，不再逐次询问。',
                    parent=self.root):
                return
            plans = []
            if camera_changes:
                plans.append((self.camera_path, camera_text, proposed_camera,
                              self.session / 'camera.before.yaml'))
            if changes:
                plans.append((self.config_path, current_text, proposed_weld,
                              self.session / 'weld_seam.before.yaml'))
            if handeye_changes:
                plans.append((self.handeye_path, handeye_text, proposed_handeye,
                              self.session / 'handeye_bridge.before.yaml'))
            completed = []
            try:
                for path, before, updated, backup in plans:
                    backup.write_text(before, encoding='utf-8')
                    temporary = path.with_name(path.name + '.operator_tmp')
                    temporary.write_text(updated, encoding='utf-8')
                    os.replace(temporary, path)
                    completed.append((path, before))
                (self.session / 'approval.json').write_text(json.dumps({
                    'approved_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'config': str(self.config_path), 'changes': changes,
                    'camera_config': str(self.camera_path), 'camera_changes': camera_changes,
                    'handeye_config': str(self.handeye_path),
                    'handeye_changes': handeye_changes,
                    'matrix_sha256': matrix_digest,
                    'preview_ply': str(self.ply) if preview_current and self.ply else None,
                    'preview_result_ply': (str(self.preview_result_ply)
                                           if preview_current and self.preview_result_ply else None),
                    'preview_points': preview_points,
                    'config_sha256_after': hashlib.sha256(
                        self.config_path.read_bytes()).hexdigest(),
                    'camera_sha256_after': hashlib.sha256(
                        self.camera_path.read_bytes()).hexdigest(),
                    'handeye_sha256_after': hashlib.sha256(
                        self.handeye_path.read_bytes()).hexdigest(),
                }, ensure_ascii=False, indent=2), encoding='utf-8')
            except Exception:
                for path, before in reversed(completed):
                    temporary = path.with_name(path.name + '.operator_tmp')
                    temporary.write_text(before, encoding='utf-8')
                    os.replace(temporary, path)
                raise
            self.approved = True
            self.root.destroy()
        except Exception as exc:
            messagebox.showerror('不能启动自动链路', str(exc))

    def cancel(self):
        if messagebox.askyesno('取消', '取消人工确认并停止本次 launch？'):
            self.root.destroy()


def main():
    if len(sys.argv) not in (4, 5):
        raise SystemExit('用法: weld_operator_setup <workspace> <weld_yaml> <camera_yaml> [handeye_yaml]')
    window = SetupWindow(*(Path(arg).resolve() for arg in sys.argv[1:]))
    window.root.mainloop()
    return 0 if window.approved else 2


if __name__ == '__main__':
    sys.exit(main())
