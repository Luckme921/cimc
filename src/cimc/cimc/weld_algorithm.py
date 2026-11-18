"""Shared operator selection/validation; no ROS, camera or robot side effects."""
from pathlib import Path
import math

BACKENDS = ('corrugated', 'gooseneck')
GOOSENECK_ROLES = (
    ('safe_start', '起始安全点'), ('weld_start', '起焊点'),
    ('corner_1', '拐点 1'), ('corner_2', '拐点 2'),
    ('corner_3', '拐点 3'), ('corner_4', '拐点 4'),
    ('weld_end', '末焊点'), ('safe_end', '末尾安全点'),
)
GOOSENECK_FIELDS = ('dx', 'dy', 'dz', 'roll', 'pitch', 'yaw')
GOOSENECK_DEFAULTS = {
    'gooseneck.vertical_length_mm': '40', 'gooseneck.clearance_mm': '30',
    'gooseneck.groove_index': '1', 'gooseneck.tool_x_positive': 'true',
    'gooseneck.tool_z_to_body': 'false',
    'gooseneck.body_tilt_deg': '0',
    'gooseneck.side_lean_deg': '0',
    **{f'gooseneck.{role}.{field}': '0'
       for role, _ in GOOSENECK_ROLES for field in GOOSENECK_FIELDS},
}


def validate_gooseneck(values):
    for key, default in GOOSENECK_DEFAULTS.items():
        value = str(values.get(key, default)).strip()
        if default in ('true', 'false'):
            if value not in ('true', 'false'):
                raise ValueError(f'{key} 必须为 true / false')
            continue
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f'{key} 必须为有限数值')
        if key.endswith('vertical_length_mm'):
            valid = 1 <= number <= 10000
        elif key.endswith('clearance_mm'):
            valid = 0 < number <= 1000
        elif key.endswith('groove_index'):
            valid = number.is_integer() and 1 <= number <= 100
        elif key.endswith('body_tilt_deg'):
            valid = -30 <= number <= 30
        elif key.endswith('side_lean_deg'):
            valid = 0 <= number <= 20
        else:
            valid = abs(number) <= (100 if key.endswith(('.dx', '.dy', '.dz')) else 180)
        if not valid:
            raise ValueError(f'鹅颈槽参数超出范围：{key}')


def select_backend(overrides):
    name = overrides.get('pipeline.algorithm', 'corrugated')
    if name not in BACKENDS:
        raise ValueError('pipeline.algorithm 必须为 corrugated / gooseneck')
    if name == 'gooseneck':
        parameters = {key: value for key, value in overrides.items()
                      if key.startswith(('gooseneck.', 'roi.'))}
        validate_gooseneck(parameters)
    else:
        parameters = {key: value for key, value in overrides.items()
                      if key != 'pipeline.algorithm' and not key.startswith('gooseneck.')}
    return name, parameters


def preview_command(workspace, source, session, prefix, overrides, config_file=''):
    name, parameters = select_backend(overrides)
    relative = ('gooseneck_weld/install/bin/gooseneck_weld_extractor'
                if name == 'gooseneck' else 'x86_weld_seam_test/install/bin/weld_seam_extractor')
    executable = Path(workspace) / relative
    if not executable.is_file():
        raise FileNotFoundError(f'请先构建并安装对应 SDK：{executable}')
    command = [str(executable), str(source), str(session), prefix]
    if config_file:
        command += ['--config', str(config_file)]
    for key, value in parameters.items():
        command += ['--set', f'{key}={value}']
    return command
