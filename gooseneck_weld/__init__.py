"""Python convenience API over the same native SDK used by ROS."""
import csv
from pathlib import Path
import subprocess

ROLES = (
    ('safe_start', '起始安全点'), ('weld_start', '起焊点'),
    ('corner_1', '拐点 1'), ('corner_2', '拐点 2'),
    ('corner_3', '拐点 3'), ('corner_4', '拐点 4'),
    ('weld_end', '末焊点'), ('safe_end', '末尾安全点'),
)
OFFSET_FIELDS = ('dx', 'dy', 'dz', 'roll', 'pitch', 'yaw')


def run_preview(workspace, session, prefix, source, settings, log_path):
    workspace, session = Path(workspace).resolve(), Path(session).resolve()
    executable = workspace / 'gooseneck_weld/install/bin/gooseneck_weld_extractor'
    if not executable.is_file():
        raise FileNotFoundError(f'请先构建和安装鹅颈槽 SDK：{executable}')
    if not prefix or prefix in ('.', '..') or Path(prefix).name != prefix:
        raise ValueError('输出前缀必须是单个文件名')
    session.mkdir(parents=True, exist_ok=True)
    command = [str(executable), str(Path(source).resolve()), str(session), prefix]
    for key, value in settings.items():
        command += ['--set', f'{key}={value}']
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open('w', encoding='utf-8') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                timeout=120, check=False)
    if result.returncode:
        raise RuntimeError(log_path.read_text(encoding='utf-8', errors='replace')[-2000:])
    with (session / f'{prefix}_features.csv').open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    return session / f'{prefix}_result.ply', rows
