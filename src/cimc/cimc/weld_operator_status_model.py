"""Read-only, hardware-independent summary of the welding ROS status topics."""

import json
from datetime import datetime


TOPIC_LABELS = {
    '/abb/connection_status': 'ABB 连接',
    '/abb/raw_text': 'ABB → 本机',
    '/abb/tx_text': '本机 → ABB（待发送）',
    '/abb/tx_status': 'ABB 发送结果',
    '/abb/task_timing': '任务耗时',
    '/weld_task/status': '任务阶段',
    '/weld_seam/status': '焊缝算法',
    '/handeye_bridge/status': '手眼转换',
}


class StatusModel:
    def __init__(self):
        self.connection = '等待连接状态'
        self.stage = '等待任务'
        self.received = '—'
        self.prepared = '—'
        self.sent = '—'
        self.algorithm = '—'
        self.timing = '—'
        self.statistics = '—'
        self.ack = '—'
        self.events = []

    def accept(self, topic, data, timestamp=None):
        """Consume one String payload. A prepared TX is not proof of delivery."""
        timestamp = timestamp or datetime.now().strftime('%H:%M:%S')
        detail = str(data)
        summary = detail.splitlines()[0][:180] if detail else '(空消息)'
        try:
            body = json.loads(detail)
        except (ValueError, TypeError):
            body = None
        if topic == '/abb/connection_status' and isinstance(body, dict):
            state = body.get('state', 'unknown')
            self.connection = {
                'listening': '监听中，等待 ABB',
                'connected': '已连接',
                'disconnected': '已断开',
                'listen_error': '监听失败',
            }.get(state, str(state))
            if state == 'connected':
                self.connection += f" · {body.get('peer', '')}"
            summary = self.connection
        elif topic == '/abb/raw_text':
            self.received = summary
            if summary.startswith('START_CAPTURE:'):
                self.ack = '等待本次 ABB ACK'
                self.prepared = '等待本次轨迹'
                self.sent = '等待本次发送结果'
                self.algorithm = '等待本次算法结果'
                self.timing = '等待本次任务耗时'
            elif summary.startswith('TRAJECTORY_ACK:'):
                self.ack = summary
        elif topic == '/abb/tx_text':
            lines = detail.splitlines()
            self.prepared = f'{lines[0] if lines else "(空消息)"} · {len(lines)} 行'
            summary = self.prepared
        elif topic == '/abb/tx_status':
            self.sent = summary
        elif topic == '/weld_task/status' and isinstance(body, dict):
            self.stage = f"{body.get('state', '?')} · task_id={body.get('task_id', '?')}"
            summary = f"{self.stage} · {body.get('message', '')}"[:180]
        elif topic == '/weld_seam/status' and isinstance(body, dict):
            self.algorithm = (
                f"状态={body.get('status', '?')} · 路径={body.get('path_points', '?')} 点"
                f" · 总耗时={body.get('total_ms', '?')} ms")
            summary = self.algorithm
        elif topic == '/handeye_bridge/status' and isinstance(body, dict):
            summary = (f"成功={body.get('success', '?')} · 轨迹={body.get('count', '?')} 点"
                       f" · {body.get('message', '')}")[:180]
        elif topic == '/abb/task_timing' and isinstance(body, dict):
            self.timing = (f"task_id={body.get('task_id', '?')} · "
                           f"{body.get('elapsed_ms', '?')} ms · "
                           f"{'成功' if body.get('success') else '失败'}")
            self.statistics = (f"成功 {body.get('success_count', '?')} 次 · "
                               f"平均 {body.get('average_ms', '?')} ms · "
                               f"最小/最大 {body.get('minimum_ms', '?')}/"
                               f"{body.get('maximum_ms', '?')} ms")
            summary = self.timing
        event = (timestamp, TOPIC_LABELS.get(topic, topic), summary, detail)
        self.events.append(event)
        if len(self.events) > 300:
            del self.events[:len(self.events) - 300]
        return event
