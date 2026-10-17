#!/usr/bin/env python3
"""Operator dashboard: subscribers only; never publishes robot commands."""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from cimc.weld_operator_status_model import StatusModel, TOPIC_LABELS
from cimc.weld_operator_setup import maximize_window


class StatusSubscriber(Node):
    def __init__(self, outbox):
        super().__init__('weld_operator_status_gui')
        self.outbox = outbox
        for topic in TOPIC_LABELS:
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.RELIABLE
            if topic in ('/abb/connection_status', '/weld_task/status'):
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                String, topic,
                lambda msg, name=topic: self._received(name, msg.data), qos)

    def _received(self, topic, data):
        try:
            self.outbox.put_nowait((topic, data))
        except queue.Full:
            try:
                self.outbox.get_nowait()
                self.outbox.put_nowait((topic, data))
            except queue.Empty:
                pass


class StatusWindow:
    def __init__(self, root):
        self.root = root
        self.model = StatusModel()
        self.inbox = queue.Queue(maxsize=600)
        root.title('华南理工大学智能焊接实验室 · 运行状态')
        root.geometry('1200x800')
        root.minsize(900, 600)
        maximize_window(root)
        root.configure(background='#f5f8fb')
        style = ttk.Style(root)
        if 'clam' in style.theme_names():
            style.theme_use('clam')
        style.configure('TFrame', background='#f5f8fb')
        style.configure('TLabel', background='#f5f8fb', foreground='#1c2b39',
                        font=('Noto Sans CJK SC', 10))
        style.configure('Title.TLabel', background='#f5f8fb', foreground='#133a56',
                        font=('Noto Sans CJK SC', 18, 'bold'))
        style.configure('Hint.TLabel', background='#f5f8fb', foreground='#8a4b00')
        outer = ttk.Frame(root, padding=16)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='华南理工大学智能焊接实验室 · 运行状态',
                  style='Title.TLabel').pack(anchor='w')
        ttk.Label(outer, text=(
            '此窗口只订阅 ROS2 状态，不发送运动命令。轨迹准备、socket 写入成功、'
            'ABB ACK 和机器人实际执行是四件不同的事。'),
            style='Hint.TLabel', wraplength=1140).pack(anchor='w', pady=(5, 12))
        cards = ttk.Frame(outer)
        cards.pack(fill='x')
        self.fields = {}
        definitions = (
            ('connection', 'ABB TCP'), ('stage', '任务阶段'),
            ('received', '最近收到 ABB'), ('prepared', '最近准备发往 ABB'),
            ('sent', 'socket 发送结果'), ('ack', '最近 ABB ACK'),
            ('algorithm', '焊缝算法时间'), ('timing', '本次任务耗时'),
            ('statistics', '成功任务统计'),
        )
        for index, (field, label) in enumerate(definitions):
            frame = ttk.LabelFrame(cards, text=label, padding=8)
            frame.grid(row=index // 3, column=index % 3, sticky='nsew', padx=4, pady=4)
            value = tk.StringVar(value=getattr(self.model, field))
            ttk.Label(frame, textvariable=value, wraplength=350).pack(anchor='w')
            self.fields[field] = value
        for column in range(3):
            cards.columnconfigure(column, weight=1)

        ttk.Label(outer, text='事件时间线（最近 300 条；点选查看完整原文）').pack(
            anchor='w', pady=(12, 4))
        table_frame = ttk.Frame(outer)
        table_frame.pack(fill='both', expand=True)
        self.table = ttk.Treeview(table_frame, columns=('time', 'type', 'summary'),
                                  show='headings', selectmode='browse')
        for column, label, width in (
                ('time', '时间', 90), ('type', '来源', 175), ('summary', '概要', 800)):
            self.table.heading(column, text=label)
            self.table.column(column, width=width, stretch=(column == 'summary'))
        self.table.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(table_frame, orient='vertical', command=self.table.yview)
        scrollbar.pack(side='right', fill='y')
        self.table.configure(yscrollcommand=scrollbar.set)
        self.table.bind('<<TreeviewSelect>>', self.show_detail)
        ttk.Label(outer, text='原始话题消息（仅显示，不作为 ABB 接收确认）').pack(
            anchor='w', pady=(8, 4))
        self.detail = tk.Text(outer, height=7, wrap='word', state='disabled',
                              background='white', foreground='#1c2b39')
        self.detail.pack(fill='x')
        self.events_by_id = {}
        self.next_id = 0
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.ros_node = None
        self.ros_thread = None
        self.root.after(100, self.drain)

    def start_ros(self):
        rclpy.init()
        self.ros_node = StatusSubscriber(self.inbox)
        self.ros_thread = threading.Thread(
            target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

    def drain(self):
        for _ in range(100):
            try:
                topic, data = self.inbox.get_nowait()
            except queue.Empty:
                break
            event = self.model.accept(topic, data)
            iid = str(self.next_id)
            self.next_id += 1
            self.events_by_id[iid] = event
            self.table.insert('', 'end', iid=iid, values=event[:3])
            children = self.table.get_children()
            if len(children) > 300:
                self.table.delete(children[0])
                self.events_by_id.pop(children[0], None)
            self.table.see(iid)
        for field, value in self.fields.items():
            value.set(getattr(self.model, field))
        self.root.after(100, self.drain)

    def show_detail(self, _event=None):
        selected = self.table.selection()
        if not selected:
            return
        event = self.events_by_id.get(selected[0])
        if event is None:
            return
        self.detail.configure(state='normal')
        self.detail.delete('1.0', 'end')
        self.detail.insert('1.0', event[3])
        self.detail.configure(state='disabled')

    def close(self):
        if rclpy.ok():
            rclpy.shutdown()
        if self.ros_thread is not None:
            self.ros_thread.join(timeout=1.0)
        if self.ros_node is not None:
            self.ros_node.destroy_node()
        self.root.destroy()


def main():
    root = tk.Tk()
    window = StatusWindow(root)
    window.start_ros()
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
