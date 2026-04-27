#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from mdog_interfaces.msg import OwnerIntent
from rclpy.node import Node


class MDogOwnerIntentUi(Node):
    def __init__(self):
        super().__init__("mdog_owner_intent_ui")
        self.declare_parameter("intent_topic", "/mdog/owner_intent")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("default_speed", 0.20)

        self.intent_topic = (
            self.get_parameter("intent_topic").get_parameter_value().string_value
        )
        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.publish_rate = max(
            1.0, self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        self.default_speed = max(
            0.0, self.get_parameter("default_speed").get_parameter_value().double_value
        )

        self.publisher = self.create_publisher(OwnerIntent, self.intent_topic, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.on_cmd_vel, 10
        )
        self.get_logger().info(
            f"publishing owner intent on {self.intent_topic}; watching cmd_vel on {self.cmd_vel_topic}"
        )

        self.root = tk.Tk()
        self.root.title("MDog Owner Intent")
        self.root.geometry("560x500")
        self.root.minsize(500, 460)

        self.command = tk.IntVar(value=OwnerIntent.STOP)
        self.strength = tk.DoubleVar(value=0.7)
        self.speed = tk.DoubleVar(value=self.default_speed)
        self.confidence = tk.DoubleVar(value=1.0)
        self.last_publish = tk.StringVar(value="STOP")
        self.cmd_vel_status = tk.StringVar(value="waiting for cmd_vel")
        self.publish_enabled = tk.BooleanVar(value=True)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<space>", lambda _event: self.set_command(OwnerIntent.STOP))
        self.root.bind("<Escape>", lambda _event: self.set_command(OwnerIntent.STOP))
        self.root.bind("<Up>", lambda _event: self.set_command(OwnerIntent.FORWARD))
        self.root.bind("<Down>", lambda _event: self.set_command(OwnerIntent.BACK))
        self.root.bind("<Left>", lambda _event: self.set_command(OwnerIntent.LEFT))
        self.root.bind("<Right>", lambda _event: self.set_command(OwnerIntent.RIGHT))
        self.root.after(self._period_ms(), self._publish_loop)

    def _period_ms(self):
        return max(20, int(1000.0 / self.publish_rate))

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=(14, 12))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="MDog Owner Intent", font=("", 16, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(header, text=f"topic: {self.intent_topic}").grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Checkbutton(
            header, text="publish", variable=self.publish_enabled
        ).grid(row=0, column=1, rowspan=2, sticky="e")

        body = ttk.Frame(self.root, padding=(14, 2, 14, 8))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(1, weight=1)

        self._add_slider(body, 0, "strength", self.strength, 0.0, 1.0)
        self._add_slider(body, 1, "preferred speed", self.speed, 0.0, 0.4)
        self._add_slider(body, 2, "confidence", self.confidence, 0.0, 1.0)

        buttons = ttk.Frame(self.root, padding=(14, 0, 14, 10))
        buttons.grid(row=2, column=0, sticky="ew")
        for col in range(4):
            buttons.columnconfigure(col, weight=1)

        ttk.Button(
            buttons, text="Forward", command=lambda: self.set_command(OwnerIntent.FORWARD)
        ).grid(row=0, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(
            buttons, text="Left", command=lambda: self.set_command(OwnerIntent.LEFT)
        ).grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(
            buttons, text="Stop", command=lambda: self.set_command(OwnerIntent.STOP)
        ).grid(row=1, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(
            buttons, text="Right", command=lambda: self.set_command(OwnerIntent.RIGHT)
        ).grid(row=1, column=2, sticky="ew", padx=4, pady=4)
        ttk.Button(
            buttons, text="Back", command=lambda: self.set_command(OwnerIntent.BACK)
        ).grid(row=2, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(
            buttons, text="Turn L", command=lambda: self.set_command(OwnerIntent.TURN_LEFT)
        ).grid(row=0, column=3, sticky="ew", padx=4, pady=4)
        ttk.Button(
            buttons, text="Turn R", command=lambda: self.set_command(OwnerIntent.TURN_RIGHT)
        ).grid(row=2, column=3, sticky="ew", padx=4, pady=4)

        footer = ttk.Frame(self.root, padding=(14, 0, 14, 12))
        footer.grid(row=3, column=0, sticky="ew")
        footer.columnconfigure(1, weight=1)
        ttk.Label(footer, text="last intent:").grid(row=0, column=0, sticky="w")
        ttk.Label(footer, textvariable=self.last_publish).grid(
            row=0, column=1, sticky="w", padx=(8, 0)
        )
        ttk.Label(footer, text="cmd_vel:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(footer, textvariable=self.cmd_vel_status).grid(
            row=1, column=1, sticky="w", padx=(8, 0), pady=(8, 0)
        )
        ttk.Label(footer, text=f"watching: {self.cmd_vel_topic}").grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(4, 0)
        )

    def _add_slider(self, parent, row, label, variable, minimum, maximum):
        value_label = ttk.Label(parent, width=7, anchor="e")

        def update_label(*_args):
            value_label.configure(text=f"{variable.get(): .2f}")

        ttk.Label(parent, text=label, width=16).grid(row=row, column=0, sticky="w", pady=8)
        ttk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            orient="horizontal",
            variable=variable,
            command=lambda _value: update_label(),
        ).grid(row=row, column=1, sticky="ew", padx=8)
        value_label.grid(row=row, column=2, sticky="e")
        update_label()

    def set_command(self, command):
        self.command.set(command)
        self.publish_once()

    def command_name(self, command):
        names = {
            OwnerIntent.STOP: "STOP",
            OwnerIntent.FORWARD: "FORWARD",
            OwnerIntent.BACK: "BACK",
            OwnerIntent.LEFT: "LEFT",
            OwnerIntent.RIGHT: "RIGHT",
            OwnerIntent.TURN_LEFT: "TURN_LEFT",
            OwnerIntent.TURN_RIGHT: "TURN_RIGHT",
        }
        return names.get(command, "UNKNOWN")

    def make_intent(self):
        msg = OwnerIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "owner"
        msg.command = int(self.command.get())
        msg.strength = float(self.strength.get())
        msg.preferred_speed = float(self.speed.get())
        msg.confidence = float(self.confidence.get())
        msg.source = "mdog_owner_intent_ui"
        return msg

    def on_cmd_vel(self, msg):
        self.cmd_vel_status.set(
            "lin x={:+.2f} y={:+.2f} z={:+.2f}   ang z={:+.2f}".format(
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z,
            )
        )

    def publish_once(self):
        if not self.publish_enabled.get():
            return
        msg = self.make_intent()
        self.publisher.publish(msg)
        self.last_publish.set(
            f"{self.command_name(msg.command)}  strength={msg.strength:.2f}  speed={msg.preferred_speed:.2f}"
        )

    def _publish_loop(self):
        self.publish_once()
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(self._period_ms(), self._publish_loop)

    def close(self):
        self.command.set(OwnerIntent.STOP)
        self.publish_once()
        self.root.after(50, self.root.destroy)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = None
    try:
        node = MDogOwnerIntentUi()
        node.run()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
