#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Go2VelController(Node):
    def __init__(self):
        super().__init__("go2_vel_controller")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 2.0)

        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.publish_rate = max(
            1.0, self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        self.max_linear = max(
            0.1, self.get_parameter("max_linear").get_parameter_value().double_value
        )
        self.max_angular = max(
            0.1, self.get_parameter("max_angular").get_parameter_value().double_value
        )

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.get_logger().info(
            f"go2_vel_controller publishing Twist on {self.cmd_vel_topic}"
        )

        self.root = tk.Tk()
        self.root.title("Go2 Velocity Controller")
        self.root.geometry("520x520")
        self.root.minsize(460, 460)

        self.vars = {
            "linear.x": tk.DoubleVar(value=0.0),
            "linear.y": tk.DoubleVar(value=0.0),
            "angular.z": tk.DoubleVar(value=0.0),
            "linear.z": tk.DoubleVar(value=0.0),
            "angular.x": tk.DoubleVar(value=0.0),
            "angular.y": tk.DoubleVar(value=0.0),
        }
        self.publish_enabled = tk.BooleanVar(value=True)
        self.last_publish = tk.StringVar(value="0.00, 0.00, 0.00")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<space>", lambda _event: self.stop())
        self.root.bind("<Escape>", lambda _event: self.stop())
        self.root.after(self._period_ms(), self._publish_loop)

    def _period_ms(self):
        return max(20, int(1000.0 / self.publish_rate))

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        header = ttk.Frame(self.root, padding=(14, 12))
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(header, text="Go2 Velocity Controller", font=("", 16, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(header, text=f"topic: {self.cmd_vel_topic}").grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )
        ttk.Checkbutton(
            header, text="publish", variable=self.publish_enabled
        ).grid(row=0, column=1, rowspan=2, sticky="e")

        body = ttk.Frame(self.root, padding=(14, 0, 14, 8))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(1, weight=1)

        row = 0
        for name, limit in (
            ("linear.x", self.max_linear),
            ("linear.y", self.max_linear),
            ("angular.z", self.max_angular),
            ("linear.z", self.max_linear),
            ("angular.x", self.max_angular),
            ("angular.y", self.max_angular),
        ):
            self._add_slider(body, row, name, limit)
            row += 1

        buttons = ttk.Frame(self.root, padding=(14, 0, 14, 10))
        buttons.grid(row=2, column=0, sticky="ew")
        for col in range(4):
            buttons.columnconfigure(col, weight=1)

        ttk.Button(buttons, text="Forward", command=lambda: self.preset(0.25, 0.0, 0.0)).grid(
            row=0, column=1, sticky="ew", padx=4, pady=4
        )
        ttk.Button(buttons, text="Left", command=lambda: self.preset(0.0, 0.2, 0.0)).grid(
            row=1, column=0, sticky="ew", padx=4, pady=4
        )
        ttk.Button(buttons, text="Stop", command=self.stop).grid(
            row=1, column=1, sticky="ew", padx=4, pady=4
        )
        ttk.Button(buttons, text="Right", command=lambda: self.preset(0.0, -0.2, 0.0)).grid(
            row=1, column=2, sticky="ew", padx=4, pady=4
        )
        ttk.Button(buttons, text="Back", command=lambda: self.preset(-0.25, 0.0, 0.0)).grid(
            row=2, column=1, sticky="ew", padx=4, pady=4
        )
        ttk.Button(buttons, text="Turn L", command=lambda: self.preset(0.0, 0.0, 0.6)).grid(
            row=0, column=3, sticky="ew", padx=4, pady=4
        )
        ttk.Button(buttons, text="Turn R", command=lambda: self.preset(0.0, 0.0, -0.6)).grid(
            row=2, column=3, sticky="ew", padx=4, pady=4
        )

        footer = ttk.Frame(self.root, padding=(14, 0, 14, 12))
        footer.grid(row=3, column=0, sticky="ew")
        footer.columnconfigure(1, weight=1)
        ttk.Label(footer, text="last x/y/yaw:").grid(row=0, column=0, sticky="w")
        ttk.Label(footer, textvariable=self.last_publish).grid(row=0, column=1, sticky="w")

    def _add_slider(self, parent, row, name, limit):
        value_label = ttk.Label(parent, width=7, anchor="e")

        def update_label(*_args):
            value_label.configure(text=f"{self.vars[name].get(): .2f}")

        ttk.Label(parent, text=name, width=11).grid(
            row=row, column=0, sticky="w", pady=7
        )
        scale = ttk.Scale(
            parent,
            from_=-limit,
            to=limit,
            orient="horizontal",
            variable=self.vars[name],
            command=lambda _value: update_label(),
        )
        scale.grid(row=row, column=1, sticky="ew", padx=8)
        value_label.grid(row=row, column=2, sticky="e")
        ttk.Button(parent, text="0", width=3, command=lambda: self.vars[name].set(0.0)).grid(
            row=row, column=3, sticky="e", padx=(8, 0)
        )
        update_label()

    def preset(self, linear_x, linear_y, angular_z):
        self.vars["linear.x"].set(linear_x)
        self.vars["linear.y"].set(linear_y)
        self.vars["angular.z"].set(angular_z)
        self.vars["linear.z"].set(0.0)
        self.vars["angular.x"].set(0.0)
        self.vars["angular.y"].set(0.0)
        self.publish_enabled.set(True)
        self.publish_once()

    def stop(self):
        for var in self.vars.values():
            var.set(0.0)
        self.publish_once()

    def make_twist(self):
        msg = Twist()
        msg.linear.x = float(self.vars["linear.x"].get())
        msg.linear.y = float(self.vars["linear.y"].get())
        msg.linear.z = float(self.vars["linear.z"].get())
        msg.angular.x = float(self.vars["angular.x"].get())
        msg.angular.y = float(self.vars["angular.y"].get())
        msg.angular.z = float(self.vars["angular.z"].get())
        return msg

    def publish_once(self):
        msg = self.make_twist()
        self.publisher.publish(msg)
        self.last_publish.set(
            f"{msg.linear.x: .2f}, {msg.linear.y: .2f}, {msg.angular.z: .2f}"
        )

    def _publish_loop(self):
        if self.publish_enabled.get():
            self.publish_once()
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(self._period_ms(), self._publish_loop)

    def close(self):
        self.stop()
        self.root.after(50, self.root.destroy)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = None
    try:
        node = Go2VelController()
        node.run()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
