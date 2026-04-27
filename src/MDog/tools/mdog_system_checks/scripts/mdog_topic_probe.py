#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node


DEFAULT_TOPICS = [
    "/mdog/dog_body",
    "/mdog/owner_body",
    "/mdog/fused_points",
    "/mdog/height_grid",
    "/mdog/local_grid",
    "/mdog/local_occupancy",
    "/mdog/traversability",
    "/mdog/nav_decision",
    "/mdog/nav_feedback",
    "/cmd_vel",
]


class TopicProbe(Node):
    def __init__(self):
        super().__init__("mdog_topic_probe")

    def topic_names(self):
        return {name for name, _types in self.get_topic_names_and_types()}


def main():
    parser = argparse.ArgumentParser(description="Check whether core MDog topics are visible.")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("topics", nargs="*", default=DEFAULT_TOPICS)
    args = parser.parse_args()

    rclpy.init()
    node = TopicProbe()
    deadline = time.monotonic() + max(0.1, args.timeout)
    seen = set()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            seen = node.topic_names()
            if all(topic in seen for topic in args.topics):
                break
        missing = [topic for topic in args.topics if topic not in seen]
        for topic in args.topics:
            print(f"{'OK' if topic in seen else 'MISSING'} {topic}")
        return 1 if missing else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
