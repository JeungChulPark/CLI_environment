#!/usr/bin/env python3
"""노드의 frame_id(stem) 인덱싱을 재현해 bag 의 RGB 프레임을 PNG 로 추출.

노드는 RGB+Depth 동기 콜백마다 `_input_image_index` 를 증가시키고(스킵 프레임 포함),
그 값을 6자리 stem(=frame_id)으로 사용한다. 이 스크립트는 동일한
ApproximateTimeSynchronizer(같은 queue_size/slop)로 같은 카운터를 재현하여
<out>/<index:06d>.png 로 컬러 프레임을 저장한다. → frame_results.csv 의 frame_id 와 정합.

사용 (별도 터미널 2개):
  # T1: 추출 노드
  python3 extract_bag_frames.py --out outputs/validation/SLAM_with_milk_nomilk/frames \
      --queue 30 --slop 0.08
  # T2: bag 재생 (clock 불필요, 추출만 목적)
  ros2 bag play data/milk_nomilk_bag

  특정 프레임만 저장하려면 --frames-list 에 frame_id 목록 파일(한 줄당 1개) 지정.
"""
import argparse
import os

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FrameExtractor(Node):
    def __init__(self, args):
        super().__init__("sam6d_frame_extractor")
        self.bridge = CvBridge()
        self.out = args.out
        os.makedirs(self.out, exist_ok=True)
        self.index = 0
        self.saved = 0
        self.wanted = None
        if args.frames_list and os.path.isfile(args.frames_list):
            with open(args.frames_list) as f:
                self.wanted = {ln.strip() for ln in f if ln.strip()}
            self.get_logger().info(f"{len(self.wanted)}개 지정 프레임만 저장")

        rgb_sub = message_filters.Subscriber(self, Image, args.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, args.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=args.queue, slop=args.slop)
        self.sync.registerCallback(self.cb)
        self.get_logger().info(
            f"동기화 구독: {args.rgb_topic} + {args.depth_topic} "
            f"(queue={args.queue}, slop={args.slop}) → {self.out}")

    def cb(self, rgb_msg, depth_msg):
        stem = f"{self.index:06d}"
        self.index += 1
        if self.wanted is not None and stem not in self.wanted:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"{stem} 변환 실패: {exc}")
            return
        cv2.imwrite(os.path.join(self.out, f"{stem}.png"), rgb)
        self.saved += 1
        if self.saved % 50 == 0:
            self.get_logger().info(f"{self.saved} 프레임 저장 (현재 index={self.index})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    ap.add_argument("--queue", type=int, default=30)
    ap.add_argument("--slop", type=float, default=0.08)
    ap.add_argument("--frames-list", default=None)
    args = ap.parse_args()

    rclpy.init()
    node = FrameExtractor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"총 {node.saved} 프레임 저장 (synced={node.index})")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
