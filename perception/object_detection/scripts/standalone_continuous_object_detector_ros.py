#!/usr/bin/env python3
"""
Standalone object detector that consumes ROS image topics instead of direct ZED SDK.
Shows 2D bounding boxes in an OpenCV window.
"""

import argparse
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def load_classes(path: str) -> List[str]:
    classes: List[str] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            label = line.strip()
            if not label:
                continue
            if label.startswith("#"):
                continue
            classes.append(label)
    return classes


def load_filter(path: Optional[str]) -> Dict[int, str]:
    if not path:
        return {}
    mapping: Dict[int, str] = {}
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            raw = line.strip()
            if not raw or raw.startswith("#"):
                continue
            if ":" not in raw:
                continue
            class_id, name = raw.split(":", 1)
            mapping[int(class_id.strip())] = name.strip()
    return mapping


def preprocess_image(image: np.ndarray, input_size: int) -> Tuple[np.ndarray, float, Tuple[int, int]]:
    original_shape = image.shape[:2]
    scale = min(input_size / original_shape[0], input_size / original_shape[1])
    new_shape = (int(original_shape[1] * scale), int(original_shape[0] * scale))
    resized = cv2.resize(image, new_shape, interpolation=cv2.INTER_LINEAR)

    pad_x = (input_size - new_shape[0]) // 2
    pad_y = (input_size - new_shape[1]) // 2
    padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    padded[pad_y : pad_y + new_shape[1], pad_x : pad_x + new_shape[0]] = resized

    tensor = padded.transpose(2, 0, 1).astype(np.float32) / 255.0
    tensor = np.expand_dims(tensor, axis=0)
    return tensor, scale, (pad_x, pad_y)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> List[int]:
    if len(boxes) == 0:
        return []

    indices = np.argsort(scores)[::-1]
    keep: List[int] = []
    while len(indices) > 0:
        current = int(indices[0])
        keep.append(current)
        if len(indices) == 1:
            break

        current_box = boxes[current]
        remaining = boxes[indices[1:]]
        x1 = np.maximum(current_box[0], remaining[:, 0])
        y1 = np.maximum(current_box[1], remaining[:, 1])
        x2 = np.minimum(current_box[2], remaining[:, 2])
        y2 = np.minimum(current_box[3], remaining[:, 3])
        intersection = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)

        current_area = (current_box[2] - current_box[0]) * (current_box[3] - current_box[1])
        remaining_areas = (remaining[:, 2] - remaining[:, 0]) * (remaining[:, 3] - remaining[:, 1])
        union = current_area + remaining_areas - intersection
        iou = intersection / (union + 1e-6)
        indices = indices[1:][iou <= iou_threshold]
    return keep


class ROSObjectDetector(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("standalone_object_detector_ros")
        self.bridge = CvBridge()
        self.args = args
        self.last_frame_time = 0.0
        self.frame_count = 0
        self.fps_time = time.time()

        try:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            self.session = ort.InferenceSession(args.onnx, providers=providers)
        except Exception:
            self.session = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [out.name for out in self.session.get_outputs()]

        self.classes = load_classes(args.classes)
        self.urc_filter = load_filter(args.urc_filter)
        self.get_logger().info(
            f"Using image topic '{args.image_topic}', classes={len(self.classes)}, filter={len(self.urc_filter)}"
        )

        self.create_subscription(Image, args.image_topic, self.image_callback, 5)

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        tensor, scale, pads = preprocess_image(frame, self.args.img_size)
        outputs = self.session.run(self.output_names, {self.input_name: tensor})
        detections = self.postprocess(outputs, scale, pads, frame.shape[:2])
        rendered = self.render(frame, detections)
        cv2.imshow("Object Detector (ROS Topic)", rendered)
        if cv2.waitKey(1) in (27, ord("q"), ord("Q")):
            rclpy.shutdown()
            return

        self.last_frame_time = time.time()
        self.frame_count += 1
        if self.last_frame_time - self.fps_time >= 2.0:
            fps = self.frame_count / (self.last_frame_time - self.fps_time)
            self.get_logger().info(f"FPS: {fps:.2f} detections: {len(detections)}")
            self.fps_time = self.last_frame_time
            self.frame_count = 0

    def postprocess(
        self, outputs: List[np.ndarray], scale: float, pads: Tuple[int, int], original_shape: Tuple[int, int]
    ) -> List[Dict[str, float]]:
        predictions = outputs[0]
        if predictions.ndim == 3 and predictions.shape[0] == 1:
            predictions = predictions[0]
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        boxes = predictions[:, :4]
        class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = np.max(class_scores, axis=1)
        mask = confidences >= self.args.conf_thres
        boxes = boxes[mask]
        class_ids = class_ids[mask]
        confidences = confidences[mask]
        if len(boxes) == 0:
            return []

        pad_x, pad_y = pads
        boxes[:, 0] = (boxes[:, 0] - pad_x) / scale
        boxes[:, 1] = (boxes[:, 1] - pad_y) / scale
        boxes[:, 2] = boxes[:, 2] / scale
        boxes[:, 3] = boxes[:, 3] / scale

        x1 = np.clip(boxes[:, 0] - boxes[:, 2] / 2, 0, original_shape[1] - 1)
        y1 = np.clip(boxes[:, 1] - boxes[:, 3] / 2, 0, original_shape[0] - 1)
        x2 = np.clip(boxes[:, 0] + boxes[:, 2] / 2, 0, original_shape[1] - 1)
        y2 = np.clip(boxes[:, 1] + boxes[:, 3] / 2, 0, original_shape[0] - 1)

        nms_boxes = np.column_stack([x1, y1, x2, y2])
        keep = nms(nms_boxes, confidences, self.args.iou_thres)

        detections: List[Dict[str, float]] = []
        for idx in keep:
            class_id = int(class_ids[idx])
            if self.urc_filter and class_id not in self.urc_filter:
                continue
            if class_id >= len(self.classes):
                continue
            label = self.urc_filter.get(class_id, self.classes[class_id])
            detections.append(
                {
                    "x1": float(x1[idx]),
                    "y1": float(y1[idx]),
                    "x2": float(x2[idx]),
                    "y2": float(y2[idx]),
                    "confidence": float(confidences[idx]),
                    "label": label,
                    "class_id": class_id,
                }
            )
        return detections

    def render(self, frame: np.ndarray, detections: List[Dict[str, float]]) -> np.ndarray:
        rendered = frame.copy()
        for det in detections:
            x1, y1, x2, y2 = int(det["x1"]), int(det["y1"]), int(det["x2"]), int(det["y2"])
            cv2.rectangle(rendered, (x1, y1), (x2, y2), (0, 220, 0), 2)
            caption = f"{det['label']} {det['confidence']:.2f}"
            cv2.putText(rendered, caption, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
        return rendered


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS-topic-based object detector viewer")
    parser.add_argument("--onnx", required=True, help="Path to ONNX model")
    parser.add_argument("--classes", required=True, help="Path to class file")
    parser.add_argument("--urc-filter", default="", help="Optional class-id filter file (id:name)")
    parser.add_argument("--image-topic", default="/zed2i/zed2i_camera/left/image_rect_color", help="ROS image topic")
    parser.add_argument("--conf-thres", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IoU threshold")
    parser.add_argument("--img-size", type=int, default=640, help="Model input size")
    parser.add_argument("--no-gl", action="store_true", help="Compatibility flag; ignored in ROS mode")
    parser.add_argument("--svo", default="", help="Compatibility flag; ignored in ROS mode")
    args = parser.parse_args()

    if args.svo:
        print("[WARN] --svo is ignored for ROS topic mode.")
    if args.no_gl:
        print("[INFO] --no-gl set; ROS mode is 2D only.")

    rclpy.init()
    node = ROSObjectDetector(args)
    timeout_start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.last_frame_time == 0.0 and time.time() - timeout_start > 5.0:
                node.get_logger().error(f"No frames received on {args.image_topic} after 5s.")
                break
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
