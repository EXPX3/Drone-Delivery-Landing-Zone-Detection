#!/usr/bin/env python3
"""Extract uniformly spaced camera frames for semantic-model evaluation."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from project_lz_into_bag_camera import (  # noqa: E402
    decode_image, open_reader, require_ros, stamp_ns,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/GremsyVIO/image/cpu")
    parser.add_argument("--interval", type=int, default=60)
    parser.add_argument("--maximum", type=int, default=120)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    _, deserialize_message, get_message = require_ros()
    reader = open_reader(args.bag)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_type = get_message(topic_types[args.topic])
    seen = written = 0
    while reader.has_next() and written < args.maximum:
        topic, raw, bag_time = reader.read_next()
        if topic != args.topic:
            continue
        if seen % args.interval == 0:
            message = deserialize_message(raw, message_type)
            timestamp = stamp_ns(message, bag_time)
            frame = decode_image(message)
            path = args.output / f"frame_{seen:06d}_{timestamp}.jpg"
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 94]):
                raise RuntimeError(f"Failed to write {path}")
            written += 1
        seen += 1
    print(f"Extracted {written} of {seen} visited frames to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
