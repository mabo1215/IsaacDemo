#!/usr/bin/env python3
"""Encode Isaac Sim PNG frames into a small H.264-compatible MP4."""

from __future__ import annotations

import argparse
from pathlib import Path

try:
    import av
except ImportError:  # Isaac Sim's Python image stack may not include PyAV.
    av = None
import numpy as np
from PIL import Image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('frames', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()

    files = sorted(args.frames.rglob('*.png'))
    if not files:
        raise SystemExit(f'no PNG frames found under {args.frames}')

    first = Image.open(files[0]).convert('RGB')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if av is not None:
        container = av.open(str(args.output), mode='w')
        stream = container.add_stream('libx264', rate=args.fps)
        stream.width, stream.height = first.size
        stream.pix_fmt = 'yuv420p'
        stream.options = {'crf': '23', 'preset': 'veryfast'}

        for path in files:
            with Image.open(path) as image:
                frame = av.VideoFrame.from_image(image.convert('RGB'))
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
        container.close()
    else:
        import cv2

        width, height = first.size
        writer = cv2.VideoWriter(
            str(args.output),
            cv2.VideoWriter_fourcc(*'mp4v'),
            args.fps,
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f'could not open video writer for {args.output}')
        try:
            for path in files:
                with Image.open(path) as image:
                    rgb = np.asarray(image.convert('RGB'))
                writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        finally:
            writer.release()
    print(f'encoded {len(files)} frames -> {args.output}')


if __name__ == '__main__':
    main()
