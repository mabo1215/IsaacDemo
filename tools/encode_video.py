#!/usr/bin/env python3
"""Encode Isaac Sim PNG frames into a small H.264 MP4 using PyAV."""

from __future__ import annotations

import argparse
from pathlib import Path

import av
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
    print(f'encoded {len(files)} frames -> {args.output}')


if __name__ == '__main__':
    main()
