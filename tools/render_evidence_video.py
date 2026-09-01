"""Render a lightweight evidence video from the Isaac trajectory log.

Isaac Sim still produces and simulates the USD scene. This host-side renderer
keeps video creation deterministic on RTX 3070 systems where Isaac Sim 4.5's
Windows Replicator orchestrator is unstable with the installed driver.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import av
from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 960, 540


def font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        Path('C:/Windows/Fonts/arial.ttf'),
        Path('C:/Windows/Fonts/segoeui.ttf'),
        Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


F16 = font(16)
F18 = font(18)
F22 = font(22)
F28 = font(28)


def px(x: float) -> int:
    return int(430 + x * 92)


def pz(z: float) -> int:
    return int(438 - z * 105)


def draw_humanoid(draw: ImageDraw.ImageDraw, board_x: float, board_y: float, label: str) -> None:
    cx, foot = 150, 430
    draw.ellipse((cx - 22, 218, cx + 22, 262), fill=(228, 190, 135), outline=(255, 255, 255), width=2)
    draw.rounded_rectangle((cx - 34, 262, cx + 34, 354), radius=10, fill=(40, 100, 190), outline=(230, 240, 255), width=2)
    target_x = max(220, min(380, px(board_x)))
    target_y = max(260, min(360, pz(1.45)))
    draw.line((cx - 26, 280, target_x, target_y), fill=(245, 205, 145), width=11)
    draw.line((cx + 26, 280, target_x + 18, target_y + 12), fill=(245, 205, 145), width=11)
    draw.line((cx - 18, 354, cx - 34, foot), fill=(35, 60, 110), width=13)
    draw.line((cx + 18, 354, cx + 34, foot), fill=(35, 60, 110), width=13)
    draw.text((82, 452), label, font=F16, fill=(190, 205, 225))


def render(row: dict[str, str], previous_nails: int) -> Image.Image:
    image = Image.new('RGB', (WIDTH, HEIGHT), (16, 22, 32))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, WIDTH, 78), fill=(24, 38, 56))
    draw.text((28, 18), 'Isaac Sim + Genie Sim G2  |  Drywall Installation', font=F28, fill=(235, 242, 250))
    draw.text((30, 52), 'PhysX collision task  /  trajectory evidence', font=F16, fill=(161, 184, 210))

    # Wall and studs.
    wall = (255, 120, 670, 442)
    draw.rectangle(wall, fill=(73, 79, 91), outline=(180, 190, 205), width=2)
    for x in (-2.6, -1.3, 0.0, 1.3, 2.6):
        sx = px(x)
        draw.rectangle((sx - 8, 132, sx + 8, 430), fill=(157, 94, 43), outline=(208, 145, 75))
    draw.text((275, 96), 'WALL + WOOD STUDS', font=F18, fill=(205, 215, 228))

    # Moving drywall board.
    bx = float(row['board_x'])
    board_cx = px(bx)
    board_top = 195
    board_w, board_h = 332, 224
    draw.rectangle((board_cx - board_w // 2, board_top, board_cx + board_w // 2, board_top + board_h),
                   fill=(211, 210, 193), outline=(255, 245, 215), width=3)
    draw.text((board_cx - 68, board_top + 100), 'DRYWALL', font=F22, fill=(110, 108, 97))

    draw_humanoid(draw, bx, float(row['board_y']), row.get('robot_label', 'Humanoid / H1 fallback'))

    # Tool and nozzle.
    gx, gz = float(row['gun_x']), float(row['gun_z'])
    gun_x, gun_z = px(gx), pz(gz)
    draw.rounded_rectangle((gun_x - 20, gun_z - 22, gun_x + 20, gun_z + 30), radius=5,
                           fill=(34, 35, 39), outline=(240, 240, 240), width=2)
    draw.line((gun_x, gun_z + 28, gun_x, gun_z + 48), fill=(235, 180, 50), width=6)
    draw.text((gun_x - 56, gun_z + 56), 'PNEUMATIC GUN', font=F16, fill=(230, 210, 140))

    nail_count = int(row['nails_installed'])
    points = [(-1.20, 1.15), (0.0, 1.15), (1.20, 1.15), (-1.20, 1.85), (0.0, 1.85), (1.20, 1.85)]
    for index, (nx, nz) in enumerate(points):
        x, y = px(nx), pz(nz)
        if index < nail_count:
            draw.ellipse((x - 7, y - 7, x + 7, y + 7), fill=(215, 45, 38), outline=(255, 210, 180), width=2)
            draw.line((x - 11, y, x + 11, y), fill=(255, 220, 190), width=2)
        else:
            draw.ellipse((x - 4, y - 4, x + 4, y + 4), outline=(125, 133, 145), width=1)

    # Right-side telemetry/status panel.
    draw.rounded_rectangle((690, 104, 936, 444), radius=10, fill=(27, 37, 51), outline=(89, 111, 139), width=2)
    state = row['state'].replace('_', ' ').upper()
    state_color = (91, 220, 151) if state == 'FASTENING' else (246, 195, 83)
    draw.text((712, 124), 'SIMULATION STATUS', font=F18, fill=(220, 230, 242))
    draw.text((712, 158), state, font=F22, fill=state_color)
    draw.text((712, 202), f"Frame   {row['frame']}", font=F18, fill=(195, 210, 228))
    draw.text((712, 230), f"Time    {float(row['time_s']):5.1f} s", font=F18, fill=(195, 210, 228))
    draw.text((712, 274), 'PHYSX COLLISION', font=F16, fill=(185, 199, 219))
    draw.text((712, 296), 'ACTIVE' if row['collision_active'] == '1' else 'ARMED', font=F22,
              fill=(93, 220, 152) if row['collision_active'] == '1' else (246, 195, 83))
    draw.text((712, 340), f"Contact  {float(row['contact_force_n']):5.1f} N", font=F18, fill=(195, 210, 228))
    draw.text((712, 368), f"Pressure {float(row['tool_pressure']):5.2f}", font=F18, fill=(195, 210, 228))
    draw.text((712, 404), f"Nails     {nail_count} / 6", font=F18, fill=(240, 167, 135))

    draw.rectangle((28, 508, 664, 516), fill=(72, 88, 111))
    progress = max(0.0, min(1.0, float(row['frame']) / max(1, int(row.get('_total_frames', row['frame'])))))
    draw.rectangle((28, 508, 28 + int(636 * progress), 516), fill=(75, 192, 220))
    event = 'FASTENER FIRED' if nail_count > previous_nails else 'Holding board / moving to target'
    draw.text((28, 476), event, font=F18, fill=(250, 196, 137) if nail_count > previous_nails else (163, 187, 212))
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()

    with args.trajectory.open(newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit('trajectory.csv is empty')
    total = len(rows) - 1
    for row in rows:
        row['_total_frames'] = str(total)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    container = av.open(str(args.output), mode='w')
    stream = container.add_stream('h264', rate=args.fps)
    stream.width = WIDTH
    stream.height = HEIGHT
    stream.pix_fmt = 'yuv420p'
    previous_nails = 0
    try:
        for row in rows:
            image = render(row, previous_nails)
            previous_nails = int(row['nails_installed'])
            frame = av.VideoFrame.from_image(image)
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    finally:
        container.close()
    print(f'[OK] evidence video: {args.output}')


if __name__ == '__main__':
    main()
