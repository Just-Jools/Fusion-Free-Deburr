"""Generates the Ball Deburr command icons. Run: python tools/make_icons.py"""

import os

from PIL import Image, ImageDraw

OUT = os.path.join(os.path.dirname(__file__), '..', 'AdditionalToolpaths', 'resources', 'BallDeburr')
SIZES = (16, 32, 64)
CANVAS = 256  # draw large, downsample for smooth edges

THEMES = {
    '': {'block': (130, 130, 130, 255), 'tool': (36, 120, 210, 255)},
    '-dark': {'block': (185, 185, 185, 255), 'tool': (90, 170, 255, 255)},
}


def draw(colors):
    img = Image.new('RGBA', (CANVAS, CANVAS), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    # Part: block whose top-right edge is being broken by the ball.
    d.rectangle((16, 140, 172, 244), fill=colors['block'])
    # Ball endmill: ball centered just outside the corner, shank straight up.
    cx, cy, r = 184, 120, 50
    d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=colors['tool'])
    d.rectangle((cx - r, 0, cx + r, cy), fill=colors['tool'])
    return img


def main():
    os.makedirs(OUT, exist_ok=True)
    for suffix, colors in THEMES.items():
        big = draw(colors)
        for size in SIZES:
            big.resize((size, size), Image.LANCZOS).save(os.path.join(OUT, f'{size}x{size}{suffix}.png'))
    print(f'Wrote icons to {os.path.abspath(OUT)}')


if __name__ == '__main__':
    main()
