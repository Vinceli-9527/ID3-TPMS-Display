#!/usr/bin/env python3
"""
img2rgb565.py —— 把一张 PNG/JPG 转成屏幕上可直接 pushImage 的 C 头文件

用途：小屏（如 240x240 圆屏）放不下图片文件，也没有文件系统，
      所以把图片先缩到目标尺寸，再把每个像素转成 16 位色（RGB565），
      生成一个 uint16_t 数组直接编译进固件。

用法：
    pip install pillow
    python img2rgb565.py assets/car_big.png car_img --width 40
    # 生成 car_img.h，里面是 car_img[40*H] 的 RGB565 数组

参数：
    --width   目标宽度（高度按比例自动算）
    --bg      透明背景合成色，默认黑 (0,0,0)
    --big     输出大端序（多数屏驱动/SPI 直传要这个，默认开启）

为什么是 RGB565：
    16 位 = R 5bit + G 6bit + B 5bit。体积是 24 位 RGB 的 2/3，
    240x240 一张图约 115KB，40x74 只要 5.9KB，塞进固件毫无压力。
"""

import argparse
import os
import sys

try:
    from PIL import Image
except ImportError:
    sys.exit("缺少 Pillow，请先运行：pip install pillow")


def to_rgb565(r: int, g: int, b: int, big: bool = True) -> int:
    v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    if big:
        v = ((v & 0xFF) << 8) | (v >> 8)      # 字节序翻转，供 SPI DMA 直传
    return v


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="源图片路径")
    ap.add_argument("name", help="输出数组名 / 文件名（不含扩展名）")
    ap.add_argument("--width", type=int, default=0, help="目标宽度，0 表示保持原宽")
    ap.add_argument("--bg", default="0,0,0", help="透明背景合成色，如 0,0,0")
    ap.add_argument("--out", default=".", help="输出目录")
    ap.add_argument("--no-big", action="store_true", help="关闭大端序")
    args = ap.parse_args()

    bg = tuple(int(x) for x in args.bg.split(","))
    im = Image.open(args.input).convert("RGBA")

    if args.width and args.width != im.width:
        h = round(im.height * args.width / im.width)
        im = im.resize((args.width, h), Image.LANCZOS)

    # 透明像素合成到背景色（避免出现黑边）
    canvas = Image.new("RGB", im.size, bg)
    canvas.paste(im, mask=im.split()[3])
    w, h = canvas.size
    px = canvas.load()

    big = not args.no_big
    data = [to_rgb565(px[x, y][0], px[x, y][1], px[x, y][2], big)
            for y in range(h) for x in range(w)]

    out_path = os.path.join(args.out, args.name + ".h")
    upper = args.name.upper()
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("// 由 img2rgb565.py 自动生成，请勿手改\n")
        f.write("// 源文件：%s\n" % os.path.basename(args.input))
        f.write("#pragma once\n#include <stdint.h>\n\n")
        f.write("#define %s_W %d\n#define %s_H %d\n\n" % (upper, w, upper, h))
        f.write("static const uint16_t %s[%s_W * %s_H] = {\n" % (args.name, upper, upper))
        for i in range(0, len(data), 12):
            f.write("  " + ",".join("0x%04X" % v for v in data[i:i + 12]) + ",\n")
        f.write("};\n")

    print("已生成 %s   尺寸 %dx%d   像素 %d   约 %.1f KB 源码"
          % (out_path, w, h, len(data), os.path.getsize(out_path) / 1024))


if __name__ == "__main__":
    main()
