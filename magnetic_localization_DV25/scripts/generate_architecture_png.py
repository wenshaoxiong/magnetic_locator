from pathlib import Path

import struct
import zlib

def main():
    out = Path("docs/images")
    out.mkdir(parents=True, exist_ok=True)

    w, h = 1200, 450
    img = bytearray([255, 255, 255] * w * h)

    def set_px(x, y, rgb):
        if 0 <= x < w and 0 <= y < h:
            i = (y * w + x) * 3
            img[i : i + 3] = bytes(rgb)

    def draw_line(x0, y0, x1, y1, rgb):
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            set_px(x, y, rgb)
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def draw_rect(x, y, ww, hh, rgb, t=3):
        for k in range(t):
            draw_line(x + k, y + k, x + ww - 1 - k, y + k, rgb)
            draw_line(x + k, y + hh - 1 - k, x + ww - 1 - k, y + hh - 1 - k, rgb)
            draw_line(x + k, y + k, x + k, y + hh - 1 - k, rgb)
            draw_line(x + ww - 1 - k, y + k, x + ww - 1 - k, y + hh - 1 - k, rgb)

    black = (0, 0, 0)
    gray = (80, 80, 80)

    b1 = (60, 80, 300, 140)
    b2 = (450, 80, 350, 140)
    b3 = (880, 80, 300, 140)
    draw_rect(*b1, black)
    draw_rect(*b2, black)
    draw_rect(*b3, black)

    draw_line(b1[0] + b1[2], b1[1] + b1[3] // 2, b2[0], b2[1] + b2[3] // 2, gray)
    draw_line(b2[0] + b2[2], b2[1] + b2[3] // 2, b3[0], b3[1] + b3[3] // 2, gray)

    draw_rect(420, 280, 360, 90, black, t=2)

    def png_chunk(tag, data):
        chunk = tag + data
        return struct.pack(">I", len(data)) + chunk + struct.pack(">I", zlib.crc32(chunk) & 0xFFFFFFFF)

    raw = bytearray()
    for y in range(h):
        raw.append(0)
        row = img[y * w * 3 : (y + 1) * w * 3]
        raw.extend(row)

    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    idat = zlib.compress(bytes(raw), level=6)

    png = bytearray()
    png.extend(signature)
    png.extend(png_chunk(b"IHDR", ihdr))
    png.extend(png_chunk(b"IDAT", idat))
    png.extend(png_chunk(b"IEND", b""))

    (out / "architecture.png").write_bytes(png)


if __name__ == "__main__":
    main()

