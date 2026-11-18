"""Convert an X11 xwd dump (xwd -root -silent > f.xwd) to PNG, optionally cropped. ZPixmap 24/32 bpp only.
usage: xwd_to_png.py IN.xwd OUT.png [x0 y0 x1 y1]"""
import struct
import sys

import numpy as np
from PIL import Image

raw = open(sys.argv[1], "rb").read()
h = struct.unpack(">25I", raw[:100])
header_size, fmt, depth, w, hgt, byte_order, bpp, bpl, ncolors = h[0], h[2], h[3], h[4], h[5], h[7], h[11], h[12], h[19]
red_mask, green_mask, blue_mask = h[13], h[14], h[15]
assert fmt == 2 and bpp in (24, 32), (fmt, bpp)
off = header_size + ncolors * 12
data = np.frombuffer(raw, np.uint8, count=bpl * hgt, offset=off).reshape(hgt, bpl)
px = data[:, : w * (bpp // 8)].reshape(hgt, w, bpp // 8)
order = "<" if byte_order == 0 else ">"
val = px[..., 0].astype(np.uint32) | (px[..., 1].astype(np.uint32) << 8) | (px[..., 2].astype(np.uint32) << 16)
if bpp == 32:
    val |= px[..., 3].astype(np.uint32) << 24
if byte_order == 1:
    val = val.byteswap() >> (32 - bpp) if bpp == 24 else val.byteswap()
def chan(mask):
    shift = (mask & -mask).bit_length() - 1
    return ((val & mask) >> shift).astype(np.uint8)
img = np.stack([chan(red_mask), chan(green_mask), chan(blue_mask)], -1)
if len(sys.argv) >= 7:
    x0, y0, x1, y1 = (int(v) for v in sys.argv[3:7])
    img = img[y0:y1, x0:x1]
Image.fromarray(img).save(sys.argv[2])
print("saved", sys.argv[2], img.shape)
