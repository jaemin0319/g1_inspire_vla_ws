"""Zoomed crop of an image with a pixel grid in ORIGINAL coordinates (for manual landmark annotation).
usage: crop_grid.py IMG OUT x0 y0 x1 y1 [zoom]"""
import sys
import cv2
im = cv2.imread(sys.argv[1]); out = sys.argv[2]
x0, y0, x1, y1 = (int(v) for v in sys.argv[3:7]); z = float(sys.argv[7]) if len(sys.argv) > 7 else 3.0
c = cv2.resize(im[y0:y1, x0:x1], None, fx=z, fy=z, interpolation=cv2.INTER_CUBIC)
for g in range((x0 // 10 + 1) * 10, x1, 10):
    x = int((g - x0) * z); cv2.line(c, (x, 0), (x, c.shape[0]), (0, 255, 255) if g % 50 == 0 else (110, 110, 110), 1)
    if g % 50 == 0: cv2.putText(c, str(g), (x + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
for g in range((y0 // 10 + 1) * 10, y1, 10):
    y = int((g - y0) * z); cv2.line(c, (0, y), (c.shape[1], y), (0, 255, 255) if g % 50 == 0 else (110, 110, 110), 1)
    if g % 50 == 0: cv2.putText(c, str(g), (2, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
cv2.imwrite(out, c)
