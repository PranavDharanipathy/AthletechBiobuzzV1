#!/usr/bin/env python3
"""Pull every figure out of the Competition Manual PDF (no PDF library needed) so they can be looked at locally.

  python -I tools/manual_figures.py MANUAL.pdf [OUT]      OUT defaults to tuning-data/manual-figures (git-ignored)

Writes obj<NNNNN>_<w>x<h>.png per image plus sheet<NN>.png contact sheets labeled with the object number. The HIVE / CELL
figures of TU09 were objects 634 (Fig 9-7), 640 (9-8), 644 (9-9), 646 (9-10), 651 (9-11), 655 (FLOWER), 676 (AprilTags);
other versions renumber them, so find them on the sheets. Get the PDF from https://ftc-resources.firstinspires.org/ftc/game/manual.
"""
import io
import os
import re
import sys
import zlib

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    pdf = open(sys.argv[1], 'rb').read()
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, 'tuning-data', 'manual-figures')
    os.makedirs(out, exist_ok=True)
    names = []
    for m in re.finditer(rb'(\d+)\s+0\s+obj(.*?)endobj', pdf, re.S):
        body = m.group(2)
        if b'/Subtype' not in body or b'/Image' not in body:
            continue
        s = re.search(rb'stream\r?\n', body)
        if not s:
            continue
        head, data = body[:s.start()], body[s.end():body.rfind(b'endstream')].rstrip(b'\r\n')
        w = int(re.search(rb'/Width\s+(\d+)', head).group(1))
        h = int(re.search(rb'/Height\s+(\d+)', head).group(1))
        if w * h < 150 * 150:
            continue
        try:
            if b'DCTDecode' in head:
                im = Image.open(io.BytesIO(data))
            elif b'FlateDecode' in head:
                raw = zlib.decompress(data)
                mode = {1: 'L', 3: 'RGB', 4: 'CMYK'}.get(len(raw) // (w * h))
                if mode is None:
                    continue
                im = Image.frombytes(mode, (w, h), raw[:w * h * len(mode)])
            else:
                continue
        except Exception:
            continue
        name = 'obj%05d_%dx%d.png' % (int(m.group(1)), w, h)
        im.convert('RGB').save(os.path.join(out, name))
        names.append(name)
    f = ImageFont.load_default(size=22)
    for start in range(0, len(names), 24):
        sheet = Image.new('RGB', (6 * 300, 4 * 240), 'white')
        d = ImageDraw.Draw(sheet)
        for i, n in enumerate(names[start:start + 24]):
            im = Image.open(os.path.join(out, n)).convert('RGB')
            im.thumbnail((296, 210))
            x, y = (i % 6) * 300, (i // 6) * 240
            sheet.paste(im, (x, y + 26))
            d.text((x + 2, y + 2), n[3:8], fill='red', font=f)
        sheet.save(os.path.join(out, 'sheet%02d.png' % (start // 24)))
    print('%d figures -> %s' % (len(names), out))


if __name__ == '__main__':
    main()
