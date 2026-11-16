"""Run Bob's bike spotter on photos, a folder of them, or a webcam, and draw what he sees.

    .venv/bin/python look.py photo.jpg                writes photo_seen.jpg next to it
    .venv/bin/python look.py photos/                  every .jpg/.png in the folder
    .venv/bin/python look.py --camera 0               live, q to quit
"""

import argparse
import pathlib

from PIL import Image, ImageDraw, ImageFont

from bob_vision import BikeSpotter

GREEN, GREY = (105, 190, 40), (230, 230, 230)


def label(bike):
    if not bike.brand:
        return f"motorcycle {bike.conf:.0%}"
    return f"{bike.brand} {bike.brand_conf:.0%}  (Kawasaki {bike.kawasaki:.0%})"


def draw(image, bikes):
    d = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=max(14, image.size[0] // 50))
    for b in bikes:
        color = GREEN if b.kawasaki >= 0.5 else GREY
        d.rectangle(b.box, outline=color, width=max(2, image.size[0] // 300))
        x, y = b.box[0], max(0, b.box[1] - font.size - 6)
        d.rectangle(d.textbbox((x, y), label(b), font=font), fill=color)
        d.text((x, y), label(b), fill=(0, 0, 0), font=font)
    return image


def photos(spotter, target):
    paths = sorted(p for p in target.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")) \
        if target.is_dir() else [target]
    for p in paths:
        if p.stem.endswith("_seen"):
            continue
        image = Image.open(p).convert("RGB")
        bikes = spotter.look(image)
        out = p.with_name(p.stem + "_seen.jpg")
        draw(image, bikes).save(out)
        print(f"{p.name}: " + ("; ".join(label(b) for b in bikes) or "no motorcycles") + f"  -> {out.name}")


def camera(spotter, index):
    import cv2
    import numpy as np
    cap = cv2.VideoCapture(index)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        image = Image.fromarray(frame[..., ::-1])
        shown = np.array(draw(image, spotter.look(image)))[..., ::-1]
        cv2.imshow("Bob sees", shown)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", nargs="?", type=pathlib.Path)
    ap.add_argument("--camera", type=int)
    args = ap.parse_args()
    spotter = BikeSpotter()
    if args.camera is not None:
        camera(spotter, args.camera)
    else:
        photos(spotter, args.target)


if __name__ == "__main__":
    main()
