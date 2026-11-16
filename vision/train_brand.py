"""Train the brand classifier on data/brands (from crop.py) and save it as models/brand.pt.

Starts from YOLO11 classification weights pretrained on ImageNet, so a few thousand crops are enough.
Prints accuracy on the held-out test split, overall and for Kawasaki in particular.

    .venv/bin/python train_brand.py                   about 10 min on an M2 Max
    .venv/bin/python train_brand.py --epochs 5        quick check
"""

import argparse
import shutil

import numpy as np
from ultralytics import YOLO

from bob_vision import CLASSIFIER, HERE, MODELS

DATA = HERE / "data" / "brands"


def evaluate(model_path, split="test"):
    """Per-brand accuracy on one split, and how well Kawasaki-or-not is told apart."""
    model = YOLO(model_path)
    rows, kawa = {}, []
    for brand_dir in sorted((DATA / split).iterdir()):
        files = sorted(brand_dir.glob("*.jpg"))
        hits = 0
        for i in range(0, len(files), 64):
            for r in model.predict([str(f) for f in files[i:i + 64]], verbose=False):
                p = r.probs.data.cpu().numpy()
                hits += r.names[int(p.argmax())] == brand_dir.name
                k = next(i for i, n in r.names.items() if n == "kawasaki")
                kawa.append((brand_dir.name == "kawasaki", p[k] >= 0.5))
        rows[brand_dir.name] = (hits, len(files))
    total = sum(h for h, _ in rows.values()), sum(n for _, n in rows.values())
    print(f"\n{split} accuracy {total[0] / total[1]:.0%} over {total[1]} bikes")
    for name, (h, n) in rows.items():
        print(f"  {name:10s} {h / n:5.0%}  ({h}/{n})")
    truth, said = np.array(kawa).T
    tp = (truth & said).sum()
    print(f"Kawasaki or not: recall {tp / max(truth.sum(), 1):.0%} (Kawasakis spotted), "
          f"precision {tp / max(said.sum(), 1):.0%} (\"Kawasaki\" calls that were right)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--model", default="yolo11m-cls.pt")
    ap.add_argument("--imgsz", type=int, default=320)
    args = ap.parse_args()
    MODELS.mkdir(exist_ok=True)
    model = YOLO(MODELS / args.model)
    model.train(data=str(DATA), epochs=args.epochs, imgsz=args.imgsz, batch=64, device="mps",
                project=str(HERE / "runs"), name="brand", exist_ok=True, patience=10,
                fliplr=0.5, hsv_h=0.0, erasing=0.2, workers=4, plots=False)
    # hsv_h=0: no hue shifts, Kawasaki green is one of the best clues
    best = HERE / "runs" / "brand" / "weights" / "best.pt"
    shutil.copy(best, CLASSIFIER)
    print(f"saved {CLASSIFIER.relative_to(HERE)}")
    evaluate(CLASSIFIER)


if __name__ == "__main__":
    main()
