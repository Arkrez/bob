"""Bob's eyes: spot motorcycles in a camera frame and say which brand each one is.

Not tied to any one body: anything with a camera can hand frames to BikeSpotter.

    spotter = BikeSpotter()
    for bike in spotter.look("photo.jpg"):          # a path, a PIL image or an RGB/BGR numpy array
        print(bike.box, bike.brand, bike.kawasaki)

The detector is YOLO11n trained on COCO (class 3 is motorcycle, no training of ours needed). The brand
classifier is models/brand.pt from train_brand.py, run on each motorcycle's crop.
"""

import pathlib
from dataclasses import dataclass

import numpy as np
from PIL import Image
from ultralytics import YOLO

HERE = pathlib.Path(__file__).resolve().parent
MODELS = HERE / "models"
DETECTOR = MODELS / "yolo11n.pt"
CLASSIFIER = MODELS / "brand.pt"
MOTORCYCLE = 3          # COCO class id
MIN_CONF = 0.35
PAD = 0.08              # crop margin, as a share of the box, so the classifier sees the whole bike


@dataclass
class Bike:
    box: tuple          # x1, y1, x2, y2 in pixels
    conf: float         # how sure the detector is that it's a motorcycle
    brand: str = ""     # best guess, empty without a classifier
    brand_conf: float = 0.0
    kawasaki: float = 0.0   # probability that it's a Kawasaki


def crop(image, box, pad=PAD):
    """The box from a PIL image, grown by pad on each side and clipped to the image."""
    x1, y1, x2, y2 = box
    dx, dy = (x2 - x1) * pad, (y2 - y1) * pad
    w, h = image.size
    return image.crop((max(0, x1 - dx), max(0, y1 - dy), min(w, x2 + dx), min(h, y2 + dy)))


def as_pil(image):
    if isinstance(image, (str, pathlib.Path)):
        return Image.open(image).convert("RGB")
    if isinstance(image, np.ndarray):
        return Image.fromarray(image[..., ::-1] if image.shape[-1] == 3 else image)   # OpenCV frames are BGR
    return image.convert("RGB")


class BikeSpotter:
    def __init__(self, classifier=CLASSIFIER, device=None):
        MODELS.mkdir(exist_ok=True)
        self.detector = YOLO(DETECTOR)          # downloads yolo11n.pt into models/ the first time
        self.classifier = YOLO(classifier) if classifier and pathlib.Path(classifier).is_file() else None
        self.device = device

    def motorcycles(self, image, min_conf=MIN_CONF):
        """Motorcycle boxes, most confident first."""
        r = self.detector.predict(image, classes=[MOTORCYCLE], conf=min_conf, device=self.device, verbose=False)[0]
        found = [Bike(tuple(b.xyxy[0].tolist()), float(b.conf)) for b in r.boxes]
        return sorted(found, key=lambda b: -b.conf)

    def brand(self, bike_crop):
        """(best brand, its probability, Kawasaki probability) for one cropped motorcycle."""
        r = self.classifier.predict(bike_crop, device=self.device, verbose=False)[0]
        names, p = r.names, r.probs.data.cpu().numpy()
        best = int(p.argmax())
        kawa = next((float(p[i]) for i, n in names.items() if n == "kawasaki"), 0.0)
        return names[best], float(p[best]), kawa

    def look(self, image, min_conf=MIN_CONF):
        image = as_pil(image)
        bikes = self.motorcycles(image, min_conf)
        if self.classifier:
            for b in bikes:
                b.brand, b.brand_conf, b.kawasaki = self.brand(crop(image, b.box))
        return bikes
