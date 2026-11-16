"""Turn the downloaded photos into the brand classifier's dataset: cut out the motorcycle in each photo.

Keeps a photo only when the detector finds one clear main motorcycle in it, which also drops the photos
that aren't of a whole bike (engines, logos, crowds). The crops go to data/brands/{train,val,test}/<brand>/,
split by a hash of the file name so a photo always lands in the same split.

    .venv/bin/python crop.py
"""

import hashlib
import shutil

from PIL import Image

from bob_vision import HERE, BikeSpotter, crop

RAW = HERE / "data" / "raw"
OUT = HERE / "data" / "brands"
MIN_CONF = 0.5
MIN_AREA = 0.12         # share of the photo the bike must fill
DOMINANT = 2.0          # the main bike must be this much bigger than the next one
SIZE = 320
PER_BRAND = 700        # keeps Kawasaki (by far the most photos) from swamping the other makes


def split(name):
    h = int(hashlib.md5(name.encode()).hexdigest(), 16) % 10
    return "test" if h == 0 else "val" if h == 1 else "train"


def main_bike(spotter, image):
    bikes = spotter.motorcycles(image, MIN_CONF)
    if not bikes:
        return None, "no bike"
    area = lambda b: (b.box[2] - b.box[0]) * (b.box[3] - b.box[1])
    bikes.sort(key=area, reverse=True)
    if area(bikes[0]) < MIN_AREA * image.size[0] * image.size[1]:
        return None, "too small"
    if len(bikes) > 1 and area(bikes[0]) < DOMINANT * area(bikes[1]):
        return None, "several bikes"
    return bikes[0], "kept"


def main():
    shutil.rmtree(OUT, ignore_errors=True)
    spotter = BikeSpotter(classifier="")
    for brand_dir in sorted(p for p in RAW.iterdir() if p.is_dir()):
        tally = {}
        # shuffled by name hash, so the capped brands get a fair mix rather than just the early years
        for path in sorted(brand_dir.glob("*.jpg"), key=lambda p: hashlib.md5(p.name.encode()).hexdigest()):
            if tally.get("kept", 0) >= PER_BRAND:
                break
            try:
                image = Image.open(path).convert("RGB")
            except OSError:
                tally["unreadable"] = tally.get("unreadable", 0) + 1
                continue
            bike, why = main_bike(spotter, image)
            tally[why] = tally.get(why, 0) + 1
            if bike:
                out = OUT / split(path.name) / brand_dir.name / path.name
                out.parent.mkdir(parents=True, exist_ok=True)
                c = crop(image, bike.box)
                c.thumbnail((SIZE, SIZE))
                c.save(out, quality=92)
        print(brand_dir.name, tally)


if __name__ == "__main__":
    main()
