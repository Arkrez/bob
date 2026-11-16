# Bob's vision

Recognition skills that belong to Bob himself, not to one body: anything with a camera (Rocky-Bob's
Pi Camera Module 3 today) hands frames to `BikeSpotter` in `bob_vision.py`.

So far he can:

1. **Spot motorcycles.** YOLO11n trained on COCO, used as is (COCO's class 3 is motorcycle).
2. **Say which brand each one is**, and how likely it is to be a Kawasaki. A YOLO11 classifier we train
   on photos of nine makes: Kawasaki, Honda, Yamaha, Suzuki, Ducati, BMW, Harley-Davidson, KTM, Triumph.

Next: finding the chain and sprockets, trained on photos of real bikes (yours).

## Setup

```bash
cd vision
python3.12 -m venv .venv
.venv/bin/pip install ultralytics requests
```

## Use it

```bash
.venv/bin/python look.py photo.jpg        # writes photo_seen.jpg with boxes and brands
.venv/bin/python look.py photos/          # a whole folder
.venv/bin/python look.py --camera 0       # live from a webcam, q to quit
```

```python
from bob_vision import BikeSpotter
for bike in BikeSpotter().look(frame):    # path, PIL image or numpy frame
    print(bike.box, bike.brand, bike.kawasaki)
```

## Train the brand classifier again

```bash
.venv/bin/python fetch_commons.py         # photos from Wikimedia Commons into data/raw/<brand>/
.venv/bin/python crop.py                  # cut out each photo's main bike into data/brands/
.venv/bin/python train_brand.py           # trains on the Mac's GPU, saves models/brand.pt
```

- `fetch_commons.py` searches each brand's Commons category tree, keeping only photos whose own
  categories or name mention the brand, so riders, races and shows full of other makes stay out. Every
  photo's author and
  licence goes in `data/raw/credits*.csv` (all are free licences, mostly CC BY / CC BY-SA).
- `crop.py` keeps a photo only when the detector finds one clear main motorcycle filling at least 12% of
  it. That throws out logos, engines and crowd shots, and trains the classifier on the same kind of crop
  it gets from the detector on Bob.
- The photos and crops stay out of git (`data/`); the trained `models/*.pt` are committed.

## How well it works

Brand classifier `models/brand.pt`: YOLO11m-cls at 320 px, trained on 1,951 crops (700 Kawasaki, 134-293
for each other make), tested on 246 held-out photos (`train_brand.py` prints this table):

| | Right |
| --- | --- |
| Kawasaki | 73% (65/89) |
| BMW | 88% |
| KTM | 73% |
| Ducati | 72% |
| Yamaha | 50% |
| Triumph | 48% |
| Harley-Davidson | 46% |
| Suzuki | 28% |
| Honda | 10% |
| All nine makes | 59% |

**Kawasaki or not** (Kawasaki probability over 50%): he spots 70% of the Kawasakis, and 76% of his
"Kawasaki" calls are right.

What it gets right and wrong:
- Green sport bikes (Ninja, ZX, ZRX) are the easiest; a ZRX 1200 he'd never seen came out 94% Kawasaki.
- Plain-coloured cruisers and standards (a Vulcan VN1500 and a Z400LTD in the spot checks) look much like
  Suzukis and Hondas of the same era, and he called both of them Suzukis. That family resemblance is
  probably also why Honda and Suzuki score worst.
- The smaller YOLO11s-cls at 224 px scored about the same overall (61%) but found fewer Kawasakis (62%);
  `--model yolo11s-cls.pt --imgsz 224` if the Pi needs something faster.

The limit is data: Commons has only 130-300 usable photos for most makes. The biggest gains next would
come from more photos per make, and from photos of the bikes Bob will actually meet.

Not measured yet: speed on the Pi 5. The detector (YOLO11n) is built for that size of computer; the
classifier only runs on each motorcycle found.
