"""Download motorcycle photos by brand from Wikimedia Commons (free licences) for the brand classifier.

Searches each brand's category tree (Commons' deepcat search), saving 640 px thumbnails to
data/raw/<brand>/ and each photo's author and licence to data/raw/credits.csv.

    .venv/bin/python fetch_commons.py                 every brand, up to 1500 photos each
    .venv/bin/python fetch_commons.py --brands kawasaki --limit 200
    .venv/bin/python fetch_commons.py --brands honda,yamaha --credits credits_2.csv     run a few at once
"""

import argparse
import csv
import pathlib
import re
import time
from concurrent.futures import ThreadPoolExecutor

import requests

HERE = pathlib.Path(__file__).resolve().parent
RAW = HERE / "data" / "raw"
API = "https://commons.wikimedia.org/w/api.php"
HEADERS = {"User-Agent": "BobVision/0.1 (https://github.com/Arkrez/bob; training data for a robot)"}
# brand: (Commons category, word the photo's own categories or name must contain)
BRANDS = {
    "kawasaki": ("Kawasaki motorcycles", "kawasaki"),
    "honda": ("Honda motorcycles", "honda"),
    "yamaha": ("Yamaha motorcycles", "yamaha"),
    "suzuki": ("Suzuki motorcycles", "suzuki"),
    "ducati": ("Ducati motorcycles", "ducati"),
    "bmw": ("BMW motorcycles", "bmw"),
    "harley": ("Harley-Davidson motorcycles", "harley"),
    "ktm": ("KTM motorcycles", "ktm"),
    "triumph": ("Triumph motorcycles", "triumph"),
}
# the deep category tree also holds riders, shows and paperwork; drop photos filed under these
SKIP = re.compile(r"logo|engine|advert|brochure|poster|manual|drawing|dealer|factory|toy|scale model|stamp|"
                  r"\bparts?\b|people|riders|portrait|interior|dashboard|instrument|badge|emblem", re.I)
WIDTH = 640
DOWNLOADS = 4        # photos fetched at once per brand


def get(params):
    for attempt in range(6):
        try:
            r = requests.get(API, params={**params, "format": "json"}, headers=HEADERS, timeout=60)
        except requests.RequestException:
            time.sleep(2 ** attempt)
            continue
        if r.status_code == 429 or r.status_code >= 500:
            time.sleep(2 ** attempt)
            continue
        r.raise_for_status()
        data = r.json()
        if "error" not in data:
            return data
        time.sleep(2 ** attempt)     # Commons sometimes answers a search with an error; try again
    raise RuntimeError(f"Commons kept refusing {params}")


def photos(category, word, limit):
    """(title, thumbnail URL, author, licence) of photos in a category tree that are filed under the brand."""
    params = {"action": "query", "generator": "search", "gsrsearch": f'deepcat:"{category}" filetype:bitmap',
              "gsrnamespace": 6, "gsrlimit": 50, "prop": "imageinfo|categories", "iiprop": "url|extmetadata",
              "iiurlwidth": WIDTH, "iiextmetadatafilter": "Artist|LicenseShortName",
              "cllimit": "max", "clshow": "!hidden"}
    found = 0
    while found < limit:
        data = get(params)
        for page in data.get("query", {}).get("pages", {}).values():
            text = page["title"] + " " + " ".join(c["title"] for c in page.get("categories", []))
            if "imageinfo" not in page or SKIP.search(text) or word not in text.lower():
                continue
            info = page["imageinfo"][0]
            meta = info.get("extmetadata", {})
            artist = re.sub(r"<[^>]+>", "", meta.get("Artist", {}).get("value", "")).strip()
            yield page["title"], info.get("thumburl") or info["url"], artist, meta.get("LicenseShortName", {}).get("value", "")
            found += 1
        if "continue" not in data:
            return
        params.update(data["continue"])


def download(url, path):
    if path.exists():
        return True
    try:
        r = requests.get(url, headers=HEADERS, timeout=60)
    except requests.RequestException:
        return False
    if r.status_code != 200:
        return False
    path.write_bytes(r.content)
    return True


def fetch(brand, limit, credits, credited):
    out = RAW / brand
    out.mkdir(parents=True, exist_ok=True)
    got = 0
    with ThreadPoolExecutor(DOWNLOADS) as pool:
        jobs = []
        for title, url, artist, licence in photos(*BRANDS[brand], limit):
            name = re.sub(r"[^\w.-]", "_", title.removeprefix("File:"))[:120]
            path = out / (pathlib.Path(name).stem + ".jpg")
            jobs.append((pool.submit(download, url, path), [brand, path.name, title, artist, licence]))
        for job, row in jobs:
            if not job.result():
                continue
            if tuple(row[:2]) not in credited:
                credits.writerow(row)
                credited.add(tuple(row[:2]))
            got += 1
    print(f"{brand}: {got} saved to {out.relative_to(HERE)}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--brands", default=",".join(BRANDS))
    ap.add_argument("--limit", type=int, default=1500, help="photos per brand")
    ap.add_argument("--credits", default="credits.csv", help="its own file per process when running several")
    args = ap.parse_args()
    RAW.mkdir(parents=True, exist_ok=True)
    listed = RAW / args.credits
    credited = {tuple(row[:2]) for row in csv.reader(open(listed))} if listed.exists() else set()
    with open(listed, "a", newline="") as f:
        credits = csv.writer(f)
        for brand in args.brands.split(","):
            fetch(brand, args.limit, credits, credited)
            f.flush()


if __name__ == "__main__":
    main()
