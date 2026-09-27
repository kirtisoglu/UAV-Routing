"""Export the packed viewer datasets into the website's static folder.

The standalone pages inline their data, which is fine for one instance but not
for a picker over fourteen. Here each instance keeps its own data.json and an
index.json lists them, so the site fetches one dataset at a time.

    python3 animation/export_site.py [dest]

dest defaults to the website's static/data/uav-routing.
"""
import json, os, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "datasets")
DEST = sys.argv[1] if len(sys.argv) > 1 else \
    "/Users/kirtisoglu/GitHub/website/static/data/uav-routing"

ORDER = ["r101_50", "r101_100", "r1_2_1_200", "c101_50", "c101_100", "c1_2_1_200",
         "rc1_2_1_200", "r102_100", "r104_100", "c104_100", "rc104_100",
         "pr11_48", "pr15_240", "pr10_288"]

os.makedirs(DEST, exist_ok=True)
index, total = [], 0
for stem in ORDER:
    src = os.path.join(SRC, stem, "data.json")
    if not os.path.exists(src):
        print(f"  skip {stem} (not packed)")
        continue
    os.makedirs(os.path.join(DEST, stem), exist_ok=True)
    shutil.copy2(src, os.path.join(DEST, stem, "data.json"))
    d = json.load(open(src))
    size = os.path.getsize(src)
    total += size
    index.append({"id": stem, "name": d.get("instance", stem),
                  "frames": len(d.get("frames", [])),
                  "best": d.get("hi"), "mb": round(size / 1e6, 2)})
    print(f"  {d.get('instance', stem):<15} {len(d.get('frames', [])):>5} frames  {size/1e6:>5.2f} MB")

with open(os.path.join(DEST, "index.json"), "w") as f:
    json.dump(index, f, indent=1)
print(f"\n{len(index)} datasets -> {DEST}  ({total/1e6:.1f} MB total)")
