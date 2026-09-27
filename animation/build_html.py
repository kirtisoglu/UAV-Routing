"""Build a standalone HTML replay from a viewer JSON.

The page is one file with the trace inlined, so it opens from the filesystem
with no server and no network beyond the web font.

    python3 animation/build_html.py animation/out/r104_300s.json animation/out/r104_300s.html
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, "viewer_template.html")
BACKUP = os.path.join(HERE, ".viewer_template.bak")


def read_template():
    """The template, restoring it from the backup or from a page already built if
    the file has gone missing."""
    import glob, re
    if os.path.exists(TEMPLATE):
        body = open(TEMPLATE).read()
        open(BACKUP, "w").write(body)          # keep the backup current
        return body
    if os.path.exists(BACKUP):
        body = open(BACKUP).read()
        open(TEMPLATE, "w").write(body)
        print(f"note: {TEMPLATE} was missing, restored from the backup")
        return body
    pages = sorted(glob.glob(os.path.join(HERE, "**", "*.html"), recursive=True),
                   key=os.path.getmtime)
    for p in reversed(pages):
        h = open(p).read()
        if 'id="trace-data"' not in h:
            continue
        body = h.split("<body>\n", 1)[-1].rsplit("\n</body>", 1)[0]
        body = re.sub(r'(<script id="trace-data" type="application/json">).*?(</script>)',
                      lambda m: m.group(1) + "__DATA__" + m.group(2), body, count=1, flags=re.S)
        body = re.sub(r"<title>.*?</title>", "<title>__TITLE__</title>", body, count=1, flags=re.S)
        if body.count("__DATA__") == 1 and body.count("__TITLE__") == 1:
            open(TEMPLATE, "w").write(body); open(BACKUP, "w").write(body)
            print(f"note: {TEMPLATE} was missing, rebuilt from {p}")
            return body
    raise SystemExit(f"{TEMPLATE} is missing and cannot be reconstructed")


def main():
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + ".html"
    data = json.load(open(src))
    blob = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    name = data.get("instance", "run")
    body = read_template()
    body = body.replace("__TITLE__", f"{name} search replay").replace("__DATA__", blob)
    # A file opened from disk carries no document wrapper, and without a doctype the
    # browser falls into quirks mode, where the canvases end up with no size.
    html = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1, '
            'viewport-fit=cover">\n</head>\n<body>\n' + body + '\n</body>\n</html>\n')
    open(out, "w").write(html)
    print(f"wrote {out}  ({os.path.getsize(out)/1e6:.2f} MB, {len(data['frames'])} frames)")


if __name__ == "__main__":
    main()
