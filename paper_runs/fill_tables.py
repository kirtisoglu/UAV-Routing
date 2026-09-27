"""Write the matheuristic tables of the manuscript from the campaign CSVs.

    python3 paper_runs/fill_tables.py              # every table whose inputs exist
    python3 paper_runs/fill_tables.py --dry-run    # print the rows, touch nothing
    python3 paper_runs/fill_tables.py --only tab:theta tab:coverage
    python3 paper_runs/fill_tables.py --tex paper/ArXiv-version.tex

For each table the script finds \\label{tab:...} in the tex, the \\midrule that
closes the header after it and the \\bottomrule that ends the body, and replaces
what lies between them. Captions, headers and everything else are untouched.
A table whose input CSVs are missing is skipped with a message; an instance
missing from a CSV gets an empty row.

Table                       inputs under paper_runs/results
  tab:matheuristic-vs-exact   design_new.csv, misocp_s1.csv
  tab:fixed-speed             design_new_fixed.csv, design_new.csv
  tab:coverage                design_new_noloiter.csv, design_new.csv
  tab:theta                   design_new.csv (D = 3), design_new_D6.csv, design_new_D12.csv
  tab:initial-tour            design_new_R1.csv, design_new_R2.csv, design_new_R3s1.csv,
                              design_new_R3s2.csv, design_new_R3s3.csv, design_new.csv (R4)
"""
import os, re, sys, csv

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
RES = os.path.join(HERE, "results")
TEX = os.path.join(REPO, "paper", "ArXiv-version.tex")
ORDER = ["R101 (50)", "R101 (100)", "R1_2_1 (200)", "C101 (50)", "C101 (100)",
         "C1_2_1 (200)", "RC1_2_1 (200)", "R102 (100)", "R104 (100)",
         "C104 (100)", "RC104 (100)", "PR11 (48)", "PR15 (240)", "PR10 (288)"]
# a \midrule after these instances, in the tables that separate the instance groups
GROUP_ENDS = {"R1_2_1 (200)", "C1_2_1 (200)", "RC1_2_1 (200)", "RC104 (100)"}
EPS = 0.005


def load(name, key="Instance"):
    p = os.path.join(RES, name)
    if not os.path.exists(p):
        return None
    with open(p, newline="") as f:
        return {r[key]: r for r in csv.DictReader(f)}


def num(s):
    return f"{s:,.2f}".replace(",", "\\,")


def num1(s):
    return f"{s:,.1f}".replace(",", "\\,")


def bold(s):
    return "\\textbf{" + s + "}"


def name_tex(n):
    return f"{n.replace('_', chr(92) + '_'):15s}"


def fnum(row, col):
    v = row.get(col, "") if row else ""
    return float(v) if v not in ("", None) else None


def signed(x):
    s = f"{x:+.2f}"
    return f"$\\mathbf{{{s}}}$" if x < 0 else f"${s}$"


def rows_matheuristic_vs_exact():
    new, mis = load("design_new.csv"), load("misocp_s1.csv")
    if new is None or mis is None:
        return None
    out = []
    for n in ORDER:
        r, m = new.get(n), mis.get(n)
        if r is None or m is None:
            out.append(f"{name_tex(n)} & & & & & & & & \\\\"); continue
        f_ils, f_star = float(r["Objective"]), float(m["Objective"])
        gap, t_mis = float(m["Gap (%)"]), float(m["Time (s)"])
        closed = gap <= 0.5
        mcells = [num(f_star), num1(t_mis), f"{gap:.2f}"]
        if closed:
            mcells = [bold(c) for c in mcells]
        delta = 100.0 * (f_star - f_ils) / f_star
        init = fnum(r, "init_obj")
        out.append(f"{name_tex(n)} & {num(init) if init is not None else ''} & {num(f_ils)} & "
                   f"{float(r['t_best (s)']):.1f} & {float(r['Wall (s)']):.1f} & "
                   f"{mcells[0]} & {mcells[1]} & {mcells[2]} & {signed(delta)} \\\\")
    return out


def rows_fixed_speed():
    fixed, new = load("design_new_fixed.csv"), load("design_new.csv")
    if fixed is None or new is None:
        return None
    out = []
    for n in ORDER:
        a, b = fixed.get(n), new.get(n)
        if a is None or b is None:
            out.append(f"{name_tex(n)} & & & & & \\\\"); continue
        fa, fb = float(a["Objective"]), float(b["Objective"])
        out.append(f"{name_tex(n)} & {num(fa)} & {a['Tour']} & {num(fb)} & {b['Tour']} & "
                   f"{100.0 * (fb - fa) / fa:.2f} \\\\")
    return out


def rows_coverage():
    nol, new = load("design_new_noloiter.csv"), load("design_new.csv")
    if nol is None or new is None:
        return None
    out = []
    for n in ORDER:
        a, b = nol.get(n), new.get(n)
        if a is None or b is None:
            out.append(f"{name_tex(n)} &  &  &  &  &  &  &  \\\\")
        else:
            fa, fb = float(a["Objective"]), float(b["Objective"])
            ka, kb = fnum(a, "flown_km"), fnum(b, "flown_km")
            out.append(f"{name_tex(n)} & {num(fa)} & {a['Tour']} & {f'{ka:.1f}' if ka is not None else ''} & "
                       f"{num(fb)} & {b['Tour']} & {f'{kb:.1f}' if kb is not None else ''} & "
                       f"{100.0 * (fb - fa) / fa:.2f} \\\\")
        if n in GROUP_ENDS:
            out.append("\\midrule")
    return out


def rows_theta():
    grid = {3: load("design_new.csv"), 6: load("design_new_D6.csv"), 12: load("design_new_D12.csv")}
    if any(v is None for v in grid.values()):
        return None
    out = []
    for n in ORDER:
        rs = {D: grid[D].get(n) for D in (3, 6, 12)}
        objs = {D: float(r["Objective"]) for D, r in rs.items() if r is not None}
        best = max(objs.values()) if objs else None
        cells = []
        for D in (3, 6, 12):
            r = rs[D]
            if r is None:
                cells += ["", "", "", ""]; continue
            o = num(objs[D])
            cells += [bold(o) if best is not None and objs[D] >= best - EPS else o,
                      str(r["shakes"]), f"{float(r['t_best (s)']):.0f}", f"{float(r['Wall (s)']):.0f}"]
        out.append(f"{name_tex(n)} & " + " & ".join(cells) + " \\\\")
    return out


def rows_initial_tour():
    r1, r2, r4 = load("design_new_R1.csv"), load("design_new_R2.csv"), load("design_new.csv")
    r3 = [load(f"design_new_R3s{s}.csv") for s in (1, 2, 3)]
    if r1 is None or r2 is None or r4 is None or any(x is None for x in r3):
        return None
    out = []
    for n in ORDER:
        vals = {}
        for key, tab in (("R1", r1), ("R2", r2), ("R4", r4)):
            if tab.get(n) is not None:
                vals[key] = float(tab[n]["Objective"])
        draws = [float(t[n]["Objective"]) for t in r3 if t.get(n) is not None]
        if draws:
            vals["R3"] = sum(draws) / len(draws)
        if not vals:
            out.append(f"{name_tex(n)} &  &  &  &  \\\\")
        else:
            best = max(vals.values())
            cell = lambda k: (bold(num(vals[k])) if vals[k] >= best - EPS else num(vals[k])) if k in vals else ""
            r3cell = (f"{cell('R3')} / {num(max(draws))}" if draws else "")
            out.append(f"{name_tex(n)} & {cell('R1')} & {cell('R2')} & {r3cell} & {cell('R4')} \\\\")
        if n in GROUP_ENDS:
            out.append("\\midrule")
    return out


TABLES = {"tab:matheuristic-vs-exact": rows_matheuristic_vs_exact,
          "tab:fixed-speed": rows_fixed_speed,
          "tab:coverage": rows_coverage,
          "tab:theta": rows_theta,
          "tab:initial-tour": rows_initial_tour}


def replace_body(tex, label, rows):
    """Replace the lines between the header's \\midrule and \\bottomrule of the table `label`."""
    i = tex.find("\\label{" + label + "}")
    if i < 0:
        raise SystemExit(f"{label}: label not found in the tex")
    j = tex.find("\\midrule", i)
    k = tex.find("\\bottomrule", j)
    if j < 0 or k < 0 or k < j:
        raise SystemExit(f"{label}: table structure not recognised")
    j = tex.find("\n", j) + 1                    # first body line
    body = "\n".join(rows) + "\n"
    return tex[:j] + body + tex[k:]


def main():
    argv = sys.argv[1:]
    dry = "--dry-run" in argv
    tex_path = argv[argv.index("--tex") + 1] if "--tex" in argv else TEX
    only = set(argv[argv.index("--only") + 1:]) if "--only" in argv else set(TABLES)
    only = {o for o in only if o.startswith("tab:")}
    tex = open(tex_path).read()
    filled = []
    for label, fn in TABLES.items():
        if label not in only:
            continue
        rows = fn()
        if rows is None:
            print(f"[skip] {label}: inputs missing (see the module docstring)")
            continue
        if dry:
            print(f"\n% {label}"); print("\n".join(rows))
        else:
            tex = replace_body(tex, label, rows)
        filled.append(label)
    if not dry and filled:
        open(tex_path, "w").write(tex)
    print(("[dry-run] " if dry else "[written] ") + (", ".join(filled) if filled else "nothing"))


if __name__ == "__main__":
    main()
