"""Write the ILS results into both papers (tables and figures), from the CSVs
and traces under paper_runs/results. Every step is idempotent: it rebuilds
the table body or figure from the files and replaces what is in the .tex.

  python3 paper_runs/fill_paper.py table9        Table 9 from ils_variable.csv + misocp_s1.csv
  python3 paper_runs/fill_paper.py table10       Table 10 from ils_fixed.csv + ils_variable.csv
  python3 paper_runs/fill_paper.py captable      joint theta x divisor table from theta.csv + cap.csv
  python3 paper_runs/fill_paper.py convergence   convergence figure from details/ils_traces
  python3 paper_runs/fill_paper.py perturb       perturbation figure from details/dynamics
  python3 paper_runs/fill_paper.py captions T D  write theta = T and c = ceil(k/D) into the Table 9 caption
  python3 paper_runs/fill_paper.py select-pair   print "T D", the pair with the smallest mean deviation
  python3 paper_runs/fill_paper.py compile       pdflatex both papers twice

Several steps may be given at once; "compile" is run last when present.
"""
import os, sys, csv, re, io, glob, shutil, subprocess, statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
RES = os.path.join(HERE, "results")
PAPERS = {
    "arxiv": "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/ArXiv-version.tex",
    "cie": "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/Submission-Comp. & Ind. Eng/main-cie.tex",
}
FIG_DIRS = {
    "arxiv": "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/fig",
    "cie": "/Users/kirtisoglu/GitHub/Brain/40-Papers/uav-routing/Submission-Comp. & Ind. Eng/fig",
}
# FILL_TARGETS="key=path;key=path" restricts the papers written to (e.g. the new-design copy only)
if os.environ.get("FILL_TARGETS"):
    PAPERS = dict(kv.split("=", 1) for kv in os.environ["FILL_TARGETS"].split(";"))
    FIG_DIRS = {k: os.path.join(os.path.dirname(p), "fig") for k, p in PAPERS.items()}
BLOCKS = [["R101 (50)", "R101 (100)", "R1_2_1 (200)"],
          ["C101 (50)", "C101 (100)", "C1_2_1 (200)"],
          ["RC1_2_1 (200)"],
          ["R102 (100)", "R104 (100)", "C104 (100)", "RC104 (100)"],
          ["PR11 (48)", "PR15 (240)", "PR10 (288)"]]
ORDER = [i for b in BLOCKS for i in b]
THETAS = (100, 300, 1000, 3000)
DIVS = (3, 6)


# ---------------------------------------------------------------- helpers
def num(x, d=2):
    return f"{x:,.{d}f}".replace(",", "\\,")


def tex(name):
    return name.replace("_", "\\_")


def rd(path):
    return list(csv.DictReader(open(path)))


def balanced(s, i):
    d = 0
    for j in range(i, len(s)):
        d += (s[j] == "{") - (s[j] == "}")
        if d == 0:
            return j + 1
    raise ValueError("unbalanced")


def replace_rows(C, label, rows):
    """Replace the data rows (after the first \\midrule) of the tabular that
    carries `label` by `rows` (a list of lines including block \\midrule)."""
    i = C.index("\\label{" + label + "}")
    ts = C.index("\\begin{tabular}", i)
    k = C.index("\\midrule\n", ts) + len("\\midrule\n")
    bt = C.index("\\bottomrule", k)
    return C[:k] + "\n".join(rows) + "\n" + C[bt:]


def replace_caption(C, label, new):
    i = C.index("\\label{" + label + "}")
    k = C.rfind("\\caption", 0, i)
    b = C.index("{", k)
    if C.startswith("\\captionof", k):        # \captionof{table|figure}{...}: skip the type group
        b = C.index("{", balanced(C, b))
    e = balanced(C, b)
    return C[:b + 1] + new + C[e - 1:]


def load_paper(key):
    return io.open(PAPERS[key]).read()


def save_paper(key, C):
    io.open(PAPERS[key], "w").write(C)


def blocked(row_of):
    out = []
    for blk in BLOCKS:
        for name in blk:
            r = row_of(name)
            if r is not None:
                out.append(r)
        out.append("\\midrule")
    return out[:-1]


# ---------------------------------------------------------------- table 9
def table9():
    ils = {r["Instance"]: r for r in rd(os.path.join(RES, "ils_variable.csv"))}
    mis = {r["Instance"]: r for r in rd(os.path.join(RES, "misocp_s1.csv"))}

    def row(name):
        if name not in ils:
            return f"{tex(name):15} & -- & -- & -- & -- & -- & -- & -- \\\\"
        a, m = ils[name], mis[name]
        f_ils, f_star = float(a["Objective"]), float(m["Objective"])
        gap = float(m["Gap (%)"]); t = float(m["Time (s)"])
        closed = gap <= 0.5
        b = (lambda s: f"\\textbf{{{s}}}") if closed else (lambda s: s)
        tstr = num(t, 1) if t < 3600 else "3\\,600"
        delta = 100 * (f_star - f_ils) / f_star
        dstr = f"$\\mathbf{{{delta:+.2f}}}$" if delta < 0 else f"${delta:+.2f}$"
        return (f"{tex(name):15} & {num(float(a['init_obj']))} & {num(f_ils)} & {int(a['t_best (s)']):,} & "
                f"{b(num(f_star))} & {b(tstr)} & {b(f'{gap:.2f}')} & {dstr} \\\\").replace(",", "\\,").replace("\\\\,", "\\,")

    rows = blocked(row)
    for key in PAPERS:
        C = load_paper(key)
        if key == "arxiv" and "\\begin{tabular}{l r c rr r r c r}" in C:   # drop the old K column header, once
            old_hdr = C[C.index("\\begin{tabular}{l r c rr r r c r}"):C.index("\\midrule", C.index("\\begin{tabular}{l r c rr r r c r}"))]
            new_hdr = ("\\begin{tabular}{l r rr r r c r}\n\\toprule\n"
                       "& \\multicolumn{1}{c}{Greedy (R4)} & \\multicolumn{2}{c}{Single-start ILS ($3\\,600$\\,s)} & \\multicolumn{3}{c}{Exact MISOCP} & \\\\\n"
                       "\\cmidrule(lr){2-2}\\cmidrule(lr){3-4}\\cmidrule(lr){5-7}\n"
                       "Instance & $f(R_4)$ & $f_{\\mathrm{best}}^{\\mathrm{ILS}}$ & $t_{\\mathrm{best}}$ (s) & $f^{\\star}$ & time (s) & gap (\\%) & $\\Delta$ (\\%) \\\\\n")
            C = C.replace(old_hdr, new_hdr)
        C = replace_rows(C, "tab:matheuristic-vs-exact", rows)
        save_paper(key, C)
    print(f"table9: {len(ils)} instances written")


# ---------------------------------------------------------------- table 10
def table10():
    var = {r["Instance"]: r for r in rd(os.path.join(RES, "ils_variable.csv"))}
    fx_path = os.path.join(RES, "ils_fixed.csv")
    fx = {r["Instance"]: r for r in rd(fx_path)} if os.path.exists(fx_path) else {}

    def row(name):
        if name not in fx or name not in var:
            return f"{tex(name):15} & -- & -- & -- & -- & -- \\\\"
        f, v = fx[name], var[name]
        of, ov = float(f["Objective"]), float(v["Objective"])
        gain = 100 * (ov - of) / of
        return f"{tex(name):15} & {num(of)} & {int(f['Tour'])} & {num(ov)} & {int(v['Tour'])} & {gain:.1f} \\\\"

    rows = blocked(row)
    for key in PAPERS:
        save_paper(key, replace_rows(load_paper(key), "tab:fixed-speed", rows))
    print(f"table10: {len(fx)} fixed-speed instances written")


# ---------------------------------------------------------------- joint grid
def cells_seed0():
    """(instance, theta, D) -> objective at seed 0; D = 6 from theta.csv."""
    out = {}
    for r in rd(os.path.join(RES, "theta.csv")):
        if int(r["seed"]) == 0:
            out[(r["Instance"], int(r["theta"]), 6)] = float(r["Objective"])
    p = os.path.join(RES, "cap.csv")
    if os.path.exists(p):
        for r in rd(p):
            if int(r["seed"]) == 0:
                out[(r["Instance"], int(r["theta"]), int(r["cap_div"]))] = float(r["Objective"])
    return out


def deviations(cells):
    pairs = [(t, d) for t in THETAS for d in DIVS]
    dev = {p: [] for p in pairs}
    for name in ORDER:
        vals = {p: cells.get((name,) + p) for p in pairs}
        if any(v is None for v in vals.values()):
            continue
        best = max(vals.values())
        for p in pairs:
            dev[p].append(100 * (best - vals[p]) / best)
    return dev


def select_pair():
    dev = deviations(cells_seed0())
    score = {p: st.mean(v) for p, v in dev.items() if v}
    if not score:
        return 1000, 6
    best = min(score.values())
    winners = [p for p, s in score.items() if abs(s - best) < 1e-9]
    pair = (1000, 6) if (1000, 6) in winners else min(winners)
    print("select-pair: " + ", ".join(f"theta={t},D={d}: {score[(t, d)]:.2f}%" for (t, d) in sorted(score))
          + f" -> {pair}", file=sys.stderr)
    return pair


def captable():
    cells = cells_seed0()
    pairs = [(t, d) for t in THETAS for d in DIVS]
    dev = deviations(cells)

    def row(name):
        vals = {p: cells.get((name,) + p) for p in pairs}
        present = [v for v in vals.values() if v is not None]
        best = max(present) if present else None
        cs = []
        for p in pairs:
            v = vals[p]
            if v is None:
                cs.append("--")
            else:
                s = num(v); cs.append(f"\\textbf{{{s}}}" if abs(v - best) < 1e-9 else s)
        return f"{tex(name):15} & " + " & ".join(cs) + " \\\\"

    rows = blocked(row)
    rows.append("\\midrule")
    rows.append("Mean deviation (\\%) & " + " & ".join(f"{st.mean(dev[p]):.2f}" if dev[p] else "--" for p in pairs) + " \\\\")
    rows.append("Max deviation (\\%) & " + " & ".join(f"{max(dev[p]):.2f}" if dev[p] else "--" for p in pairs) + " \\\\")
    rows.append("Wins & " + " & ".join(str(sum(1 for x in dev[p] if x < 1e-9)) if dev[p] else "--" for p in pairs) + " \\\\")
    cap = ("Shake threshold $\\theta$ and cap divisor $D$, $c = \\lceil k/D \\rceil$. Single-start ILS of $600$\\,s from R4, one seed. "
           "Objective per pair; bold marks the best per instance. Deviation rows and Wins as in Table~\\ref{tab:theta}.")
    hdr = ("\\begin{tabular}{l rr rr rr rr}\n\\toprule\n"
           "& " + " & ".join(f"\\multicolumn{{2}}{{c}}{{$\\theta = {t}$}}" for t in THETAS) + " \\\\\n"
           + "".join(f"\\cmidrule(lr){{{2 + 2 * i}-{3 + 2 * i}}}" for i in range(4)) + "\n"
           "Instance & " + " & ".join(f"$D = {d}$" for t in THETAS for d in DIVS) + " \\\\\n\\midrule\n")
    tab = hdr + "\n".join(rows) + "\n\\bottomrule\n\\end{tabular}"
    for key in PAPERS:
        C = load_paper(key)
        if "\\label{tab:theta-cap}" in C:
            C = replace_rows(C, "tab:theta-cap", rows)
            C = replace_caption(C, "tab:theta-cap", cap)
        else:
            if key == "arxiv":
                block = ("\\begin{table}[H]\n\\centering\\scriptsize\n\\setlength{\\tabcolsep}{4pt}\n"
                         "\\caption{" + cap + "}\\label{tab:theta-cap}\n" + tab + "\n\\end{table}\n\n")
                anchor = C.index("\\end{table}", C.index("\\label{tab:theta}")) + len("\\end{table}\n")
            else:
                block = ("\\begin{center}\n\\begin{minipage}{\\textwidth}\n\\centering\\scriptsize\n\\setlength{\\tabcolsep}{4pt}\n"
                         "\\captionof{table}{" + cap + "}\\label{tab:theta-cap}\n" + tab + "\n\\end{minipage}\n\\end{center}\n\n")
                anchor = C.index("\\end{center}", C.index("\\label{tab:theta}")) + len("\\end{center}\n")
            C = C[:anchor] + "\n" + block + C[anchor:]
        save_paper(key, C)
    print(f"captable: {len(cells)} cells written")


# ---------------------------------------------------------------- figures
def convergence():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    mis = {r["Instance"]: (float(r["Objective"]), float(r["Gap (%)"])) for r in rd(os.path.join(RES, "misocp_s1.csv"))}
    tdir = os.path.join(RES, "details", "ils_traces")
    ends = {r["Instance"]: min(float(r["Wall (s)"]), float(r["budget (s)"]))
            for r in rd(os.path.join(RES, "ils_variable.csv"))}
    fig, axes = plt.subplots(3, 5, figsize=(22, 11), squeeze=False)
    for ax, name in zip(axes.flat, ORDER):
        stem = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
        p = os.path.join(tdir, f"{stem}_variable.csv")
        if os.path.exists(p):
            rows = rd(p)
            w = [float(r["wall_s"]) for r in rows]; b = [float(r["best_obj"]) for r in rows]
            end = max(w[-1], ends.get(name, w[-1]))     # run end: the budget, or the early stop
            w.append(end); b.append(b[-1])
            ax.step(w, b, where="post", color="tab:blue", linewidth=1.8, label="ILS best")
            ax.set_xlim(0, end * 1.02)
        f, g = mis[name]
        ax.axhline(f, color="tab:red", linestyle="--", linewidth=1.2,
                   label="MISOCP optimum" if g <= 0.5 else "MISOCP incumbent")
        ax.set_title(name); ax.set_xlabel("wall-clock (s)"); ax.set_ylabel("Objective")
        ax.grid(True, alpha=0.3); ax.legend(loc="lower right", fontsize=8)
    for ax in list(axes.flat)[len(ORDER):]:
        ax.axis("off")
    fig.tight_layout()
    out = os.path.join(RES, "figures", "meta_convergence_timematched.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight"); plt.close(fig)
    for d in FIG_DIRS.values():
        shutil.copy(out, os.path.join(d, os.path.basename(out)))
    cap = ("ILS convergence under the protocol of Table~\\ref{tab:matheuristic-vs-exact}: best-seen objective against wall-clock time, "
           "one panel per instance. The dashed line is the MISOCP value at the $3\\,600$\\,s cap, a proven optimum or the incumbent.")
    for key in PAPERS:
        C = load_paper(key)
        if "\\label{fig:ils-convergence}" in C:
            C = replace_caption(C, "fig:ils-convergence", cap)
        else:
            block = ("\\begin{center}\n\\begin{minipage}{\\textwidth}\n\\centering\n"
                     "\\includegraphics[width=\\textwidth]{fig/meta_convergence_timematched.png}\n"
                     "\\captionof{figure}{" + cap + "}\n\\label{fig:ils-convergence}\n\\end{minipage}\n\\end{center}\n\n")
            anchor = C.index("\\end{center}", C.index("\\label{tab:fixed-speed}")) + len("\\end{center}\n")
            C = C[:anchor] + "\n" + block + C[anchor:]
        save_paper(key, C)
    print("convergence: figure written")


def perturb():
    subprocess.run([sys.executable, os.path.join(HERE, "make_perturbation_figure.py")], check=True)
    cap = ("Perturbation dynamics of the ILS on PR15 (240), one $600$\\,s run from R4 per pair $(\\theta, D)$. "
           "Grey, the current objective; colored, the best-seen objective; orange lines, the shake steps; dashed, the MISOCP incumbent.")
    for key in PAPERS:
        C = load_paper(key)
        if "\\label{fig:ils-perturbation}" in C:
            C = replace_caption(C, "fig:ils-perturbation", cap)
        else:
            anchor_lab = "\\label{tab:theta-cap}" if "\\label{tab:theta-cap}" in C else "\\label{tab:theta}"
            if key == "arxiv":
                block = ("\\begin{figure}[H]\n\\centering\n\\includegraphics[width=\\textwidth]{fig/ils_perturb_dynamics.png}\n"
                         "\\caption{" + cap + "}\n\\label{fig:ils-perturbation}\n\\end{figure}\n\n")
                anchor = C.index("\\end{table}", C.index(anchor_lab)) + len("\\end{table}\n")
            else:
                block = ("\\begin{center}\n\\begin{minipage}{\\textwidth}\n\\centering\n"
                         "\\includegraphics[width=\\textwidth]{fig/ils_perturb_dynamics.png}\n"
                         "\\captionof{figure}{" + cap + "}\n\\label{fig:ils-perturbation}\n\\end{minipage}\n\\end{center}\n\n")
                anchor = C.index("\\end{center}", C.index(anchor_lab)) + len("\\end{center}\n")
            C = C[:anchor] + "\n" + block + C[anchor:]
        save_paper(key, C)
    print("perturb: figure written")


def captions(theta, div):
    for key in PAPERS:
        C = load_paper(key)
        i = C.index("\\label{tab:matheuristic-vs-exact}"); k = C.rfind("\\caption", 0, i)
        seg = C[k:i]
        seg2 = re.sub(r"\\theta = \d+", f"\\\\theta = {theta}", seg)
        seg2 = re.sub(r"k/\d+", f"k/{div}", seg2)
        save_paper(key, C[:k] + seg2 + C[i:])
    print(f"captions: theta = {theta}, D = {div}")


def compile_papers():
    for key, path in PAPERS.items():
        d, f = os.path.split(path)
        for _ in range(2):
            subprocess.run(["pdflatex", "-interaction=nonstopmode", f], cwd=d,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        log = io.open(os.path.join(d, f.replace(".tex", ".log")), errors="replace").read()
        errs = sum(1 for l in log.splitlines() if l.startswith("!"))
        out = [l for l in log.splitlines() if "Output written" in l]
        print(f"compile {key}: errors {errs}; {out[-1] if out else 'NO OUTPUT'}")


if __name__ == "__main__":
    args = sys.argv[1:]
    if "select-pair" in args:
        t, d = select_pair(); print(t, d); sys.exit(0)
    if "captions" in args:
        i = args.index("captions"); captions(int(args[i + 1]), int(args[i + 2]))
    for step in ("table9", "table10", "captable", "convergence", "perturb"):
        if step in args:
            globals()[step]()
    if "compile" in args:
        compile_papers()
