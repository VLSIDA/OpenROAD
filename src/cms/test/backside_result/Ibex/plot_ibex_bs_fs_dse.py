#!/usr/bin/env python3
# Side-by-side DSE Pareto frontiers: BACKSIDE (100ps-budget study) vs FRONTSIDE.
# Shared axes so the two frontiers are directly comparable. Each panel shows
# its frontier points (distinct colors) + a color-matched config table.
import optuna
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

I = "/home/wali2/backside/OpenROAD/src/cms/test/backside_result/Ibex"

def load_front(study_name, db):
    st = optuna.load_study(study_name=study_name, storage=f"sqlite:///{I}/{db}")
    done = [t for t in st.trials if t.state.name == "COMPLETE"]
    seen, uniq = set(), []
    for t in done:
        k = tuple(sorted(t.params.items()))
        if k not in seen:
            seen.add(k); uniq.append(t)
    pts = [(t.values[1], t.values[0], t.params,
            t.user_attrs.get('insertion_ps', None)) for t in uniq]
    front = [q for q in pts if not any(
        p2 <= q[0] and s2 <= q[1] and (p2 < q[0] or s2 < q[1])
        for p2, s2, _, _ in pts)]
    front.sort()
    return front, len(pts)

bs, nbs = load_front("ibex_mesh_dse_s100", "dse_s100/ibex_dse_s100.db")
fs, nfs = load_front("ibex_fs_mesh_dse", "dse_fs/ibex_fs_dse.db")

def bs_tag(p): return (f"{p['PITCH']:>4} {p['MBUF']:>4} {p['FMAX']:>3} "
                       f"{p['SBUF']:>3} {p['CLKLAYERS']:>6}")
def fs_tag(p): return (f"{p['PITCH']:>4} {p['MBUF']:>4} {p['MESHLAYERS']:>5} "
                       f"{p['CLKLAYERS']:>6}")
BS_HDR = (f"{'':2s} {'skew':>7} {'power':>7} {'ins':>6}  {'Ptch':>4} {'Mbuf':>4}"
          f" {'Fmx':>3} {'Sbf':>3} {'Layers':>6}")
FS_HDR = (f"{'':2s} {'skew':>7} {'power':>7} {'ins':>6}  {'Ptch':>4} {'Mbuf':>4}"
          f" {'Mesh':>5} {'Layers':>6}")

cmap = list(plt.cm.tab10.colors) + list(plt.cm.Dark2.colors) + list(plt.cm.Set1.colors)
allp = [q[0] for q in bs + fs]; alls = [q[1] for q in bs + fs]
xlim = (min(allp) * 0.92, max(allp) * 1.06)
ylim = (0, max(alls) * 1.10)

fig, axes = plt.subplots(1, 2, figsize=(16, 6.8), sharey=True)
for ax, front, n, title, tagf, hdr in [
        (axes[0], bs, nbs, f"(a) BACKSIDE mesh  ({nbs} valid configs)", bs_tag, BS_HDR),
        (axes[1], fs, nfs, f"(b) FRONTSIDE mesh  ({nfs} valid configs)", fs_tag, FS_HDR)]:
    ax.plot([q[0] for q in front], [q[1] for q in front], "-",
            color="#9e9e9e", lw=1.3, zorder=3)
    for i, (pw, sk, prm, ins) in enumerate(front):
        ax.scatter([pw], [sk], s=150, marker="o", color=cmap[i % len(cmap)],
                   edgecolors="k", linewidths=0.6, zorder=4)
        ax.annotate(chr(65 + i), (pw, sk), textcoords="offset points",
                    xytext=(-11, 5), fontsize=9, fontweight="bold")
    ax.set_xlim(*xlim); ax.set_ylim(*ylim)
    ax.set_xlabel("Clock power (mW)")
    ax.set_title(title, fontweight="bold", fontsize=12)
    ax.grid(alpha=0.3)
    from matplotlib.patches import FancyBboxPatch
    y0, dy = 0.955, 0.0235
    panel_h = (len(front) + 1.8) * dy
    ax.add_patch(FancyBboxPatch((0.415, y0 - panel_h + dy), 0.57, panel_h,
                                boxstyle="round,pad=0.006",
                                transform=ax.transAxes, fc="white",
                                ec="#9e9e9e", alpha=0.93, zorder=5))
    ax.text(0.975, y0, hdr, transform=ax.transAxes, fontsize=7.2, va="top",
            ha="right", family="monospace", fontweight="bold", zorder=6)
    for i, q in enumerate(front):
        ins_s = f"{q[3]:5.0f}p" if q[3] is not None else "    --"
        ax.text(0.975, y0 - (i + 1) * dy,
                f"{chr(65+i)}: {q[1]:5.1f}ps {q[0]:5.2f}mW {ins_s}  {tagf(q[2])}",
                transform=ax.transAxes, fontsize=7.2, va="top", ha="right",
                family="monospace", color=cmap[i % len(cmap)],
                fontweight="bold", zorder=6)
axes[0].set_ylabel("Worst-case skew, max-min (ps)")
fig.suptitle("Ibex - Skew vs Power: DSE Pareto frontiers, backside vs frontside"
             " (shared axes, 100ps slew budget)", fontweight="bold", fontsize=13)
out = f"{I}/ibex_bs_fs_dse_front.pdf"
plt.tight_layout()
plt.savefig(out, bbox_inches="tight", dpi=130)
plt.savefig(out.replace(".pdf", ".png"), bbox_inches="tight", dpi=130)
print("wrote", out)
print(f"BS frontier {len(bs)} pts")
print(f"FS frontier {len(fs)} pts")
