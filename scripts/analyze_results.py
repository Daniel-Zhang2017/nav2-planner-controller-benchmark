#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Post-process benchmark CSVs into a summary table, plots and a Markdown report.

Usage:
    python3 scripts/analyze_results.py --csv results/run.csv \
        --out results/analysis

Outputs in --out:
    summary.csv          per controller: mean +/- std over all trials
    by_scenario.csv      per controller x scenario-type (detour/straight)
    summary.md           human-readable report
    plots/*.png          box plots + Pareto scatter + success-rate bar
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

METRICS = [
    ('completion_time_s', 'Time to goal [s]', 'time [s]', 'completion_time_s.png'),
    ('traveled_dist_m', 'Travelled distance [m]', 'distance [m]', 'traveled_dist_m.png'),
    ('cte_mean_m', 'Mean cross-track error [m]', 'CTE [m]', 'cte_mean_m.png'),
    ('cte_max_m', 'Max cross-track error [m]', 'CTE [m]', 'cte_max_m.png'),
    ('min_clearance_forward_m', 'Min forward clearance [m]', 'clearance [m]',
     'min_clearance_forward_m.png'),
    ('jerk_lin_mean', 'Mean linear jerk', '|jerk|', 'jerk_lin_mean.png'),
    ('lin_oscillations', 'Linear oscillations [#]', 'oscillations',
     'lin_oscillations.png'),
    ('avg_lin_vel', 'Avg linear speed [m/s]', 'speed [m/s]', 'avg_lin_vel.png'),
]

CONTROLLER_ORDER = ['mppi', 'dwb', 'rpp', 'pid_safe', 'lqr_safe']


def mean_std(s):
    s = pd.to_numeric(s, errors='coerce')
    return f"{s.mean():.3f} ± {s.std():.3f}" if s.notna().any() else '-'


def fmt_pct(v):
    return f"{100.0 * v:.0f}%"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--out', default='results/analysis')
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df['scenario_type'] = df['scenario_id'].str.extract(r'_(detour|straight)$')[0]
    df['controller'] = pd.Categorical(df['controller'], CONTROLLER_ORDER)
    out = Path(args.out)
    (out / 'plots').mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------- summary
    cols = ['controller', 'scenario_type', 'n', 'success_rate'] + \
           [c for c, _, _, _ in METRICS]
    rows = []
    for ctrl, grp in df.groupby('controller', observed=True):
        r = {'controller': ctrl, 'scenario_type': 'all', 'n': len(grp),
             'success_rate': fmt_pct(grp['success'].mean())}
        for c, _, _, _ in METRICS:
            r[c] = mean_std(grp[c])
        rows.append(r)
        for st, g2 in grp.groupby('scenario_type'):
            r = {'controller': ctrl, 'scenario_type': st, 'n': len(g2),
                 'success_rate': fmt_pct(g2['success'].mean())}
            for c, _, _, _ in METRICS:
                r[c] = mean_std(g2[c])
            rows.append(r)
    summary = pd.DataFrame(rows)[cols]
    summary.to_csv(out / 'summary.csv', index=False)

    by_scenario = df.groupby(['controller', 'scenario_id'], observed=True).agg(
        n=('trial', 'count'),
        success_rate=('success', 'mean'),
        completion_time_mean=('completion_time_s', 'mean'),
        cte_mean=('cte_mean_m', 'mean'),
        cte_max=('cte_max_m', 'mean'),
        min_clearance_forward=('min_clearance_forward_m', 'min'),
    ).round(3).reset_index()
    by_scenario.to_csv(out / 'by_scenario.csv', index=False)

    # ------------------------------------------------------------- markdown
    md = ['# Controller benchmark report',
          '',
          f'- CSV source: `{args.csv}`',
          f'- trials analysed: `{len(df)}`',
          '',
          '## Success rate (all scenarios)', '']
    md.append('| controller | n | success |')
    md.append('|---|---|---|')
    for ctrl, grp in df.groupby('controller', observed=True):
        md.append(f'| {ctrl} | {len(grp)} | {fmt_pct(grp["success"].mean())} |')
    md += ['', '## Metrics (mean ± std over trials)', '',
           '| controller | ' + ' | '.join(c for c, _, _, _ in METRICS) + ' |',
           '|---|' + '---|' * len(METRICS)]
    for _, r in summary[summary['scenario_type'] == 'all'].iterrows():
        cells = ' | '.join(str(r[c]) for c, _, _, _ in METRICS)
        md.append(f'| {r["controller"]} {cells} |')
    md += ['', '## Per scenario type (detour / straight)', '']
    for _, r in summary.iterrows():
        md.append(f"- `{r['controller']}` [{r['scenario_type']}]: "
                  f"success={r['success_rate']}, "
                  f"time={r['completion_time_s']}, "
                  f"cte_mean={r['cte_mean_m']}, "
                  f"min_clearance_fwd={r['min_clearance_forward_m']}")
    (out / 'summary.md').write_text('\n'.join(md) + '\n')

    # ------------------------------------------------------------- plots
    palette = plt.cm.tab10.colors
    order = [c for c in CONTROLLER_ORDER if c in df['controller'].cat.categories]

    def box(col, title, ylabel, fname, log=False):
        fig, ax = plt.subplots(figsize=(7, 4.2))
        data = [pd.to_numeric(df[df['controller'] == c][col], errors='coerce')
                .dropna().values for c in order]
        bp = ax.boxplot(data, patch_artist=True, showfliers=False)
        ax.set_xticks(range(1, len(order) + 1))
        ax.set_xticklabels(order)
        for patch, c in zip(bp['boxes'], palette):
            patch.set_facecolor(c)
            patch.set_alpha(0.6)
        if log:
            ax.set_yscale('log')
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        fig.tight_layout()
        fig.savefig(out / 'plots' / fname, dpi=140)
        plt.close(fig)

    # success rate
    fig, ax = plt.subplots(figsize=(7, 4.2))
    rates = [df[df['controller'] == c]['success'].mean() for c in order]
    ax.bar(order, rates, color=palette[:len(order)], alpha=0.8)
    ax.set_ylim(0, 1.05)
    ax.set_title('Success rate by controller')
    ax.set_ylabel('success rate')
    for i, v in enumerate(rates):
        ax.text(i, v + 0.02, f'{v:.0%}', ha='center')
    fig.tight_layout()
    fig.savefig(out / 'plots' / 'success_rate.png', dpi=140)
    plt.close(fig)

    for col, title, ylabel, fname in METRICS:
        box(col, f'{title} by controller', ylabel, f'{col}.png',
            log=(col in ('jerk_lin_mean', 'jerk_lin_max')))

    # Pareto: max CTE vs completion time
    fig, ax = plt.subplots(figsize=(7, 5))
    for c, color in zip(order, palette):
        g = df[df['controller'] == c]
        ax.scatter(g['completion_time_s'], g['cte_max_m'],
                   label=c, color=color, alpha=0.7, s=40)
    ax.set_xlabel('completion time [s]')
    ax.set_ylabel('max cross-track error [m]')
    ax.set_title('Pareto: accuracy vs speed (lower-left is better)')
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / 'plots' / 'pareto_cte_time.png', dpi=140)
    plt.close(fig)

    print(f'analysis written to {out}')
    print(out / 'summary.md')


if __name__ == '__main__':
    main()
#（注：内容由AI生成）
