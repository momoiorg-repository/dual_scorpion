"""Render selected mocopi metrics as a dependency-free comparison SVG."""

import json
from pathlib import Path

from .report_csv import SCHEMA_PREFIX, _flatten

PLOT_SCHEMA = "dual_scorpion.mocopi.metrics_plot.v1"


def _escape(value):
    return str(value).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")


def _label(path, limit=38):
    text = str(path)
    return text if len(text) <= limit else "…" + text[-limit:]


def _panel(metric, rows, top, width):
    values = [row.get(metric) for row in rows.values()]
    available = [value for value in values if isinstance(value, (int, float)) and not isinstance(value, bool)]
    if not available:
        raise ValueError(f"Metric has no numeric value in any report: {metric}")
    minimum = min(min(available), 0.0)
    maximum = max(max(available), 0.0)
    if minimum == maximum:
        maximum = minimum + 1.0

    left, plot_width = 250, width - 320

    def scale(value):
        return left + plot_width * (float(value) - minimum) / (maximum - minimum)

    zero_x = scale(0.0)
    row_height, first_row_top = 26, top + 44
    height = first_row_top + row_height * len(rows) + 24
    parts = [
        f'<text x="24" y="{top + 22}" class="metric">{_escape(metric)}</text>',
        f'<line x1="{left}" y1="{top + 32}" x2="{left + plot_width}" y2="{top + 32}" class="axis"/>',
        f'<line x1="{zero_x:.2f}" y1="{top + 32}" x2="{zero_x:.2f}" y2="{height - 16}" class="zero"/>',
    ]
    for index, (report, row) in enumerate(rows.items()):
        y = first_row_top + index * row_height
        value = row.get(metric)
        parts.append(f'<text x="24" y="{y + 12}" class="report">{_escape(_label(report))}</text>')
        if value is None:
            parts.append(f'<text x="{zero_x + 6:.2f}" y="{y + 12}" class="missing">missing</text>')
            continue
        x = scale(value)
        x1, width_bar = min(x, zero_x), abs(x - zero_x)
        css = "positive" if value >= 0 else "negative"
        parts.append(f'<rect x="{x1:.2f}" y="{y}" width="{width_bar:.2f}" height="14" class="bar {css}"/>')
        text_x = max(x, zero_x) + 6
        parts.append(f'<text x="{text_x:.2f}" y="{y + 12}" class="value">{value:.6g}</text>')
    return height, "\n".join(parts)


def export_metrics_svg(report_paths, metrics, output_path):
    """Write one comparison panel per requested metric and return a summary."""
    if not metrics or not isinstance(metrics, list):
        raise ValueError("At least one metric is required")
    if len(metrics) != len(set(metrics)):
        raise ValueError("Metrics must not contain duplicates")
    output = Path(output_path)
    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")
    rows = {}
    for path in report_paths:
        path = Path(path)
        document = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or not str(document.get("schema", "")).startswith(SCHEMA_PREFIX):
            raise ValueError(f"Not a mocopi evaluation report: {path}")
        if str(path) in rows:
            raise ValueError(f"Report listed more than once: {path}")
        rows[str(path)] = _flatten(document)

    width = 860
    panels, top = [], 16
    for metric in metrics:
        panel_height, markup = _panel(metric, rows, top, width)
        panels.append(markup)
        top = panel_height + 18
    height = top + 8
    provenance = {
        "schema": PLOT_SCHEMA,
        "reports": list(rows),
        "metrics": list(metrics),
    }
    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <title>Dual Scorpion mocopi metrics comparison</title>
  <desc>{_escape(json.dumps(provenance, ensure_ascii=False, sort_keys=True))}</desc>
  <style>
    background {{ fill: #ffffff; }}
    metric {{ font: 600 15px sans-serif; fill: #172554; }}
    report {{ font: 12px sans-serif; fill: #334155; }}
    value {{ font: 12px sans-serif; fill: #0f172a; }}
    missing {{ font: italic 12px sans-serif; fill: #64748b; }}
    axis {{ stroke: #94a3b8; stroke-width: 1; }}
    zero {{ stroke: #475569; stroke-width: 1; stroke-dasharray: 3 3; }}
    bar {{ stroke: none; }}
    positive {{ fill: #2563eb; }}
    negative {{ fill: #ea580c; }}
  </style>
  <rect class="background" width="{width}" height="{height}"/>
  {"\n  ".join(panels)}
</svg>
'''
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8", newline="\n") as file:
        file.write(svg)
    return {
        "schema": PLOT_SCHEMA,
        "reports": len(rows),
        "metrics": list(metrics),
        "output": str(output),
        "missing_values": {
            metric: sum(rows[str(path)].get(metric) is None for path in report_paths) for metric in metrics
        },
    }
