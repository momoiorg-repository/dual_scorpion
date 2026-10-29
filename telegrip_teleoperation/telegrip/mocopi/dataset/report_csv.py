"""Combine saved evaluation reports into one comparison table.

Reports keep their full structure in JSON. This module writes a wide CSV with one
row per report so sessions, episodes and simulation models can be compared with a
spreadsheet or pandas. Nested values use dotted names, list lengths become
`.count` columns, and unavailable values stay empty. Nothing is inferred or
replaced by a default.
"""

import csv
import json
from pathlib import Path

SCHEMA_PREFIX = "dual_scorpion.mocopi."
IDENTITY = ("schema", "source", "capture_source", "episode", "task", "model_kind", "slowdown")


def _flatten(document, prefix="", flat=None):
    flat = {} if flat is None else flat
    for key, value in document.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            _flatten(value, f"{name}.", flat)
        elif isinstance(value, list):
            flat[f"{name}.count"] = len(value)
        elif value is None or isinstance(value, (bool, int, float, str)):
            flat[name] = value
        else:
            raise ValueError(f"Unsupported report value at {name}")
    return flat


def export_metrics_csv(report_paths, output_path):
    """Write one CSV row per report and return a summary of what was written."""
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
    columns = [key for key in IDENTITY if any(key in row for row in rows.values())]
    columns += sorted({key for row in rows.values() for key in row} - set(columns))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["report", *columns])
        for name, row in rows.items():
            values = ["" if row.get(column) is None else row[column] for column in columns]
            writer.writerow([name, *values])
    return {
        "schema": "dual_scorpion.mocopi.metrics_table.v1",
        "reports": len(rows),
        "columns": columns,
        "output": str(output),
    }
