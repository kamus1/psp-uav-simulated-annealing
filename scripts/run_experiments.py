#!/usr/bin/env python3
"""
Lanza baterías de pruebas para PSP-UAV combinando instancias, iteraciones,
drones y ticks. Soporta múltiples experimentos declarados en un JSON de
configuración y guarda reportes `.txt` y `.csv` en la carpeta `experiments/`.
"""
from __future__ import annotations

import argparse
import json
import csv
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
BINARY_PATH = REPO_ROOT / "PSP-UAV"
CONFIG_PATH_DEFAULT = REPO_ROOT / "scripts" / "experiments_config.json"

# Valores por defecto cuando no se proporciona archivo JSON (o faltan campos).
DEFAULT_CONFIG = {
    "instances": [
        "instancias/PSP-UAV_01_a.txt",
        "instancias/PSP-UAV_01_b.txt",
        "instancias/PSP-UAV_02_a.txt",
        "instancias/PSP-UAV_02_b.txt",
        "instancias/PSP-UAV_03_a.txt",
        "instancias/PSP-UAV_03_b.txt",
    ],
    "iterations": [1000, 3000],
    "iterations_by_instance": {},
    "drones": [2, 3],
    "ticks": [50],
    "repeats": 3,
    "export_data": False,
    "include_time_breakdown": False,
    "export_csv": True,
}

OUTPUT_DIR = REPO_ROOT / "experiments"


@dataclass
class Experiment:
    name: str
    instances: List[str]
    iterations: List[int]
    iterations_by_instance: Dict[str, List[int]]
    drones: List[int]
    ticks: List[int]
    repeats: int
    export_data: bool
    include_time_breakdown: bool
    export_csv: bool


@dataclass
class RunOutcome:
    experiment: str
    instance: str
    drones: int
    iterations: int
    ticks: int
    repeat_index: int
    return_code: int
    metrics: Dict[str, float]
    stdout: str
    stderr: str
    command: List[str]
    instance_stem: str
    instance_variant: str  # a/b/cuando aplica


def ensure_binary_exists() -> None:
    if BINARY_PATH.exists():
        return
    print(f"No se encontró el binario en {BINARY_PATH}. Ejecutando 'make'...")
    try:
        subprocess.run(["make"], cwd=REPO_ROOT, check=True)
    except subprocess.CalledProcessError as exc:
        raise SystemExit(f"Falló la compilación del binario: {exc}") from exc


def verify_instances(instances: List[str]) -> None:
    missing = [inst for inst in instances if not (REPO_ROOT / inst).exists()]
    if missing:
        raise SystemExit(f"No se encontraron las siguientes instancias: {', '.join(missing)}")


def iterations_for_instance(instance: str, exp: Experiment) -> List[int]:
    return exp.iterations_by_instance.get(instance, exp.iterations)


def parse_metrics(output: str, include_time_breakdown: bool) -> Dict[str, float]:
    patterns = {
        "urgencia": r"Urgencia acumulada:\s+(-?\d+)",
        "tiempo_ejecucion": r"Tiempo de ejecución:\s+([0-9]*\.?[0-9]+)\s+segundos",
        "colisiones": r"Colisiones detectadas:\s+(-?\d+)",
    }
    if include_time_breakdown:
        patterns["tiempo_evaluador"] = r"Tiempo acumulado Evaluador:\s+([0-9]*\.?[0-9]+)\s+segundos"
        patterns["tiempo_decodificador"] = r"Tiempo acumulado Decodificador:\s+([0-9]*\.?[0-9]+)\s+segundos"

    metrics: Dict[str, float] = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, output)
        if not match:
            continue
        value = match.group(1)
        metrics[key] = float(value) if "." in value else float(int(value))
    return metrics


def run_single(exp: Experiment, instance: str, drones: int, iterations: int, ticks: int, repeat_index: int) -> RunOutcome:
    cmd = [
        str(BINARY_PATH),
        str(REPO_ROOT / instance),
        str(drones),
        str(iterations),
        str(ticks),
    ]
    if exp.export_data:
        cmd.append("--export")
    if exp.include_time_breakdown:
        cmd.append("--times")

    print(f"[+] Ejecutando {Path(instance).name} | drones={drones} | iter={iterations} | "
          f"T={ticks} | run {repeat_index}/{exp.repeats} | exp={exp.name}")
    completed = subprocess.run(cmd, capture_output=True, text=True)
    metrics = parse_metrics(completed.stdout, exp.include_time_breakdown) if completed.returncode == 0 else {}

    resumen = (
        f"   --> Resultado | solucion={metrics.get('urgencia', 'NA')} | "
        f"tiempo={metrics.get('tiempo_ejecucion', 'NA')}s | T={ticks} | "
        f"colisiones={metrics.get('colisiones', 'NA')}"
    )
    print(resumen)

    stem = Path(instance).stem
    match_variant = re.search(r"_([ab])$", stem)
    variant = match_variant.group(1) if match_variant else ""

    return RunOutcome(
        experiment=exp.name,
        instance=instance,
        instance_stem=stem,
        instance_variant=variant,
        drones=drones,
        iterations=iterations,
        ticks=ticks,
        repeat_index=repeat_index,
        return_code=completed.returncode,
        metrics=metrics,
        stdout=completed.stdout,
        stderr=completed.stderr,
        command=cmd,
    )


def run_experiment(exp: Experiment) -> List[RunOutcome]:
    verify_instances(exp.instances)

    outcomes: List[RunOutcome] = []
    for instance in exp.instances:
        for iterations in iterations_for_instance(instance, exp):
            for ticks in exp.ticks:
                for drones in exp.drones:
                    for rep in range(1, exp.repeats + 1):
                        outcomes.append(run_single(exp, instance, drones, iterations, ticks, rep))
    return outcomes


def list_from(value, default):
    if value is None:
        return list(default)
    if isinstance(value, list):
        return value
    return [value]


def dict_from(value, default):
    if isinstance(value, dict):
        return value
    return dict(default)


def int_list(values) -> List[int]:
    return [int(v) for v in values]


def slugify(name: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "_", name).strip("_").lower()
    return slug or "exp"


def load_experiments(config_path: Path) -> Tuple[List[Experiment], bool]:
    if not config_path.exists():
        exp = Experiment(
            name="default",
            instances=list(DEFAULT_CONFIG["instances"]),
            iterations=list(DEFAULT_CONFIG["iterations"]),
            iterations_by_instance=dict(DEFAULT_CONFIG["iterations_by_instance"]),
            drones=list(DEFAULT_CONFIG["drones"]),
            ticks=list(DEFAULT_CONFIG["ticks"]),
            repeats=int(DEFAULT_CONFIG["repeats"]),
            export_data=bool(DEFAULT_CONFIG["export_data"]),
            include_time_breakdown=bool(DEFAULT_CONFIG["include_time_breakdown"]),
            export_csv=bool(DEFAULT_CONFIG["export_csv"]),
        )
        return [exp], False

    with config_path.open("r", encoding="utf-8") as f:
        try:
            data = json.load(f)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"Error al leer config JSON: {exc}") from exc

    defaults_raw = data.get("defaults", {})
    defaults = {
        "instances": list_from(defaults_raw.get("instances"), DEFAULT_CONFIG["instances"]),
        "iterations": int_list(list_from(defaults_raw.get("iterations"), DEFAULT_CONFIG["iterations"])),
        "iterations_by_instance": dict_from(defaults_raw.get("iterations_by_instance"), DEFAULT_CONFIG["iterations_by_instance"]),
        "drones": int_list(list_from(defaults_raw.get("drones"), DEFAULT_CONFIG["drones"])),
        "ticks": int_list(list_from(defaults_raw.get("ticks"), DEFAULT_CONFIG["ticks"])),
        "repeats": int(defaults_raw.get("repeats", DEFAULT_CONFIG["repeats"])),
        "export_data": bool(defaults_raw.get("export_data", DEFAULT_CONFIG["export_data"])),
        "include_time_breakdown": bool(defaults_raw.get("include_time_breakdown", DEFAULT_CONFIG["include_time_breakdown"])),
        "export_csv": bool(defaults_raw.get("export_csv", DEFAULT_CONFIG["export_csv"])),
    }

    experiments_data = data.get("experiments", [])
    experiments: List[Experiment] = []
    if experiments_data:
        for idx, raw in enumerate(experiments_data):
            merged = {
                **defaults,
                **{k: v for k, v in raw.items() if v is not None},
            }
            exp = Experiment(
                name=str(raw.get("name", f"exp_{idx+1}")),
                instances=list_from(raw.get("instances"), merged["instances"]),
                iterations=int_list(list_from(raw.get("iterations"), merged["iterations"])),
                iterations_by_instance=dict_from(raw.get("iterations_by_instance"), merged["iterations_by_instance"]),
                drones=int_list(list_from(raw.get("drones"), merged["drones"])),
                ticks=int_list(list_from(raw.get("ticks"), merged["ticks"])),
                repeats=int(raw.get("repeats", merged["repeats"])),
                export_data=bool(raw.get("export_data", merged["export_data"])),
                include_time_breakdown=bool(raw.get("include_time_breakdown", merged["include_time_breakdown"])),
                export_csv=bool(raw.get("export_csv", merged["export_csv"])),
            )
            experiments.append(exp)
    else:
        experiments.append(
            Experiment(
                name="default",
                instances=defaults["instances"],
                iterations=defaults["iterations"],
                iterations_by_instance=defaults["iterations_by_instance"],
                drones=defaults["drones"],
                ticks=defaults["ticks"],
                repeats=defaults["repeats"],
                export_data=defaults["export_data"],
                include_time_breakdown=defaults["include_time_breakdown"],
                export_csv=defaults["export_csv"],
            )
        )

    return experiments, True


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def build_report(exp: Experiment, outcomes: List[RunOutcome]) -> str:
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines: List[str] = []
    lines.append(f"PSP-UAV batch run - {timestamp} | Experimento: {exp.name}")
    lines.append(f"Binario: {BINARY_PATH}")
    lines.append(f"Instancias: {', '.join(exp.instances)}")
    lines.append(f"Iteraciones por defecto: {exp.iterations}")
    if exp.iterations_by_instance:
        lines.append(f"Iteraciones específicas: {exp.iterations_by_instance}")
    lines.append(f"Drones: {exp.drones}")
    lines.append(f"T (ticks): {exp.ticks}")
    lines.append(f"Repeticiones por combinación: {exp.repeats}")
    lines.append(f"--export: {exp.export_data} (sobrescribe exported_data/ en cada corrida)")
    lines.append(f"--times: {exp.include_time_breakdown}")
    lines.append("")
    lines.append("=== Resultados individuales ===")

    for outcome in outcomes:
        lines.append(
            f"[{outcome.instance}] drones={outcome.drones} iter={outcome.iterations} T={outcome.ticks} "
            f"rep={outcome.repeat_index}/{exp.repeats}"
        )
        lines.append(f"cmd: {' '.join(outcome.command)}")
        lines.append(f"exit_code: {outcome.return_code}")
        if outcome.metrics:
            for key, value in outcome.metrics.items():
                lines.append(f"{key}: {value}")
        else:
            lines.append("No se pudieron extraer métricas (posible error en la ejecución).")
        if outcome.stderr.strip():
            lines.append("stderr:")
            lines.append(outcome.stderr.strip())
        if outcome.stdout.strip():
            lines.append("stdout:")
            lines.append(outcome.stdout.strip())
        lines.append("")

    lines.append("=== Promedios por combinación ===")
    grouped: Dict[Tuple[str, int, int, int], List[RunOutcome]] = {}
    for outcome in outcomes:
        key = (outcome.instance, outcome.drones, outcome.iterations, outcome.ticks)
        grouped.setdefault(key, []).append(outcome)

    for (instance, drones, iterations, ticks), runs in grouped.items():
        lines.append(f"[{instance}] drones={drones} iter={iterations} T={ticks} runs={len(runs)}")
        for metric_name in ("urgencia", "colisiones", "tiempo_ejecucion", "tiempo_evaluador", "tiempo_decodificador"):
            values = [r.metrics[metric_name] for r in runs if metric_name in r.metrics]
            if values:
                lines.append(f"promedio_{metric_name}: {mean(values):.6f}")
        lines.append("")

    return "\n".join(lines)


def save_csv(outcomes: List[RunOutcome], path: Path) -> None:
    headers = [
        "experiment",
        "instance",
        "instance_stem",
        "instance_variant",
        "drones",
        "iterations",
        "ticks",
        "repeat",
        "exit_code",
        "urgencia",
        "colisiones",
        "tiempo_ejecucion",
        "tiempo_evaluador",
        "tiempo_decodificador",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        for o in outcomes:
            m = o.metrics
            writer.writerow({
                "experiment": o.experiment,
                "instance": o.instance,
                "instance_stem": o.instance_stem,
                "instance_variant": o.instance_variant,
                "drones": o.drones,
                "iterations": o.iterations,
                "ticks": o.ticks,
                "repeat": o.repeat_index,
                "exit_code": o.return_code,
                "urgencia": m.get("urgencia"),
                "colisiones": m.get("colisiones"),
                "tiempo_ejecucion": m.get("tiempo_ejecucion"),
                "tiempo_evaluador": m.get("tiempo_evaluador"),
                "tiempo_decodificador": m.get("tiempo_decodificador"),
            })


def save_report(report: str, exp_name: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = OUTPUT_DIR / f"run_{stamp}_{slugify(exp_name)}.txt"
    output_file.write_text(report, encoding="utf-8")
    return output_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Runner de experimentos PSP-UAV.")
    parser.add_argument(
        "-c",
        "--config",
        default=str(CONFIG_PATH_DEFAULT),
        help="Ruta al JSON de experimentos (por defecto scripts/experiments_config.json).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    experiments, from_file = load_experiments(config_path)

    if from_file:
        print(f"Usando configuración desde: {config_path}")
    else:
        print(f"No se encontró config en {config_path}, usando valores por defecto embebidos.")

    ensure_binary_exists()
    OUTPUT_DIR.mkdir(exist_ok=True)

    for exp in experiments:
        if exp.repeats < 1:
            raise SystemExit(f"REPEATS debe ser al menos 1 (experimento {exp.name}).")
        print(f"\n=== Experimento: {exp.name} ===")
        outcomes = run_experiment(exp)
        report = build_report(exp, outcomes)
        output_file = save_report(report, exp.name)
        print(f"Reporte guardado en: {output_file}")
        if exp.export_csv:
            csv_path = output_file.with_suffix(".csv")
            save_csv(outcomes, csv_path)
            print(f"CSV guardado en: {csv_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit("\nEjecución interrumpida por el usuario.")
