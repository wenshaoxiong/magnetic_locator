import argparse
import math
from pathlib import Path

import numpy as np
import yaml


def dipole_field_T(p, src_pos, m):
    r = p - src_pos
    r2 = float(np.dot(r, r))
    rnorm = math.sqrt(r2)
    if rnorm < 1e-9:
        return np.zeros(3)
    rhat = r / rnorm
    term = 3.0 * float(np.dot(m, rhat)) * rhat - m
    return 1e-7 * term / (r2 * rnorm)


def load_sources(map_yaml: Path):
    doc = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    out = []
    for s in doc.get("sources", []):
        out.append(
            {
                "id": str(s.get("id", "")),
                "position_m": np.array(s.get("position_m", [0.0, 0.0, 0.0]), dtype=float),
                "moment": np.array(s.get("moment", [0.0, 0.0, 1.0]), dtype=float),
            }
        )
    if not out:
        raise RuntimeError("no sources in magnetic_map.yaml")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="magnetic_map.yaml")
    ap.add_argument("--out", required=True, help="output npz (ZIP_STORED)")
    ap.add_argument("--xmin", type=float, default=-0.2)
    ap.add_argument("--xmax", type=float, default=0.2)
    ap.add_argument("--ymin", type=float, default=-0.2)
    ap.add_argument("--ymax", type=float, default=0.2)
    ap.add_argument("--zmin", type=float, default=0.0)
    ap.add_argument("--zmax", type=float, default=0.3)
    ap.add_argument("--nx", type=int, default=81)
    ap.add_argument("--ny", type=int, default=81)
    ap.add_argument("--nz", type=int, default=81)
    args = ap.parse_args()

    sources = load_sources(Path(args.map))
    x = np.linspace(args.xmin, args.xmax, args.nx, dtype=np.float64)
    y = np.linspace(args.ymin, args.ymax, args.ny, dtype=np.float64)
    z = np.linspace(args.zmin, args.zmax, args.nz, dtype=np.float64)

    B = np.zeros((args.nx, args.ny, args.nz, 3), dtype=np.float64)
    for ix, xv in enumerate(x):
        for iy, yv in enumerate(y):
            for iz, zv in enumerate(z):
                p = np.array([xv, yv, zv], dtype=float)
                b = np.zeros(3)
                for s in sources:
                    b += dipole_field_T(p, s["position_m"], s["moment"])
                B[ix, iy, iz, :] = b

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out_path, x=x, y=y, z=z, B=B)


if __name__ == "__main__":
    main()

