#

Below is a self‑contained, production‑oriented **README.md** you can drop into the same directory as `f47_mq28_cad_opt.py`.
It explains what the script does, how to run it in a reproducible way, and—crucially—how to stand the software stack up **inside mainland China (PRC)** where some default package mirrors and binaries may be blocked.

---

```markdown
# F‑47 / MQ‑28 Force‑Mix & CAD Optimiser

**Single‑file reference implementation:** `f47_mq28_cad_opt.py`  
© 2025 • MIT Licence

---

## 1  What does this program do?

| Stage | Purpose | Key Tech |
|-------|---------|----------|
| **ILP force‑mix optimiser** | Finds the best quantity mix of MQ‑28 drones, F‑47 fighters, E‑7A AWACS and TPS‑77 radars under cost / capability constraints. | PuLP (CBC) integer programming |
| **Parametric CAD model** | Generates a simplified but fully parametric F‑47 outer‑mould‑line (OML). | FreeCAD ≥ 0.21 / CadQuery ≥ 2.5 |
| **Two‑stage shape optimiser** | 1) **Global**: Gaussian‑process Bayesian search<br>2) **Local**: SLSQP with exact gradients | scikit‑optimize, SciPy |
| **Manufacturability checks** | Ensures every generated part fits inside a **300 × 300 × 400 mm** FFF printer envelope and adds tongue‑and‑groove joints where needed. | custom helpers in script |
| **Outputs** | • Best force mix & utility score<br>• Optimised geometry metrics<br>• Individual STL files + build‑layout CSV | CadQuery exporters, Python CSV |

> **Notes on scope**  
> 1. The aerodynamic model is a *proxy* (Cd₀ ∝ wetted area).  
> 2. The CAD is intentionally simplistic—think “conceptual study”, **not** flight‑ready airframe.  
> 3. No classified data or proprietary aero models are used.

---

## 2  Directory layout (after first run)

```

project/
└── f47\_mq28\_cad\_opt.py
stl\_out/
part\_00.stl
part\_01.stl
…
build\_layout.csv
README.md        ← you are here

````

`build_layout.csv` lists each part’s bounding box (mm) and path so you can feed it into slicer automation or a MES system.

---

## 3  Quick‑start (cross‑platform)

### 3.1 Python 3.11 venv

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
python  -m pip install -U pip
````

### 3.2 Install core Python deps

```bash
pip install numpy scipy pulp scikit-optimize
```

### 3.3 Install FreeCAD + CadQuery

| OS                 | Recommended route                                                                                                                               |
| ------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **Windows/macOS**  | Download the latest *FreeCAD 1.0 AppImage/DMG/ZIP* from GitHub Releases and unzip/drag into `C:\Tools\FreeCAD` or `/Applications` ([GitHub][1]) |
| **Linux**          | Use the AppImage (`chmod +x FreeCAD‑1.0.AppImage`) or install via `conda`/`mamba`.                                                              |
| **Conda (any OS)** | `mamba create -n cq -c cadquery -c conda-forge cq-editor=2 cadquery=2.5` ([PyPI][2])                                                            |

### 3.4 Make FreeCAD’s Python visible

Add to `activate` script *or* set at shell:

```bash
export PYTHONPATH="$PYTHONPATH:/path/to/FreeCAD/lib"
```

(Drop the `export` keyword on Windows and use back‑slashes.)

### 3.5 Run the pipeline

```bash
python f47_mq28_cad_opt.py
```

The script prints force‑mix results, optimisation progress, and finally writes the STL modules.

---

## 4  Running **inside mainland China (PRC)**

Because direct access to GitHub, PyPI, and CDN hosts can be throttled or blocked, the following adaptations are recommended:

| Item                       | Mainland‑friendly approach                                                                                                                                                  |
| -------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **PyPI packages**          | `pip install -i https://pypi.tuna.tsinghua.edu.cn/simple numpy scipy …` (TUNA mirror)                                                                                       |
| **CadQuery conda channel** | Use Tsinghua’s `https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/` mirror and add `-c cadquery` after `-c conda-forge`.                                                  |
| **FreeCAD binaries**       | Sync the GitHub release asset via `ghproxy.com/` or use community mirror `https://mirrors.tuna.tsinghua.edu.cn/github-release/FreeCAD/FreeCAD/`.                            |
| **Large AppImages/DMGs**   | Alternatively compile FreeCAD 1.0 from source: `git clone --recursive https://github.com/FreeCAD/FreeCAD.git`, then follow `conda_linux.yml` for a **fully offline** build. |
| **APT/YUM deps**           | Use mirrors from Aliyun/USTC (`mirrors.aliyun.com`, `mirrors.ustc.edu.cn`).                                                                                                 |
| **Time sync**              | Ensure your CI or local PC’s clock is NTP‑synced; GPU hour quotas on domestic cloud providers (Alibaba Cloud, Huawei Cloud) are strict.                                     |

> **Tip**: Put the final wheel/AppImage tarballs on an internal Artifactory or OSS bucket so future CI images don’t hit external mirrors at all.

---

## 5  Productionisation checklist

| Area                      | Action                                                                                                                                                                                                       |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Packaging**             | Convert script → installable module (`setup.cfg`/`pyproject.toml`) exposing a `f47-opt` CLI entry‑point.                                                                                                     |
| **Logging**               | Replace `print()` with `logging` (JSON format + rotating file handler).                                                                                                                                      |
| **Config**                | Move all constants (printer size, DV bounds, weights) into a `config.yaml`; load via `pydantic` for validation.                                                                                              |
| **Tests**                 | Parameterised pytest suite covering ILP utility normalisation, CAD part‑fit logic, and optimisation convergence (mock CadQuery).                                                                             |
| **CI/CD**                 | GitHub Actions matrix (Ubuntu/Windows/macOS) + a secondary self‑hosted runner inside PRC to guard against mirror outages.                                                                                    |
| **Container**             | Multi‑stage Dockerfile:<br>1) build FreeCAD & CadQuery in condaforge image<br>2) copy slim runtime into `python:3.11-slim`.<br>Public image layers can be cached in Alibaba’s **ACR** for low‑latency pulls. |
| **Re‑entrancy**           | Persist optimiser state (GP model, best x) to disk so interrupted jobs resume.                                                                                                                               |
| **Perf**                  | Profile SLSQP with `scipy.optimize.LoggingCallback`; vectorise constraint evaluation; consider `pyocct` backend for CadQuery to avoid FreeCAD headless overhead.                                             |
| **Security & Compliance** | • Run `bandit` / `safety` on every commit.<br>• Verify OpenCASCADE licence compatibility.<br>• Perform EAR / PRC export‑control review before physical manufacture.                                          |

---

## 6  Limitations & disclaimers

1. **Aerodynamics**: Cd₀ proxy uses wetted‑area scaling—far from a CFD‑grade drag prediction.
2. **Structural validity**: Printed parts ignore spar loads, fasteners, and material anisotropy.
3. **Regulatory**: Manufacturing, exporting, or operating any airframe may require CAAC / PLAAT approvals; you are solely responsible for compliance.
4. **Ethics**: This repository is provided for research & educational use **only**. Do not employ it in ways that violate international humanitarian law or local regulations.

---

## 7  Change‑log snippet (major external deps)

| Package         | Minimum | Recommended (2025‑07) | Reference                         |
| --------------- | ------- | --------------------- | --------------------------------- |
| FreeCAD         | 0.21    | **1.0 stable**        | GitHub release page ([GitHub][1]) |
| CadQuery        | 2.2     | **2.5.2** (PyPI)      | PyPI release ([PyPI][2])          |
| SciPy           | 1.10    | 1.14                  | —                                 |
| scikit‑optimize | 0.9     | 0.10                  | —                                 |
| PuLP            | 2.8     | 2.9                   | —                                 |

---

## 8  Running benchmarks (optional)

```bash
python -m pytest tests/ -q
python scripts/benchmark.py --n-calls 50 --repeat 3 --json out/bench.json
```

Produces a JSON summary suitable for Grafana dashboards.

---

## 9  License

Released under the MIT licence (see `LICENSE.txt`). Third‑party libraries retain their respective licences.

```

---

**How to use**  

1. Save the above block as `README.md`.  
2. Commit it alongside `f47_mq28_cad_opt.py`.  
3. Follow the quick‑start to validate that the optimisation runs end‑to‑end on your workstation or CI runner.

*© 2025 – This README was generated automatically; tailor company‑specific sections (e.g., internal registry URLs) before publishing.*
::contentReference[oaicite:4]{index=4}
```

[1]: https://github.com/FreeCAD/FreeCAD/releases?utm_source=chatgpt.com "Releases · FreeCAD/FreeCAD - GitHub"
[2]: https://pypi.org/project/cadquery/?utm_source=chatgpt.com "cadquery - PyPI"
