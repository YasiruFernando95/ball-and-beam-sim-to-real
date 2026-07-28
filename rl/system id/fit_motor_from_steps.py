import json
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# EDIT THESE PATHS DIRECTLY (NO COMMAND LINE NEEDED)
# ============================================================
NO_LOAD_CSV = "no_load_test.csv"
BEAM_LOAD_CSV = "beam_load_test.csv"
OUT_PREFIX = "motor_id"
MIN_SAMPLES = 25
PLOT_RESULTS = True

RUN_ID_FILTER = None          # e.g., 1, 2, 3 ... or None for all
TEST_NAME_FILTER = None       # e.g., "NO_LOAD" or "BEAM_LOAD" or None
MIN_DELTA_W = 1e-3            # raise if your W noise is bigger, e.g., 0.5 or 1.0

# ============================================================


STEP_RE = re.compile(r"^(STEP_POS|STEP_NEG)_(\d+(\.\d+)?)$")


@dataclass
class StepFit:
    dataset: str
    phase: str
    u_cmd: float
    w0: float
    wss: float
    K: float
    tau: float
    n: int


def _require_cols(df: pd.DataFrame, cols: List[str], name: str):
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise ValueError(f"{name}: Missing required columns: {missing}\nFound: {list(df.columns)}")


def _robust_tau_from_step(t: np.ndarray, w: np.ndarray, w0: float, wss: float) -> Optional[float]:
    """
    Robust tau estimate for 1st order:
      w(t) = wss - (wss-w0) exp(-t/tau)

    Use points where normalized response y is between 0.1 and 0.9:
      y = (w - w0)/(wss - w0) = 1 - exp(-t/tau)
      => tau = -t / ln(1 - y)
    Take median tau across selected points.
    """
    denom = (wss - w0)
    if abs(denom) < 1e-9:
        return None

    y = (w - w0) / denom
    mask = (y > 0.10) & (y < 0.90) & np.isfinite(y) & np.isfinite(t)
    if mask.sum() < 5:
        return None

    tt = t[mask]
    yy = y[mask]
    tau_vals = -tt / np.log(1.0 - yy)

    tau_vals = tau_vals[np.isfinite(tau_vals) & (tau_vals > 1e-4) & (tau_vals < 10.0)]
    if tau_vals.size < 3:
        return None

    return float(np.median(tau_vals))


def fit_dataset(
    csv_path: str,
    dataset_name: str,
    min_samples: int = 25,
    run_id_filter: int | None = None,
    test_name_filter: str | None = None,
    min_delta_w: float = 1e-3,
) -> Tuple[List[StepFit], pd.DataFrame]:
    df = pd.read_csv(csv_path)

    _require_cols(df, ["t_s", "phase_label", "u_sent", "w_deg_s"], dataset_name)

    # Numeric coercion
    df["t_s"] = pd.to_numeric(df["t_s"], errors="coerce")
    df["u_sent"] = pd.to_numeric(df["u_sent"], errors="coerce")
    df["w_deg_s"] = pd.to_numeric(df["w_deg_s"], errors="coerce")

    # --- NEW: filter to active test rows if available ---
    if "test_active" in df.columns:
        # could be "0/1" strings, ints, bools
        ta = pd.to_numeric(df["test_active"], errors="coerce")
        df = df[ta == 1].copy()

    # --- NEW: optionally filter by run_id / test_name ---
    if run_id_filter is not None and "test_run_id" in df.columns:
        rid = pd.to_numeric(df["test_run_id"], errors="coerce")
        df = df[rid == run_id_filter].copy()

    if test_name_filter is not None and "test_name" in df.columns:
        df = df[df["test_name"].astype(str).str.strip() == test_name_filter].copy()

    # Drop invalid rows
    df = df.dropna(subset=["t_s", "phase_label", "u_sent", "w_deg_s"]).copy()

    fits: List[StepFit] = []

    # --- NEW: only accept STEP_ phases (extra guard) ---
    df["phase_label"] = df["phase_label"].astype(str).str.strip()

    for phase, g in df.groupby("phase_label"):
        if not STEP_RE.match(phase):
            continue

        if len(g) < min_samples:
            continue

        g = g.sort_values("t_s")
        t = g["t_s"].to_numpy()
        u = g["u_sent"].to_numpy()
        w = g["w_deg_s"].to_numpy()

        tr = t - t[0]

        # robust command (skip weird steps)
        u_cmd = float(np.median(u))
        if abs(u_cmd) < 1e-6:
            continue

        n = len(w)
        n_win = max(5, int(0.15 * n))
        w0 = float(np.median(w[:n_win]))
        wss = float(np.median(w[-n_win:]))

        # --- NEW: use delta gain (fixes K=0 issues) ---
        dw = (wss - w0)
        if abs(dw) < min_delta_w:
            continue

        K = dw / u_cmd

        tau = _robust_tau_from_step(tr, w, w0=w0, wss=wss)
        if tau is None:
            continue

        fits.append(StepFit(
            dataset=dataset_name,
            phase=phase,
            u_cmd=u_cmd,
            w0=w0,
            wss=wss,
            K=float(K),
            tau=float(tau),
            n=n
        ))

    # Helpful debug if nothing fitted
    if len(fits) == 0:
        print(f"[DEBUG] {dataset_name}: 0 fits after filtering.")
        print(f"[DEBUG] {dataset_name}: rows remaining = {len(df)}")
        if len(df) > 0:
            print("[DEBUG] Example phase_labels:", df["phase_label"].unique()[:10])

    return fits, df



def summarize_fits(fits: List[StepFit]) -> Dict[str, float]:
    Ks = np.array([f.K for f in fits], dtype=float)
    taus = np.array([f.tau for f in fits], dtype=float)

    def med(x): return float(np.median(x)) if x.size else float("nan")
    def mad(x):
        if x.size == 0:
            return float("nan")
        m = np.median(x)
        return float(np.median(np.abs(x - m)))

    return {
        "K_median": med(Ks),
        "K_mad": mad(Ks),
        "tau_median_s": med(taus),
        "tau_mad_s": mad(taus),
        "n_steps": int(len(fits))
    }


def main():
    print("[INFO] Loading datasets...")
    print(f"  NO_LOAD:  {NO_LOAD_CSV}")
    print(f"  BEAM_LOAD:{BEAM_LOAD_CSV}")

    all_fits: List[StepFit] = []

    # nl_fits, _ = fit_dataset(NO_LOAD_CSV, "NO_LOAD", min_samples=MIN_SAMPLES)
    # bl_fits, _ = fit_dataset(BEAM_LOAD_CSV, "BEAM_LOAD", min_samples=MIN_SAMPLES)
    nl_fits, _ = fit_dataset(
        NO_LOAD_CSV, "NO_LOAD",
        min_samples=MIN_SAMPLES,
        run_id_filter=RUN_ID_FILTER,
        test_name_filter=TEST_NAME_FILTER,
        min_delta_w=MIN_DELTA_W
    )

    bl_fits, _ = fit_dataset(
        BEAM_LOAD_CSV, "BEAM_LOAD",
        min_samples=MIN_SAMPLES,
        run_id_filter=RUN_ID_FILTER,
        test_name_filter=TEST_NAME_FILTER,
        min_delta_w=MIN_DELTA_W
    )


    all_fits.extend(nl_fits)
    all_fits.extend(bl_fits)

    if len(nl_fits) == 0:
        print("[WARN] No steps fitted for NO_LOAD. Check phase_label and columns.")
    if len(bl_fits) == 0:
        print("[WARN] No steps fitted for BEAM_LOAD. Check phase_label and columns.")

    # Save per-step table
    rows = [{
        "dataset": f.dataset,
        "phase": f.phase,
        "u_cmd": f.u_cmd,
        "w0": f.w0,
        "wss": f.wss,
        "K_degps_per_u": f.K,
        "tau_s": f.tau,
        "n": f.n
    } for f in all_fits]

    per_step_df = pd.DataFrame(rows)
    per_step_csv = f"{OUT_PREFIX}_per_step.csv"
    per_step_df.to_csv(per_step_csv, index=False)

    # Summary stats
    summary = {
        "NO_LOAD": summarize_fits(nl_fits),
        "BEAM_LOAD": summarize_fits(bl_fits),
    }

    # Recommended model (prefer BEAM_LOAD)
    K_use = summary["BEAM_LOAD"]["K_median"] if np.isfinite(summary["BEAM_LOAD"]["K_median"]) else summary["NO_LOAD"]["K_median"]
    tau_use = summary["BEAM_LOAD"]["tau_median_s"] if np.isfinite(summary["BEAM_LOAD"]["tau_median_s"]) else summary["NO_LOAD"]["tau_median_s"]

    summary["SIM_MODEL_RECOMMENDED"] = {
        "form": "W(s)/U(s) = K / (tau*s + 1)",
        "K_degps_per_u": float(K_use),
        "tau_s": float(tau_use),
        "discrete_update": "W_next = W + (dt/tau) * (K*u - W)"
    }

    summary_json = f"{OUT_PREFIX}_summary.json"
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"[OK] Wrote: {per_step_csv}")
    print(f"[OK] Wrote: {summary_json}")
    print(json.dumps(summary, indent=2))

    if PLOT_RESULTS and not per_step_df.empty:
        for ds in ["NO_LOAD", "BEAM_LOAD"]:
            d = per_step_df[per_step_df["dataset"] == ds]
            if d.empty:
                continue

            plt.figure()
            plt.hist(d["K_degps_per_u"], bins=20)
            plt.xlabel("K (deg/s per command)")
            plt.ylabel("count")
            plt.title(f"{ds}: K distribution")

            plt.figure()
            plt.hist(d["tau_s"], bins=20)
            plt.xlabel("tau (s)")
            plt.ylabel("count")
            plt.title(f"{ds}: tau distribution")

        plt.show()


if __name__ == "__main__":
    main()
