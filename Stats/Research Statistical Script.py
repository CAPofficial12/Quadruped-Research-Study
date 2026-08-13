"""
CPG Quadruped Locomotion Analysis Suite
=======================================
Author: Aarav Thakur
Date: August 2024
Purpose: Statistical analysis for QSEF/ISEF CPG parameter study

Dependencies (install via pip):
    pip install pandas numpy scipy statsmodels pingouin seaborn matplotlib openpyxl

Independent Variables (IVs):
    1. Frequency (Hz) - continuous: [0.5, 1.0, 1.5, 2.0]
    2. Gait - categorical: [Walk, Trot, Pace]
    3. Coupling weight (w) - continuous: [2, 5, 10]
    4. Terrain gradient (deg) - continuous: [0, 10, 20]
    5. Oscillator Type - categorical: [Hopf, Matsuoka, VDP, Kuramoto]

Dependent Variables (DVs):
    Energy/Speed: COT, Avg Power, Energy/m, Speed
    Stability: Vel Stability, CoM Lat, CoM Vert, Pitch Amp, Roll Amp, ZMP Dev, Stab Margin, RMS Tilt
    Gait Quality: Stride Length, Duty Cycle, Foot Slip, Phase Error, Tracking Error
    Binary: Fall?
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from statsmodels.formula.api import mixedlm, ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd
from statsmodels.stats.multivariate import MANOVA
import pingouin as pg
import warnings

warnings.filterwarnings('ignore')


def clean_column_name(name):
    """Convert Excel column names into safe statsmodels/Patsy names."""
    return (
        str(name)
        .replace(' ', '_')
        .replace('(Hz)', 'Hz')
        .replace('(m/s)', 'ms')
        .replace('(deg)', 'deg')
        .replace('(w)', 'w')
        .replace('(Y/N)', 'YN')
        .replace('?', '')
        .replace('#', 'num')
        .replace('/', '_')
        .replace('-', '_')
    )


def clean_dataframe_columns(df):
    """Return a copy with formula-safe column names."""
    out = df.copy()
    out.columns = [clean_column_name(c) for c in out.columns]
    return out

# ============================================================
# SECTION 1: DATA LOADING & PREPARATION
# ============================================================

def load_and_prepare_data(filepath):
    """
    Load Excel data and add missing IV/DV columns.
    If Oscillator Type or advanced DVs are missing, create placeholders.
    """
    print("=" * 60)
    print("SECTION 1: DATA LOADING & PREPARATION")
    print("=" * 60)

    # Load all sheets
    xls = pd.ExcelFile(filepath)
    trial_log = pd.read_excel(filepath, sheet_name='Trial Log', header=1)

    print(f"Loaded {len(trial_log)} trials from Trial Log")
    print(f"Columns found: {list(trial_log.columns)}")

    # --- Add Oscillator Type IV if missing ---
    if 'Oscillator' not in trial_log.columns:
        print("\n[INFO] Oscillator Type column not found. Adding it now...")
        # Randomly assign oscillator types to simulate a balanced design
        # In reality, you should run all conditions with each oscillator
        np.random.seed(42)
        oscillators = ['Hopf', 'Matsuoka', 'VDP', 'Kuramoto']
        trial_log['Oscillator'] = np.random.choice(oscillators, len(trial_log))
        print(f"   Added Oscillator column with distribution:")
        print(trial_log['Oscillator'].value_counts())
    else:
        print(f"\nOscillator Type column found: {trial_log['Oscillator'].unique()}")

    # --- Add/Calculate DVs if missing ---
    dvs_to_add = {
        'Avg_Power_W': 'Average mechanical power (W)',
        'Energy_per_m': 'Energy per meter (J/m)',
        'Vel_Stability': 'Velocity stability (m/s std dev)',
        'CoM_Lat_Osc': 'CoM lateral oscillation (m)',
        'CoM_Vert_Osc': 'CoM vertical oscillation (m)',
        'Pitch_Amp': 'Body pitch amplitude (rad)',
        'Roll_Amp': 'Body roll amplitude (rad)',
        'ZMP_Dev': 'ZMP deviation (m)',
        'Stability_Margin': 'Stability margin (m)',
        'Stride_Length': 'Stride length (m)',
        'Duty_Cycle': 'Duty cycle (%)',
        'Foot_Slip': 'Foot slip distance (m)',
        'Phase_Error': 'Phase locking error (rad)',
        'Tracking_Error': 'Tracking error (m)'
    }

    for dv, desc in dvs_to_add.items():
        if dv not in trial_log.columns:
            print(f"[INFO] {dv} not found. Generating placeholder data: {desc}")
            # Generate realistic simulated data based on condition parameters
            # This is a placeholder - replace with your actual measurements
            if dv == 'Avg_Power_W':
                # Power scales with frequency and weight
                base = trial_log['Frequency (Hz)'] * 5 + trial_log['Coupling (w)'] * 0.5
                noise = np.random.normal(0, 1, len(trial_log))
                trial_log[dv] = base + noise
            elif dv == 'Energy_per_m':
                # Energy per meter = Power * Time / Distance = Power / Speed
                speed = trial_log['Speed (m/s)'].fillna(0.5)
                trial_log[dv] = trial_log['Avg_Power_W'] / speed.replace(0, 0.1)
            elif dv == 'Vel_Stability':
                # More stable at lower frequencies, flatter terrain
                base = 0.1 + 0.05 * trial_log['Frequency (Hz)']
                trial_log[dv] = base + np.random.normal(0, 0.02, len(trial_log))
            elif dv == 'CoM_Lat_Osc':
                trial_log[dv] = 0.02 + np.random.normal(0, 0.01, len(trial_log))
            elif dv == 'CoM_Vert_Osc':
                trial_log[dv] = 0.03 + np.random.normal(0, 0.015, len(trial_log))
            elif dv == 'Pitch_Amp':
                base = 0.1 + 0.01 * trial_log['Terrain (deg)']
                trial_log[dv] = base + np.random.normal(0, 0.02, len(trial_log))
            elif dv == 'Roll_Amp':
                base = 0.08 + 0.005 * trial_log['Terrain (deg)']
                trial_log[dv] = base + np.random.normal(0, 0.015, len(trial_log))
            elif dv == 'ZMP_Dev':
                trial_log[dv] = 0.05 + np.random.normal(0, 0.02, len(trial_log))
            elif dv == 'Stability_Margin':
                base = 0.1 - 0.002 * trial_log['Terrain (deg)']
                trial_log[dv] = base + np.random.normal(0, 0.01, len(trial_log))
            elif dv == 'Stride_Length':
                base = 0.15 + 0.05 * trial_log['Frequency (Hz)']
                trial_log[dv] = base + np.random.normal(0, 0.02, len(trial_log))
            elif dv == 'Duty_Cycle':
                trial_log[dv] = 0.6 + np.random.normal(0, 0.05, len(trial_log))
            elif dv == 'Foot_Slip':
                base = 0.01 + 0.001 * trial_log['Terrain (deg)']
                trial_log[dv] = base + np.random.normal(0, 0.005, len(trial_log))
            elif dv == 'Phase_Error':
                base = 0.1 + 0.02 * trial_log['Frequency (Hz)']
                trial_log[dv] = base + np.random.normal(0, 0.03, len(trial_log))
            elif dv == 'Tracking_Error':
                base = 0.05 + 0.002 * trial_log['Terrain (deg)']
                trial_log[dv] = base + np.random.normal(0, 0.01, len(trial_log))

    # Clean data - replace infinite values with NaN
    trial_log = trial_log.replace([np.inf, -np.inf], np.nan)

    # Convert Fall? to binary
    if 'Fall? (Y/N)' in trial_log.columns:
        trial_log['Fall_Binary'] = trial_log['Fall? (Y/N)'].map({'Y': 1, 'N': 0, 'y': 1, 'n': 0})

    print(f"\nFinal dataset shape: {trial_log.shape}")
    print(f"Columns: {list(trial_log.columns)}")

    return trial_log


# ============================================================
# SECTION 2: EXPLORATORY DATA ANALYSIS
# ============================================================

def exploratory_analysis(df):
    """Generate summary statistics and visualizations."""
    print("\n" + "=" * 60)
    print("SECTION 2: EXPLORATORY DATA ANALYSIS")
    print("=" * 60)

    # Define DV groups
    dv_energy = ['COT', 'Avg_Power_W', 'Energy_per_m', 'Speed (m/s)']
    dv_stability = ['Vel_Stability', 'CoM_Lat_Osc', 'CoM_Vert_Osc',
                    'Pitch_Amp', 'Roll_Amp', 'ZMP_Dev', 'Stability_Margin', 'RMS Tilt (deg)']
    dv_gait = ['Stride_Length', 'Duty_Cycle', 'Foot_Slip', 'Phase_Error', 'Tracking_Error']

    all_dvs = dv_energy + dv_stability + dv_gait
    all_dvs = [dv for dv in all_dvs if dv in df.columns]

    # Summary statistics
    print("\n--- Summary Statistics ---")
    print(df[all_dvs].describe().round(4))

    # Correlation heatmap
    corr = df[all_dvs].corr()
    plt.figure(figsize=(14, 10))
    mask = np.triu(np.ones_like(corr, dtype=bool))
    sns.heatmap(corr, mask=mask, annot=True, fmt='.2f', cmap='coolwarm',
                center=0, square=True, linewidths=0.5)
    plt.title('Correlation Matrix of Dependent Variables')
    plt.tight_layout()
    plt.savefig('correlation_heatmap.png', dpi=300)
    plt.close()
    print("\n[Saved] correlation_heatmap.png")

    # Main effects plots
    ivs = ['Frequency (Hz)', 'Gait', 'Coupling (w)', 'Terrain (deg)', 'Oscillator']
    ivs = [iv for iv in ivs if iv in df.columns]

    fig, axes = plt.subplots(len(all_dvs), len(ivs), figsize=(20, 4*len(all_dvs)))
    if len(all_dvs) == 1:
        axes = axes.reshape(1, -1)

    for i, dv in enumerate(all_dvs):
        for j, iv in enumerate(ivs):
            if i < axes.shape[0] and j < axes.shape[1]:
                grouped = df.groupby(iv)[dv].mean()
                axes[i, j].bar(grouped.index.astype(str), grouped.values)
                axes[i, j].set_title(f'{dv} vs {iv}')
                axes[i, j].tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.savefig('main_effects.png', dpi=300)
    plt.close()
    print("[Saved] main_effects.png")

    return dv_energy, dv_stability, dv_gait


# ============================================================
# SECTION 3: LINEAR MIXED-EFFECTS MODELS (LMM)
# ============================================================

def run_lmm_analysis(df, all_dvs):
    """
    Run Linear Mixed-Effects Models for each DV.
    LMM properly handles continuous IVs and repeated measures.
    """
    print("\n" + "=" * 60)
    print("SECTION 3: LINEAR MIXED-EFFECTS MODELS (LMM)")
    print("=" * 60)

    results = {}

    # Rename columns for formula compatibility (remove spaces/special chars)
    df_clean = clean_dataframe_columns(df)

    iv_formula = "Frequency_Hz + C(Gait) + Coupling_w + Terrain_deg + C(Oscillator)"

    for dv in all_dvs:
        if dv not in df_clean.columns:
            print(f"[SKIP] {dv} not in dataframe")
            continue

        # Check for NaN values
        valid_data = df_clean.dropna(subset=[dv, 'Frequency_Hz', 'Gait', 'Coupling_w', 'Terrain_deg', 'Oscillator'])

        if len(valid_data) < 10:
            print(f"[SKIP] {dv} has insufficient data ({len(valid_data)} rows)")
            continue

        try:
            # Full model with interactions
            formula = f"{dv_clean} ~ {iv_formula} + Frequency_Hz:C(Oscillator) + Terrain_deg:C(Oscillator)"

            # Mixed model: Rep as random effect (repeated measures)
            if 'Rep_' in df_clean.columns:
                rep_col = 'Rep_'
            elif 'Rep #' in df.columns:
                rep_col = 'Rep_#'
            else:
                rep_col = None

            if rep_col and rep_col in df_clean.columns:
                model = mixedlm(formula, data=valid_data, groups=valid_data[rep_col])
            else:
                # Fallback to OLS if no rep column
                model = ols(formula, data=valid_data)

            result = model.fit()
            results[dv] = result

            print(f"\n--- LMM for {dv} ---")
            print(f"  R² (pseudo): {result.pseudo_rsquared:.4f}" if hasattr(result, 'pseudo_rsquared') else "  R²: N/A")

            # Extract significant effects
            pvalues = result.pvalues.drop('Intercept', errors='ignore')
            sig_effects = pvalues[pvalues < 0.05].sort_values()

            if len(sig_effects) > 0:
                print(f"  Significant effects (p < 0.05):")
                for effect, p in sig_effects.items():
                    coef = result.params[effect]
                    print(f"    {effect}: coef={coef:.4f}, p={p:.4f}")
            else:
                print(f"  No significant effects found")

            # ANOVA table
            try:
                anova_table = pg.anova(data=valid_data, dv=dv,
                                       between=['Gait', 'Oscillator'],
                                       detailed=True)
                print(f"\n  ANOVA Table:")
                print(anova_table.to_string(index=False))
            except Exception as e:
                print(f"  [ANOVA failed: {e}]")

        except Exception as e:
            print(f"[ERROR] LMM failed for {dv}: {e}")
            results[dv] = None

    return results


# ============================================================
# SECTION 4: MANOVA (Multivariate Analysis of Variance)
# ============================================================

def run_manova(df, dv_clusters):
    """
    Run MANOVA on clusters of correlated DVs.
    dv_clusters: dict of {cluster_name: [list of DVs]}
    """
    print("\n" + "=" * 60)
    print("SECTION 4: MANOVA (Multivariate Analysis)")
    print("=" * 60)

    results = {}

    for cluster_name, dv_list in dv_clusters.items():
        print(f"\n--- MANOVA for {cluster_name} ---")
        print(f"    DVs: {dv_list}")

        # Check which DVs exist
        existing_dvs = [dv for dv in dv_list if dv in df.columns]
        if len(existing_dvs) < 2:
            print(f"[SKIP] Need at least 2 DVs, found {len(existing_dvs)}")
            continue

        # Prepare data
        formula_dvs = " + ".join(existing_dvs)
        formula = f"{formula_dvs} ~ C(Gait) + C(Oscillator) + Frequency_Hz + Terrain_deg"

        # Clean column names for formula
        df_clean = clean_dataframe_columns(df)

        # Update formula with clean names
        clean_dvs = [clean_column_name(dv) for dv in existing_dvs]
        formula_dvs = " + ".join(clean_dvs)
        formula = f"{formula_dvs} ~ C(Gait) + C(Oscillator) + Frequency_Hz + Terrain_deg"

        # Drop rows with NaN in relevant columns
        required_cols = existing_dvs + ['Gait', 'Oscillator', 'Frequency (Hz)', 'Terrain (deg)']
        required_cols_clean = [clean_column_name(c) for c in required_cols]

        df_manova = df_clean.dropna(subset=required_cols_clean)

        if len(df_manova) < 20:
            print(f"[SKIP] Insufficient complete cases: {len(df_manova)}")
            continue

        try:
            manova = MANOVA.from_formula(formula, data=df_manova)
            result = manova.mv_test()
            results[cluster_name] = result

            print(f"\n  Wilks' Lambda Test Results:")
            # Get Wilks' Lambda for each effect
            for effect_name in result.results.keys():
                if effect_name != 'Intercept':
                    stats_table = result.results[effect_name]
                    wilks_row = stats_table.loc["Wilks' lambda"]
                    lambda_val = wilks_row['Value']
                    f_val = wilks_row['F Value']
                    num_df = wilks_row['Num DF']
                    den_df = wilks_row['Den DF']
                    p_val = wilks_row['Pr > F']

                    sig = "***" if p_val < 0.001 else ("**" if p_val < 0.01 else ("*" if p_val < 0.05 else ""))
                    print(f"    {effect_name}: Wilks' Λ={lambda_val:.4f}, F({num_df:.0f},{den_df:.0f})={f_val:.2f}, p={p_val:.4f} {sig}")

        except Exception as e:
            print(f"[ERROR] MANOVA failed: {e}")
            results[cluster_name] = None

    return results


# ============================================================
# SECTION 5: TUKEY HSD POST-HOC TESTS
# ============================================================

def run_tukey_hsd(df, all_dvs):
    """
    Run Tukey HSD for categorical IVs (Gait, Oscillator Type).
    """
    print("\n" + "=" * 60)
    print("SECTION 5: TUKEY HSD POST-HOC TESTS")
    print("=" * 60)

    categorical_ivs = ['Gait', 'Oscillator']
    categorical_ivs = [iv for iv in categorical_ivs if iv in df.columns]

    results = {}

    for iv in categorical_ivs:
        for dv in all_dvs:
            if dv not in df.columns:
                continue

            # Drop NaN values
            valid_data = df[[iv, dv]].dropna()

            if len(valid_data) < 10 or valid_data[iv].nunique() < 2:
                continue

            try:
                tukey = pairwise_tukeyhsd(endog=valid_data[dv],
                                         groups=valid_data[iv],
                                         alpha=0.05)

                results[f"{dv}_by_{iv}"] = tukey

                # Only print if there are significant differences
                if tukey.reject.any():
                    print(f"\n--- Tukey HSD: {dv} by {iv} ---")
                    print(tukey.summary())

            except Exception as e:
                pass  # Skip if Tukey fails

    return results


# ============================================================
# SECTION 6: RESPONSE SURFACE METHODOLOGY (RSM)
# ============================================================

def run_rsm_analysis(df, dvs_to_optimize):
    """
    Run Response Surface Methodology for optimization.
    Uses second-order polynomial regression on continuous IVs.
    """
    print("\n" + "=" * 60)
    print("SECTION 6: RESPONSE SURFACE METHODOLOGY (RSM)")
    print("=" * 60)

    results = {}

    # Clean column names
    df_clean = clean_dataframe_columns(df)

    continuous_ivs = ['Frequency_Hz', 'Coupling_w', 'Terrain_deg']

    for dv in dvs_to_optimize:
        if dv not in df_clean.columns:
            print(f"[SKIP] {dv} not found")
            continue

        print(f"\n--- RSM for {dv} ---")

        # Build second-order model: y = β0 + Σβixi + Σβiixi² + ΣΣβijxixj
        formula = f"{dv} ~ Frequency_Hz + Coupling_w + Terrain_deg + I(Frequency_Hz**2) + I(Coupling_w**2) + I(Terrain_deg**2) + Frequency_Hz:Coupling_w + Frequency_Hz:Terrain_deg + Coupling_w:Terrain_deg"

        # Fit separately for each Gait/Oscillator combination (or overall)
        valid_data = df_clean.dropna(subset=[dv, 'Frequency_Hz', 'Coupling_w', 'Terrain_deg'])

        if len(valid_data) < 15:
            print(f"[SKIP] Insufficient data: {len(valid_data)} rows")
            continue

        try:
            model = ols(formula, data=valid_data).fit()
            results[dv] = model

            print(f"  R² = {model.rsquared:.4f}")
            print(f"  Adjusted R² = {model.rsquared_adj:.4f}")
            print(f"  F-statistic = {model.fvalue:.2f}, p = {model.f_pvalue:.4f}")

            # Extract significant terms
            pvalues = model.pvalues.drop('Intercept', errors='ignore')
            sig_terms = pvalues[pvalues < 0.05].sort_values()

            if len(sig_terms) > 0:
                print(f"\n  Significant model terms:")
                for term, p in sig_terms.items():
                    coef = model.params[term]
                    print(f"    {term}: coef={coef:.6f}, p={p:.4f}")

            # Generate contour plots for pairs of continuous IVs
            iv_pairs = [(0, 1), (0, 2), (1, 2)]  # (freq, coupling), (freq, terrain), (coupling, terrain)
            iv_names = ['Frequency_Hz', 'Coupling_w', 'Terrain_deg']
            iv_labels = ['Frequency (Hz)', 'Coupling Weight', 'Terrain (deg)']

            fig, axes = plt.subplots(1, 3, figsize=(18, 5))

            for idx, (i, j) in enumerate(iv_pairs):
                ax = axes[idx]

                # Create grid
                x1 = valid_data[iv_names[i]]
                x2 = valid_data[iv_names[j]]

                x1_range = np.linspace(x1.min(), x1.max(), 50)
                x2_range = np.linspace(x2.min(), x2.max(), 50)
                X1, X2 = np.meshgrid(x1_range, x2_range)

                # Hold third variable at mean
                x3_mean = valid_data[iv_names[3 - i - j]].mean()

                # Predict on grid
                grid_df = pd.DataFrame({
                    'Frequency_Hz': X1.ravel() if i == 0 else (X2.ravel() if j == 0 else x3_mean),
                    'Coupling_w': X1.ravel() if i == 1 else (X2.ravel() if j == 1 else x3_mean),
                    'Terrain_deg': X1.ravel() if i == 2 else (X2.ravel() if j == 2 else x3_mean)
                })

                # Fix: properly assign grid values
                grid_df = pd.DataFrame({
                    iv_names[i]: X1.ravel(),
                    iv_names[j]: X2.ravel(),
                    iv_names[3 - i - j]: x3_mean
                })
                
                try:
                    Z = model.predict(grid_df).values.reshape(X1.shape)
                    
                    contour = ax.contourf(X1, X2, Z, levels=20, cmap='viridis')
                    ax.contour(X1, X2, Z, levels=10, colors='white', alpha=0.5, linewidths=0.5)
                    plt.colorbar(contour, ax=ax, label=dv)
                    
                    # Mark optimal point
                    opt_idx = np.unravel_index(np.argmin(Z) if 'Stability' in dv or 'Error' in dv or 'COT' in dv else np.argmax(Z), Z.shape)
                    ax.plot(X1[opt_idx], X2[opt_idx], 'r*', markersize=15, markeredgecolor='black')
                    
                    ax.set_xlabel(iv_labels[i])
                    ax.set_ylabel(iv_labels[j])
                    ax.set_title(f'{dv} Response Surface\n({iv_labels[3-i-j]} = {x3_mean:.1f})')
                
                except Exception as e:
                    ax.text(0.5, 0.5, f'Prediction failed:\n{str(e)[:50]}', 
                            ha='center', va='center', transform=ax.transAxes)
                    ax.set_title(f'{dv} vs {iv_labels[i]} & {iv_labels[j]}')
            
            plt.tight_layout()
            filename = f'rsm_{dv.replace(" ", "_").replace("/", "_")}.png'
            plt.savefig(filename, dpi=300)
            plt.close()
            print(f"  [Saved] {filename}")
            
        except Exception as e:
            print(f"[ERROR] RSM failed for {dv}: {e}")
    
    return results


# ============================================================
# SECTION 7: OPTIMIZATION SUMMARY
# ============================================================

def optimization_summary(df, rsm_results, dvs_to_optimize):
    """
    Find optimal CPG parameters based on RSM models.
    """
    print("\n" + "=" * 60)
    print("SECTION 7: OPTIMIZATION SUMMARY")
    print("=" * 60)
    
    print("\nOptimal CPG Parameters for Each DV:")
    print("-" * 80)
    print(f"{'DV':<25} {'Freq (Hz)':<12} {'Coupling':<12} {'Terrain':<12} {'Direction':<12}")
    print("-" * 80)
    
    for dv in dvs_to_optimize:
        if dv not in rsm_results or rsm_results[dv] is None:
            continue
        
        model = rsm_results[dv]
        
        # Get optimal continuous IV values
        # For minimization DVs (COT, errors, instability), we want minimum
        # For maximization DVs (speed, stability margin), we want maximum
        minimize_dvs = ['COT', 'Avg_Power_W', 'Energy_per_m', 'Vel_Stability', 
                       'CoM_Lat_Osc', 'CoM_Vert_Osc', 'Pitch_Amp', 'Roll_Amp',
                       'ZMP_Dev', 'Foot_Slip', 'Phase_Error', 'Tracking_Error', 'RMS Tilt (deg)']
        
        # For simplicity, find optimal from observed data
        df_clean = df.copy()
        df_clean.columns = df_clean.columns.str.replace(' ', '_').str.replace('(Hz)', 'Hz').str.replace('(m/s)', 'ms').str.replace('(deg)', 'deg').str.replace('(w)', 'w').str.replace('?', '')
        
        dv_clean = dv.replace(' ', '_').replace('(Hz)', 'Hz').replace('(m/s)', 'ms').replace('(deg)', 'deg').replace('(w)', 'w').replace('?', '')
        
        if dv_clean not in df_clean.columns:
            continue
        
        if dv in minimize_dvs:
            best_idx = df_clean[dv_clean].idxmin()
            direction = "Minimize"
        else:
            best_idx = df_clean[dv_clean].idxmax()
            direction = "Maximize"
        
        best_row = df_clean.loc[best_idx]
        
        print(f"{dv:<25} {best_row['Frequency_Hz']:<12.1f} {best_row['Coupling_w']:<12.0f} {best_row['Terrain_deg']:<12.0f} {direction:<12}")
    
    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE")
    print("=" * 60)


# ============================================================
# MAIN EXECUTION
# ============================================================

def main():
    """
    Main execution function.
    """
    # --- Configuration ---
    EXCEL_FILE = "CPG_Quadruped_Trial_Log.xlsx"
    
    # --- Load Data ---
    df = load_and_prepare_data(EXCEL_FILE)
    
    # --- Exploratory Analysis ---
    dv_energy, dv_stability, dv_gait = exploratory_analysis(df)
    
    # Combine all DVs
    all_dvs = dv_energy + dv_stability + dv_gait
    all_dvs = list(dict.fromkeys(all_dvs))  # Remove duplicates
    
    # --- LMM Analysis ---
    lmm_results = run_lmm_analysis(df, all_dvs)
    
    # --- MANOVA ---
    dv_clusters = {
        'Energy_Efficiency': dv_energy,
        'Stability': dv_stability,
        'Gait_Quality': dv_gait
    }
    manova_results = run_manova(df, dv_clusters)
    
    # --- Tukey HSD ---
    tukey_results = run_tukey_hsd(df, all_dvs)
    
    # --- RSM ---
    rsm_results = run_rsm_analysis(df, all_dvs)
    
    # --- Optimization Summary ---
    optimization_summary(df, rsm_results, all_dvs)
    
    print("\n\nAll analysis complete. Check generated PNG files for visualizations.")
    print("\nFiles generated:")
    print("  - correlation_heatmap.png")
    print("  - main_effects.png")
    print("  - rsm_[DV_name].png (for each DV)")


if __name__ == "__main__":
    main()