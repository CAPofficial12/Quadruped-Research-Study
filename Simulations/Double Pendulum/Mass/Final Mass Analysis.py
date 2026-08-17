import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

from scipy import stats
import statsmodels.api as sm
from sklearn.metrics import mean_squared_error

from pathlib import Path

script_dir = Path(__file__).resolve().parent
csv_fullpath = script_dir / "Double Pendulum Mass Summary Statistics.csv"
df = pd.read_csv(csv_fullpath)

IV = "weight_mass"

main_dvs = ["total_energy_relative_loss",
            "trajectory_length",
            "q1_dominant_frequency",
            "q2_dominant_frequency",
            "q1_decay_rate",
            "q2_decay_rate",
            "q1_velocity_max_abs",
            "q2_velocity_max_abs",
            "q1_acceleration_max_abs",
            "q2_acceleration_max_abs"]

primary_dvs = main_dvs


# ============================================================
# PLOTTING DATA STORAGE
# ============================================================
# One row = one point used to create one of the graphs.
# This is saved separately from the final statistical results.
plot_results = []


def plotter(x, labelX, y, labelY, i):
    plt.figure(i)
    plt.scatter(x, y)
    plt.xlabel(labelX)
    plt.ylabel(labelY)
    plt.title(f"{labelX} vs {labelY}")
    plt.show()


def add_plot_point(DV, plot, series, X, Y):
    """
    Add one point to the plotting-data CSV.
    """
    plot_results.append({
        "DV": DV,
        "Plot": plot,
        "Series": series,
        "X": X,
        "Y": Y
    })


def Regressions(x, y):
    # Use only positive masses for ALL three models so that
    # RMSE/AIC/BIC comparisons are made on the same observations.
    positive_mass = df[x] > 0

    regression_data = df.loc[
        positive_mass,
        [x, y]
    ].reset_index(drop=True)

    X = regression_data[x]
    Y = regression_data[y]

    # Linear regression
    X_linear = sm.add_constant(
        pd.DataFrame({x: X})
    )

    model = sm.OLS(
        Y,
        X_linear
    ).fit()

    # Quadratic regression
    X_squared = pd.DataFrame({
        x: X.values,
        "mass_squared": X.values ** 2
    })

    X_squared = sm.add_constant(
        X_squared
    )

    model_squared = sm.OLS(
        Y,
        X_squared
    ).fit()

    # Logarithmic regression
    X_log = pd.DataFrame({
        "mass_log": np.log(X.values)
    })

    X_log = sm.add_constant(
        X_log
    )

    model_log = sm.OLS(
        Y,
        X_log
    ).fit()

    return (
        model,
        model_squared,
        model_log,
        regression_data
    )


def RMSE_Predictionss(x, y):

    model, model_squared, model_log, regression_data = Regressions(
        x,
        y
    )

    Y = regression_data[y]
    observed_masses = regression_data[x]

    # 1000 points are used to create smooth regression curves.
    masses = np.linspace(
        df[x].min(),
        df[x].max(),
        1000
    )

    # ========================================================
    # LINEAR PREDICTIONS
    # ========================================================

    X_linear_plot = sm.add_constant(
        pd.DataFrame({x: masses})
    )

    linear_pred = model.predict(
        X_linear_plot
    )

    # ========================================================
    # QUADRATIC PREDICTIONS
    # ========================================================

    X_quad_plot = pd.DataFrame({
        x: masses,
        "mass_squared": masses ** 2
    })

    X_quad_plot = sm.add_constant(
        X_quad_plot
    )

    quad_pred = model_squared.predict(
        X_quad_plot
    )

    # ========================================================
    # LOGARITHMIC PREDICTIONS
    # ========================================================

    # Log model cannot use mass = 0.
    positive_masses = masses > 0

    X_log_plot = pd.DataFrame({
        "mass_log": np.log(
            masses[positive_masses]
        )
    })

    X_log_plot = sm.add_constant(
        X_log_plot
    )

    log_pred = model_log.predict(
        X_log_plot
    )

    # ========================================================
    # OBSERVED-MASS PREDICTIONS
    # ========================================================

    X_observed = sm.add_constant(
        pd.DataFrame({x: observed_masses})
    )

    linear_observed_pred = model.predict(
        X_observed
    )

    X_quad_observed = pd.DataFrame({
        x: observed_masses,
        "mass_squared": observed_masses ** 2
    })

    X_quad_observed = sm.add_constant(
        X_quad_observed
    )

    quad_observed_pred = model_squared.predict(
        X_quad_observed
    )

    X_log_observed = pd.DataFrame({
        "mass_log": np.log(observed_masses)
    })

    X_log_observed = sm.add_constant(
        X_log_observed
    )

    log_observed_pred = model_log.predict(
        X_log_observed
    )

    # ========================================================
    # RMSE
    # ========================================================

    # All three models use exactly the same positive-mass
    # observations for fair comparison.
    lin_rmse = np.sqrt(
        mean_squared_error(
            Y,
            linear_observed_pred
        )
    )

    quad_rmse = np.sqrt(
        mean_squared_error(
            Y,
            quad_observed_pred
        )
    )

    log_rmse = np.sqrt(
        mean_squared_error(
            Y,
            log_observed_pred
        )
    )

    print("Linear RMSE:", lin_rmse)
    print("Quadratic RMSE:", quad_rmse)
    print("Logarithmic RMSE:", log_rmse)

    # ========================================================
    # SAVE OBSERVED MASS vs DV POINTS
    # ========================================================

    for mass, dv_value in zip(
        df[x],
        df[y]
    ):
        add_plot_point(
            y,
            "Mass vs DV",
            "Observed",
            mass,
            dv_value
        )

    # ========================================================
    # SAVE LINEAR REGRESSION CURVE
    # ========================================================

    for mass, prediction in zip(
        masses,
        linear_pred
    ):
        add_plot_point(
            y,
            "Mass vs DV",
            "Linear Regression",
            mass,
            prediction
        )

    # ========================================================
    # SAVE QUADRATIC REGRESSION CURVE
    # ========================================================

    for mass, prediction in zip(
        masses,
        quad_pred
    ):
        add_plot_point(
            y,
            "Mass vs DV",
            "Quadratic Regression",
            mass,
            prediction
        )

    # ========================================================
    # SAVE LOGARITHMIC REGRESSION CURVE
    # ========================================================

    for mass, prediction in zip(
        masses[positive_masses],
        log_pred
    ):
        add_plot_point(
            y,
            "Mass vs DV",
            "Logarithmic Regression",
            mass,
            prediction
        )

    # ========================================================
    # PLOTS
    # ========================================================

    plotter(
        df[x],
        x,
        df[y],
        y,
        0
    )

    plt.figure(1)

    plt.scatter(
        df[x],
        df[y],
        label="Observed data"
    )

    plt.plot(
        masses,
        linear_pred,
        label="Linear regression",
        color="red"
    )

    plt.xlabel("Mass")
    plt.ylabel(y)
    plt.title(
        f"Linear Regression: Mass vs {y}"
    )
    plt.legend()
    plt.show()

    plt.figure(2)

    plt.scatter(
        df[x],
        df[y],
        label="Observed data"
    )

    plt.plot(
        masses,
        quad_pred,
        label="Quadratic regression",
        color="red"
    )

    plt.xlabel("Mass")
    plt.ylabel(y)
    plt.title(
        f"Quadratic Regression: Mass vs {y}"
    )
    plt.legend()
    plt.show()

    plt.figure(3)

    plt.scatter(
        df[x],
        df[y],
        label="Observed data"
    )

    plt.plot(
        masses[positive_masses],
        log_pred,
        label="Logarithmic regression",
        color="red"
    )

    plt.xlabel("Mass")
    plt.ylabel(y)
    plt.title(
        f"Logarithmic Regression: Mass vs {y}"
    )
    plt.legend()
    plt.show()

    return (
        lin_rmse,
        quad_rmse,
        log_rmse,
        linear_pred,
        quad_pred,
        log_pred
    )


def Residual_Analysis(x, y):

    model, model_squared, model_log, regression_data = Regressions(
        x,
        y
    )

    # --------------------------------------------------------
    # Linear residuals
    # --------------------------------------------------------

    for fitted, residual in zip(
        model.fittedvalues,
        model.resid
    ):
        add_plot_point(
            y,
            "Residual vs Fitted",
            "Linear Regression",
            fitted,
            residual
        )

    # --------------------------------------------------------
    # Quadratic residuals
    # --------------------------------------------------------

    for fitted, residual in zip(
        model_squared.fittedvalues,
        model_squared.resid
    ):
        add_plot_point(
            y,
            "Residual vs Fitted",
            "Quadratic Regression",
            fitted,
            residual
        )

    # --------------------------------------------------------
    # Logarithmic residuals
    # --------------------------------------------------------

    for fitted, residual in zip(
        model_log.fittedvalues,
        model_log.resid
    ):
        add_plot_point(
            y,
            "Residual vs Fitted",
            "Logarithmic Regression",
            fitted,
            residual
        )

    # Plot the linear residuals, preserving your original
    # residual-analysis graph.
    plt.scatter(
        model.fittedvalues,
        model.resid
    )

    plt.axhline(
        0,
        linestyle="--"
    )

    plt.xlabel("Fitted value")
    plt.ylabel("Residual")
    plt.title("Residuals vs Fitted")
    plt.show()


def Q_Q_plot(x, y):

    model, model_squared, model_log, regression_data = Regressions(
        x,
        y
    )

    # ========================================================
    # LINEAR Q-Q DATA
    # ========================================================

    qq_result_linear = stats.probplot(
        model.resid,
        dist="norm"
    )

    qq_theoretical_linear = qq_result_linear[0][0]
    qq_residual_linear = qq_result_linear[0][1]

    qq_slope_linear = qq_result_linear[1][0]
    qq_intercept_linear = qq_result_linear[1][1]

    qq_line_linear = (
        qq_intercept_linear
        + qq_slope_linear * qq_theoretical_linear
    )

    for x_value, y_value, line_value in zip(
        qq_theoretical_linear,
        qq_residual_linear,
        qq_line_linear
    ):
        add_plot_point(
            y,
            "Q-Q Plot",
            "Linear Regression",
            x_value,
            y_value
        )

        add_plot_point(
            y,
            "Q-Q Plot",
            "Linear Reference Line",
            x_value,
            line_value
        )

    # ========================================================
    # QUADRATIC Q-Q DATA
    # ========================================================

    qq_result_quad = stats.probplot(
        model_squared.resid,
        dist="norm"
    )

    qq_theoretical_quad = qq_result_quad[0][0]
    qq_residual_quad = qq_result_quad[0][1]

    qq_slope_quad = qq_result_quad[1][0]
    qq_intercept_quad = qq_result_quad[1][1]

    qq_line_quad = (
        qq_intercept_quad
        + qq_slope_quad * qq_theoretical_quad
    )

    for x_value, y_value, line_value in zip(
        qq_theoretical_quad,
        qq_residual_quad,
        qq_line_quad
    ):
        add_plot_point(
            y,
            "Q-Q Plot",
            "Quadratic Regression",
            x_value,
            y_value
        )

        add_plot_point(
            y,
            "Q-Q Plot",
            "Quadratic Reference Line",
            x_value,
            line_value
        )

    # ========================================================
    # LOGARITHMIC Q-Q DATA
    # ========================================================

    qq_result_log = stats.probplot(
        model_log.resid,
        dist="norm"
    )

    qq_theoretical_log = qq_result_log[0][0]
    qq_residual_log = qq_result_log[0][1]

    qq_slope_log = qq_result_log[1][0]
    qq_intercept_log = qq_result_log[1][1]

    qq_line_log = (
        qq_intercept_log
        + qq_slope_log * qq_theoretical_log
    )

    for x_value, y_value, line_value in zip(
        qq_theoretical_log,
        qq_residual_log,
        qq_line_log
    ):
        add_plot_point(
            y,
            "Q-Q Plot",
            "Logarithmic Regression",
            x_value,
            y_value
        )

        add_plot_point(
            y,
            "Q-Q Plot",
            "Logarithmic Reference Line",
            x_value,
            line_value
        )

    # --------------------------------------------------------
    # Display the linear Q-Q plot, preserving your original
    # graph.
    # --------------------------------------------------------

    sm.qqplot(
        model.resid,
        line="45",
        color="red"
    )

    plt.title("Q-Q Plot")
    plt.show()


def Turing_points(x, y):

    model_quad = Regressions(x, y)[1]

    b1 = model_quad.params[x]
    b2 = model_quad.params["mass_squared"]

    if np.isclose(b2, 0):
        return None

    turning_mass = -b1 / (2 * b2)

    return turning_mass


def Correlation_matrix():

    corr = df[primary_dvs].corr()

    sns.heatmap(
        corr,
        annot=True,
        cmap="coolwarm",
        center=0
    )

    plt.title("Correlation Matrix")
    plt.show()


def Effect_size(x, y):

    X = df[x]
    Y = df[y]

    # Find Y corresponding to minimum and maximum mass.
    min_index = X.idxmin()
    max_index = X.idxmax()

    Y_min = Y.loc[min_index]
    Y_max = Y.loc[max_index]

    Y_delta = (
        (Y_max - Y_min)
        / Y_min
        * 100
    )

    return Y_delta


# ============================================================
# CORRELATION MATRIX
# ============================================================

Correlation_matrix()


# ============================================================
# FINAL STATISTICAL RESULTS
# ============================================================

results = []


for i in primary_dvs:

    print()
    print(i)

    plotter(
        df[IV],
        IV,
        df[i],
        i,
        -1
    )

    # --------------------------------------------------------
    # Pearson and Spearman
    # --------------------------------------------------------

    pearson_result = stats.pearsonr(
        df[IV],
        df[i]
    )

    spearman_result = stats.spearmanr(
        df[IV],
        df[i]
    )

    r = pearson_result.statistic
    p1 = pearson_result.pvalue

    rho = spearman_result.statistic
    p2 = spearman_result.pvalue

    print("Pearson r:", r)
    print("Pearson p-value:", p1)

    print("Spearman rho:", rho)
    print("Spearman p-value:", p2)

    # --------------------------------------------------------
    # Regressions
    # --------------------------------------------------------

    (
        model,
        model_squared,
        model_log,
        regression_data
    ) = Regressions(
        IV,
        i
    )

    (
        lin_rmse,
        quad_rmse,
        log_rmse,
        linear_pred,
        quad_pred,
        log_pred
    ) = RMSE_Predictionss(
        IV,
        i
    )

    # --------------------------------------------------------
    # Linear regression
    # --------------------------------------------------------

    print()
    print("Linear regression:")

    print(
        "Intercept:",
        model.params["const"]
    )

    print(
        "Slope:",
        model.params[IV]
    )

    print(
        "Adjusted R²:",
        model.rsquared_adj
    )

    print(
        "AIC:",
        model.aic
    )

    print(
        "BIC:",
        model.bic
    )

    # --------------------------------------------------------
    # Quadratic regression
    # --------------------------------------------------------

    print()
    print("Quadratic regression:")

    print(
        "Intercept:",
        model_squared.params["const"]
    )

    print(
        "Linear coefficient:",
        model_squared.params[IV]
    )

    print(
        "Squared coefficient:",
        model_squared.params["mass_squared"]
    )

    print(
        "Adjusted R²:",
        model_squared.rsquared_adj
    )

    print(
        "AIC:",
        model_squared.aic
    )

    print(
        "BIC:",
        model_squared.bic
    )

    # --------------------------------------------------------
    # Logarithmic regression
    # --------------------------------------------------------

    print()
    print("Logarithmic regression:")

    print(
        "Intercept:",
        model_log.params["const"]
    )

    print(
        "Log coefficient:",
        model_log.params["mass_log"]
    )

    print(
        "Adjusted R²:",
        model_log.rsquared_adj
    )

    print(
        "AIC:",
        model_log.aic
    )

    print(
        "BIC:",
        model_log.bic
    )

    # --------------------------------------------------------
    # Residual analysis
    # --------------------------------------------------------

    Residual_Analysis(
        IV,
        i
    )

    # --------------------------------------------------------
    # Q-Q plot
    # --------------------------------------------------------

    Q_Q_plot(
        IV,
        i
    )

    # --------------------------------------------------------
    # Quadratic turning point
    # --------------------------------------------------------

    turning_mass = Turing_points(
        IV,
        i
    )

    if turning_mass is None:

        print(
            "No quadratic turning point because the quadratic "
            "coefficient is approximately zero."
        )

    else:

        print(
            "Quadratic turning mass:",
            turning_mass
        )

    # --------------------------------------------------------
    # Percentage change
    # --------------------------------------------------------

    percent_change = Effect_size(
        IV,
        i
    )

    print(
        "Percentage change from minimum to maximum mass:",
        percent_change,
        "%"
    )

    # --------------------------------------------------------
    # Best model according to RMSE
    # --------------------------------------------------------

    rmse_values = {
        "Linear": lin_rmse,
        "Quadratic": quad_rmse,
        "Logarithmic": log_rmse
    }

    rmse_low = min(
        rmse_values,
        key=rmse_values.get
    )

    print(
        f"Best Model: {rmse_low}"
    )

    # --------------------------------------------------------
    # Save one row for this DV
    # --------------------------------------------------------

    result = {

        "DV": i,

        "Number of tests":
            len(df),

        "Pearson r":
            r,

        "Pearson p-value":
            p1,

        "Spearman rho":
            rho,

        "Spearman p-value":
            p2,

        "Linear Intercept":
            model.params["const"],

        "Linear Slope":
            model.params[IV],

        "Linear RMSE":
            lin_rmse,

        "Linear Adjusted R²":
            model.rsquared_adj,

        "Linear AIC":
            model.aic,

        "Linear BIC":
            model.bic,

        "Quadratic Intercept":
            model_squared.params["const"],

        "Quadratic Linear Coefficient":
            model_squared.params[IV],

        "Quadratic Squared Coefficient":
            model_squared.params["mass_squared"],

        "Quadratic RMSE":
            quad_rmse,

        "Quadratic Adjusted R²":
            model_squared.rsquared_adj,

        "Quadratic AIC":
            model_squared.aic,

        "Quadratic BIC":
            model_squared.bic,

        "Quadratic Turning Mass":
            turning_mass,

        "Log Intercept":
            model_log.params["const"],

        "Log Coefficient":
            model_log.params["mass_log"],

        "Log RMSE":
            log_rmse,

        "Log Adjusted R²":
            model_log.rsquared_adj,

        "Log AIC":
            model_log.aic,

        "Log BIC":
            model_log.bic,

        "Percent change":
            percent_change,

        "Best Model":
            rmse_low
    }

    results.append(result)


# ============================================================
# SAVE FINAL STATISTICAL ANALYSIS CSV
# ============================================================

results_df = pd.DataFrame(
    results
)

analysis_csv_path = (
    script_dir /
    "Double Pendulum Final Analysis.csv"
)

results_df.to_csv(
    analysis_csv_path,
    index=False
)

print()
print(
    f"Final analysis saved to: {analysis_csv_path}"
)


# ============================================================
# SAVE PLOT DATA CSV
# ============================================================

plot_results_df = pd.DataFrame(
    plot_results
)

plot_csv_path = (
    script_dir /
    "Double Pendulum Mass Plot Data.csv"
)

plot_results_df.to_csv(
    plot_csv_path,
    index=False
)

print(
    f"Plot data saved to: {plot_csv_path}"
)