import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime
from matplotlib.ticker import MaxNLocator  # Ensure this is imported
import numpy as np
import glob

# Configuration
BASE_DIR = "/home/giri/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection/results"
ALGORITHMS = [
    "Region_Growing_Segmentation",
    "seq_overlap",
    "kdtree_InflatingCircles",
    "sequentialApproachKdtree",
    "sequentialApproach"
]
PARAMETERS = ["radius", "slope", "density", "relief", "roughness"]
METRICS = [
    ("SuccessRate", "Success Rate (%)", "%"),
    ("AvgTime", "Average CPU Time (s)", "s"),
    ("AvgMemory", "Average Memory Usage (MB)", "MB")
]
SUBDIRS = ["combined", "individual", "comparison"]

# Area parameter (modify this for different areas)
AREA = "70mx70m"

# X-axis limits for each parameter
X_LIMITS = {
    "radius": (1.0, 4.0),
    "slope": (10.0, 40.0),
    "density": (10.0, 70.0),
    "relief": (0.10, 0.40),
    "roughness": (0.0, 0.04)
}

# Units for parameters
PARAM_UNITS = {
    "radius": "m",
    "slope": "deg",
    "density": "pts/m²",
    "relief": "m",
    "roughness": "m"
}

# Thresholds for ideal success rate
THRESHOLDS = {
    "radius": {"value": 2.5, "condition": ">=", "label": "min_radius_threshold"},
    "slope": {"value": 25.0, "condition": "<=", "label": "max_slope_threshold"},
    "density": {"value": 30.0, "condition": ">=", "label": "min_point_density_threshold"},
    "relief": {"value": 0.25, "condition": "<=", "label": "max_relief_threshold"},
    "roughness": {"value": 0.02, "condition": "<=", "label": "max_roughness_threshold"}
}

# Styling
COLORS = ["b", "r", "g", "m", "c"]
MARKERS = ["o", "s", "^", "D", "x"]
IDEAL_COLOR = "k"
IDEAL_LINESTYLE = "--"
FIGSIZE = (10, 6)
COMPARISON_FIGSIZE = (15, 5)
VARIANCE_FIGSIZE = (15, 12)
DPI = 300

# Find the latest results directory
def get_latest_results_dir():
    if not os.path.exists(BASE_DIR):
        raise FileNotFoundError(f"Base results directory '{BASE_DIR}' does not exist")
    result_dirs = sorted(glob.glob(os.path.join(BASE_DIR, "results_*")))
    if not result_dirs:
        raise FileNotFoundError(f"No results directories found in {BASE_DIR}")
    latest_dir = result_dirs[-1]
    if not os.path.isdir(latest_dir):
        raise NotADirectoryError(f"Latest results path '{latest_dir}' is not a directory")
    return latest_dir

# Create timestamped output directory with area
try:
    RESULTS_DIR = get_latest_results_dir()
except Exception as e:
    print(f"Error: {e}")
    exit(1)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_DIR = os.path.join(RESULTS_DIR, f"plots_{AREA}_{timestamp}")
try:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    for subdir in SUBDIRS:
        os.makedirs(os.path.join(OUTPUT_DIR, subdir), exist_ok=True)
    print(f"Created output directory: {OUTPUT_DIR}")
except Exception as e:
    print(f"Error creating output directory '{OUTPUT_DIR}': {e}")
    exit(1)

# Verify OUTPUT_DIR is a subdirectory of RESULTS_DIR
if not OUTPUT_DIR.startswith(RESULTS_DIR):
    print(f"Error: Output directory '{OUTPUT_DIR}' is not a subdirectory of '{RESULTS_DIR}'")
    exit(1)

# Verify and list files
def verify_build_files():
    if not os.path.exists(RESULTS_DIR):
        print(f"Error: Results directory '{RESULTS_DIR}' does not exist")
        return False
    
    csv_files = []
    py_files = []
    for algo in ALGORITHMS:
        for param in PARAMETERS:
            csv_file = os.path.join(RESULTS_DIR, f"results_{algo}_{param}.csv")
            py_file = os.path.join(RESULTS_DIR, f"plot_{algo}_{param}.py")
            if os.path.exists(csv_file):
                csv_files.append(csv_file)
            if os.path.exists(py_file):
                py_files.append(py_file)
    
    print(f"Found {len(csv_files)} CSV files in {RESULTS_DIR}:")
    for f in csv_files:
        print(f"  - {f}")
    print(f"Found {len(py_files)} Python script files in {RESULTS_DIR}:")
    for f in py_files:
        print(f"  - {f}")
    
    return len(csv_files) > 0

# Load CSV data
def load_csv(algo, param):
    filename = os.path.join(RESULTS_DIR, f"results_{algo}_{param}.csv")
    if not os.path.exists(filename):
        print(f"Error: {filename} not found")
        return None
    
    try:
        df = pd.read_csv(filename)
        expected_columns = ["Algorithm", "Parameter", "Value", "SuccessRate", "AvgTime", "AvgMemory", "NumSimulations"]
        if not all(col in df.columns for col in expected_columns):
            print(f"Error: {filename} missing expected columns. Found: {list(df.columns)}")
            return None
        if df.empty:
            print(f"Error: {filename} is empty")
            return None
        print(f"First 2 rows of {filename}:\n{df.head(2).to_string()}")
        return df
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return None

def create_combined_plot(metric, ylabel, unit):
    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(15, 8), sharey=True, 
                            gridspec_kw={'height_ratios': [1, 1], 'hspace': 0.25, 'wspace': 0.1})
    fig.suptitle(f"{ylabel} for All Algorithms Across Parameters ({AREA})", fontsize=16, y=0.98)
    
    axes = axes.flatten()  # Flatten for easier iteration
    for idx, param in enumerate(PARAMETERS):
        ax = axes[idx]
        ax.set_title(param.capitalize(), fontsize=12)
        ax.set_xlabel(f"{param} ({PARAM_UNITS[param]})", fontsize=10)
        ax.set_xlim(X_LIMITS[param])
        if idx % 3 == 0:  # Leftmost plots
            ax.set_ylabel(ylabel, fontsize=10)
        if metric == "SuccessRate":
            ax.set_ylim(0, 100)  # Success Rate in %
        ax.grid(True)
        ax.tick_params(axis='both', which='major', labelsize=8, pad=5)
        
        for algo, color, marker in zip(ALGORITHMS, COLORS, MARKERS):
            df = load_csv(algo, param)
            if df is not None:
                y_values = df[metric] * 100 if metric == "SuccessRate" else df[metric]
                ax.plot(df["Value"], y_values, color=color, marker=marker, 
                       label=algo.replace("_", " "))
        
        if metric == "SuccessRate":
            threshold = THRESHOLDS[param]
            x = np.linspace(X_LIMITS[param][0], X_LIMITS[param][1], 100)
            if threshold["condition"] == ">=":
                y = np.where(x >= threshold["value"], 100.0, 0.0)
            else:
                y = np.where(x <= threshold["value"], 100.0, 0.0)
            ax.plot(x, y, color='grey', linestyle='--', linewidth=2.5, 
                   label="Ideal Success Rate")
    
    # Hide unused axes (if PARAMETERS < 5)
    for idx in range(len(PARAMETERS), 6):
        axes[idx].set_visible(False)
    
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=len(ALGORITHMS) + 1, 
              bbox_to_anchor=(0.5, -0.02), fontsize=10)
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.95], pad=0.5)
    
    output_path = os.path.join(OUTPUT_DIR, "combined", f"combined_{metric.lower()}.png")
    plt.savefig(output_path, dpi=DPI, bbox_inches="tight")
    plt.close()
    print(f"Saved combined plot: {output_path}")
    
# Create individual plots
def create_individual_plots(metric, ylabel, unit):
    for algo in ALGORITHMS:
        for param in PARAMETERS:
            df = load_csv(algo, param)
            if df is None:
                continue
            
            plt.figure(figsize=FIGSIZE)
            y_values = df[metric] * 100 if metric == "SuccessRate" else df[metric]
            plt.plot(df["Value"], y_values, color=COLORS[ALGORITHMS.index(algo)],
                     marker=MARKERS[ALGORITHMS.index(algo)], label=algo.replace("_", " "))
            
            if metric == "SuccessRate":
                threshold = THRESHOLDS[param]
                x = np.linspace(X_LIMITS[param][0], X_LIMITS[param][1], 100)
                if threshold["condition"] == ">=":
                    y = np.where(x >= threshold["value"], 100.0, 0.0)
                else:
                    y = np.where(x <= threshold["value"], 100.0, 0.0)
                plt.plot(x, y, color=IDEAL_COLOR, linestyle=IDEAL_LINESTYLE, label="Ideal Success Rate")
            
            plt.xlabel(f"{param} ({PARAM_UNITS[param]})")
            plt.ylabel(ylabel)
            plt.xlim(X_LIMITS[param])
            if metric == "SuccessRate":
                plt.ylim(0, 100)
            plt.grid(True)
            plt.title(f"{ylabel} for {algo.replace('_', ' ')} vs {param} ({AREA})")
            plt.legend()
            
            output_path = os.path.join(OUTPUT_DIR, "individual",
                                     f"{metric.lower()}_{algo}_{param}.png")
            plt.savefig(output_path, dpi=DPI, bbox_inches="tight")
            plt.close()
            print(f"Saved individual plot: {output_path}")

# Create comparison bar graph (raw values)
def create_comparison_plot():
    errors = []
    means = {algo: {"SuccessRate": [], "AvgTime": [], "AvgMemory": []} for algo in ALGORITHMS}
    
    # Load data
    for algo in ALGORITHMS:
        for param in PARAMETERS:
            df = load_csv(algo, param)
            if df is None:
                errors.append(f"No data for {algo}, {param}")
                continue
            for metric, _, _ in METRICS:
                try:
                    values = pd.to_numeric(df[metric], errors='coerce').dropna().values
                    values = [float(v) for v in values if isinstance(v, (int, float, np.number))]
                    if len(values) == 0:
                        errors.append(f"No valid numeric data for {algo}, {param}, {metric}")
                    else:
                        print(f"Filtered data for {algo}, {param}, {metric}: {values[:4]}... (length: {len(values)})")
                        if metric == "SuccessRate":
                            values = [v * 100 for v in values]  # Convert to percentage
                        means[algo][metric].extend(values)
                except Exception as e:
                    errors.append(f"Error processing {algo}, {param}, {metric}: {e}")
    
    # Calculate mean values
    mean_values = {}
    for algo in ALGORITHMS:
        mean_values[algo] = {}
        for metric, _, _ in METRICS:
            values = means[algo][metric]
            values = [v for v in values if isinstance(v, (int, float, np.number))]
            print(f"Computing mean for {algo}, {metric}: {values[:5]}... (length: {len(values)})")
            if not values:
                errors.append(f"No valid values for {algo}, {metric}")
                mean_values[algo][metric] = 0.0
            else:
                mean_values[algo][metric] = np.mean(values)
    
    print("Mean values before plotting:")
    for algo in ALGORITHMS:
        print(f"{algo}: {mean_values[algo]}")
    
    # Check if any valid data exists
    if all(mean_values[algo][metric] == 0.0 for algo in ALGORITHMS for metric, _, _ in METRICS):
        print(f"Error: No valid data available for comparison plot")
        if errors:
            print("Issues encountered:\n" + "\n".join(errors))
        return
    
    # Create combined comparison plot (all metrics in one image)
    fig, axes = plt.subplots(nrows=1, ncols=len(METRICS), figsize=COMPARISON_FIGSIZE)
    fig.suptitle(f"Algorithm Comparison: Success Rate, CPU Time, and Memory Usage ({AREA})", fontsize=16)
    
    for idx, (metric, ylabel, unit) in enumerate(METRICS):
        ax = axes[idx] if len(METRICS) > 1 else axes
        x = np.arange(len(ALGORITHMS))
        width = 0.25
        
        values = [mean_values[algo][metric] for algo in ALGORITHMS]
        bars = ax.bar(x, values, width, color=COLORS, label=[algo.replace("_", " ") for algo in ALGORITHMS])
        
        # Add actual values as labels
        for bar in bars:
            height = bar.get_height()
            ax.text(
                bar.get_x() + bar.get_width() / 2, height + 0.02 * max(values, default=1),
                f"{height:.2f}",
                ha="center", va="bottom", fontsize=8
            )
        
        ax.set_xlabel("")
        ax.set_ylabel(ylabel)
        ax.set_xticks([])
        ax.set_xticklabels([])
        ax.grid(True, axis="y")
        ax.set_title(ylabel)
        
        # Adjust y-axis
        if metric == "SuccessRate":
            ax.set_ylim(0, 100)  # Success Rate in %
        else:
            max_height = max(values, default=1)
            ax.set_ylim(bottom=0, top=max_height * 1.1)
    
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=len(ALGORITHMS), bbox_to_anchor=(0.5, -0.05))
    plt.tight_layout(rect=[0, 0.1, 1, 0.95])
    
    output_path = os.path.join(OUTPUT_DIR, "comparison", "algorithm_comparison_all_metrics.png")
    plt.savefig(output_path, dpi=DPI, bbox_inches="tight")
    plt.close()
    print(f"Saved combined comparison plot: {output_path}")
    
    # Create separate comparison plots for each metric
    for metric, ylabel, unit in METRICS:
        fig, ax = plt.subplots(figsize=(8, 5))
        x = np.arange(len(ALGORITHMS))
        width = 0.25
        
        values = [mean_values[algo][metric] for algo in ALGORITHMS]
        bars = ax.bar(x, values, width, color=COLORS, label=[algo.replace("_", " ") for algo in ALGORITHMS])
        
        # Add actual values as labels
        for bar in bars:
            height = bar.get_height()
            ax.text(
                bar.get_x() + bar.get_width() / 2, height + 0.02 * max(values, default=1),
                f"{height:.2f}",
                ha="center", va="bottom", fontsize=8
            )
        
        ax.set_xlabel("")
        ax.set_ylabel(ylabel)
        ax.set_xticks([])
        ax.set_xticklabels([])
        ax.grid(True, axis="y")
        ax.set_title(f"{ylabel} Comparison ({AREA})")
        
        # Adjust y-axis
        if metric == "SuccessRate":
            ax.set_ylim(0, 100)  # Success Rate in %
        else:
            max_height = max(values, default=1)
            ax.set_ylim(bottom=0, top=max_height * 1.1)
        
        ax.legend()
        plt.tight_layout()
        
        output_path = os.path.join(OUTPUT_DIR, "comparison", f"algorithm_comparison_{metric.lower()}.png")
        plt.savefig(output_path, dpi=DPI, bbox_inches="tight")
        plt.close()
        print(f"Saved individual comparison plot: {output_path}")
    
    if errors:
        print("Issues encountered during comparison plot generation:\n" + "\n".join(errors))

# Create variance stats bar graph (raw values)
def create_variance_stats_plot():
    errors = []
    data = {algo: {"SuccessRate": [], "AvgTime": [], "AvgMemory": []} for algo in ALGORITHMS}
    
    # Load data
    for algo in ALGORITHMS:
        for param in PARAMETERS:
            df = load_csv(algo, param)
            if df is None:
                errors.append(f"No data for {algo}, {param}")
                continue
            for metric, _, _ in METRICS:
                try:
                    values = pd.to_numeric(df[metric], errors='coerce').dropna().values
                    values = [float(v) for v in values if isinstance(v, (int, float, np.number))]
                    if len(values) == 0:
                        errors.append(f"No valid numeric data for {algo}, {param}, {metric}")
                    else:
                        print(f"Filtered data for {algo}, {param}, {metric}: {values[:4]}... (length: {len(values)})")
                        if metric == "SuccessRate":
                            values = [v * 100 for v in values]  # Convert to percentage
                        data[algo][metric].extend(values)
                except Exception as e:
                    errors.append(f"Error processing {algo}, {param}, {metric}: {e}")
    
    # Calculate stats
    stats = {}
    for algo in ALGORITHMS:
        stats[algo] = {}
        for metric, _, _ in METRICS:
            values = data[algo][metric]
            values = [v for v in values if isinstance(v, (int, float, np.number))]
            print(f"Computing stats for {algo}, {metric}: {values[:5]}... (length: {len(values)})")
            if not values:
                errors.append(f"No valid values for {algo}, {metric}")
                stats[algo][metric] = {"mean": 0.0, "variance": 0.0, "std_dev": 0.0}
            else:
                stats[algo][metric] = {
                    "mean": np.mean(values),
                    "variance": np.var(values),
                    "std_dev": np.std(values)
                }
    
    print("Stats before plotting:")
    for algo in ALGORITHMS:
        print(f"{algo}: {stats[algo]}")
    
    # Check if any valid data exists
    if all(stats[algo][metric]["mean"] == 0.0 and
           stats[algo][metric]["variance"] == 0.0 and
           stats[algo][metric]["std_dev"] == 0.0
           for algo in ALGORITHMS for metric, _, _ in METRICS):
        print(f"Error: No valid data available for variance stats plot")
        if errors:
            print("Issues encountered:\n" + "\n".join(errors))
        return
    
    # Create subplots for each statistic (Mean, Variance, Std Dev)
    stat_types = ["Mean", "Variance", "Std Dev"]
    fig, axes = plt.subplots(nrows=len(stat_types), ncols=len(METRICS), 
                            figsize=VARIANCE_FIGSIZE)
    fig.suptitle(f"Statistical Outlook of the Montecarlo Simulation Results for Landing Zone Detection Algorithms ({AREA})", fontsize=16)
    
    # Add single legend below title
    handles = [plt.Rectangle((0,0),1,1, color=COLORS[i]) for i in range(len(ALGORITHMS))]
    fig.legend(handles, [algo.replace("_", " ") for algo in ALGORITHMS], 
              loc="upper center", ncol=len(ALGORITHMS), 
              bbox_to_anchor=(0.5, 0.92))
    
    for row_idx, stat_type in enumerate(stat_types):
        stat_key = stat_type.lower().replace(" ", "_")
        for col_idx, (metric, ylabel, unit) in enumerate(METRICS):
            ax = axes[row_idx, col_idx]
            
            # Prepare data for grouped bars
            x = np.arange(len(ALGORITHMS))
            width = 0.25
            values = [stats[algo][metric][stat_key] for algo in ALGORITHMS]
            
            # Plot bars
            bars = ax.bar(x, values, width, color=COLORS)
            
            # Add value labels above bars
            for bar in bars:
                height = bar.get_height()
                ax.text(
                    bar.get_x() + bar.get_width() / 2, 
                    height + 0.02 * max(values, default=1),
                    f"{height:.2f}",
                    ha="center", va="bottom", fontsize=8
                )
            
            # Set y-axis label
            if stat_key == "variance":
                ax.set_ylabel("Variance")
            else:
                ax.set_ylabel(f"{stat_type} of {ylabel}")
            
            # Remove x-axis ticks and labels
            ax.set_xticks([])
            ax.set_xticklabels([])
            
            ax.grid(True, axis="y")
            ax.set_title(f"{stat_type} of {ylabel.split(' (')[0]}")
            
            # Adjust y-axis
            if metric == "SuccessRate" and stat_key == "mean":
                ax.set_ylim(0, 100)  # Mean Success Rate in %
            else:
                max_height = max(values, default=1)
                ax.set_ylim(bottom=0, top=max_height * 1.2)  # 20% padding
    
    # Remove x-axis label from all subplots
    for ax in axes.flat:
        ax.set_xlabel("")
    
    plt.tight_layout(rect=[0, 0.05, 1, 0.90])
    output_path = os.path.join(OUTPUT_DIR, "comparison", f"algorithm_variance_stats.png")
    plt.savefig(output_path, dpi=DPI, bbox_inches="tight")
    plt.close()
    print(f"Saved variance stats plot: {output_path}")
    
    if errors:
        print("Issues encountered during variance stats plot generation:\n" + "\n".join(errors))

# Main execution
def main():
    print(f"Using results directory: {RESULTS_DIR}")
    print(f"Output plots will be saved to: {OUTPUT_DIR}")
    if not verify_build_files():
        print("Error: No CSV files found in results directory. Exiting.")
        return
    
    for metric, ylabel, unit in METRICS:
        create_combined_plot(metric, ylabel, unit)
    
    for metric, ylabel, unit in METRICS:
        create_individual_plots(metric, ylabel, unit)
    
    create_comparison_plot()
    create_variance_stats_plot()

if __name__ == "__main__":
    main()