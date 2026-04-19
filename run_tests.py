import os
import re
import random
import subprocess
import statistics
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

# Paths to the datasets
drowsy_dir = "dataset/drowsy"
alert_dir = "dataset/alert"

NUM_CASES = 5_000

# Makefile target/binary
SIM_BINARY = Path("obj_dir/Vdrowsiness_detection_top")
MAX_WORKERS = int(os.getenv("TEST_WORKERS", max(4, (os.cpu_count() or 4))))

# Sampling rate (kept from current behavior)
SAMPLE_PROB = 0.5

COMPLEXITY_RE = re.compile(r"COMPLEXITY:\s*compl_scaled=(\d+)")


def ensure_built():
    """Build the simulator once before running the test batch."""
    if SIM_BINARY.exists():
        return
    result = subprocess.run(["make", "-s", "all"], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to build simulator binary\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )


def extract_complexity(output: str):
    """Return the last reported complexity score from simulator output, or None."""
    matches = COMPLEXITY_RE.findall(output)
    if not matches:
        return None
    return int(matches[-1])


def run_case(image_path: str, expected_result: str):
    """Run one image and return (ok, complexity, image_path, err)."""
    try:
        result = subprocess.run([str(SIM_BINARY), image_path], capture_output=True, text=True)
        if result.returncode != 0:
            return False, None, image_path, result.stderr.strip()

        output = result.stdout
        complexity = extract_complexity(output)
        ok = expected_result in output
        return ok, complexity, image_path, None
    except Exception as e:
        return False, None, image_path, str(e)


def gather_cases():
    """Create a flat list of test cases for both categories."""
    cases = []

    count_drowsy = 0
    count_alert = 0

    for filename in os.listdir(drowsy_dir):
        file_path = os.path.join(drowsy_dir, filename)
        if os.path.isfile(file_path) and random.random() >= SAMPLE_PROB and count_drowsy < (NUM_CASES / 2):
            cases.append(("drowsy", file_path, "RESULT: DROWSY ", "drowsy_list.txt"))
            count_drowsy += 1

    for filename in os.listdir(alert_dir):
        file_path = os.path.join(alert_dir, filename)
        if os.path.isfile(file_path) and random.random() >= SAMPLE_PROB and count_alert < (NUM_CASES / 2):
            cases.append(("alert", file_path, "RESULT: ALERT", "alert_list.txt"))
            count_alert += 1

    return cases


def main():
    ensure_built()

    passed = 0
    failed = 0

    drowsy_complexity_sum = 0
    drowsy_complexity_count = 0
    alert_complexity_sum = 0
    alert_complexity_count = 0
    drowsy_scores = []
    alert_scores = []

    cases = gather_cases()
    if not cases:
        print("No sampled test cases selected.")
        return

    with ThreadPoolExecutor(max_workers=MAX_WORKERS) as pool:
        future_to_case = {
            pool.submit(run_case, path, expected): (category, path, expected, out_file)
            for category, path, expected, out_file in cases
        }

        for fut in as_completed(future_to_case):
            category, path, expected, out_file = future_to_case[fut]
            ok, complexity, image_path, err = fut.result()

            if ok:
                passed += 1
                print(f"Test passed for {image_path}, ratio: {passed / (passed + failed):.3f}")
                # If you want pass-lists enabled in threaded mode, uncomment below:
                # with open(out_file, "a") as f:
                #     f.write(image_path + "\n")
            else:
                failed += 1
                if err:
                    print(f"Test failed for {image_path} (run error): {err}")
                else:
                    print(f"Test failed for {image_path} (unexpected output)")

            if complexity is not None:
                if category == "drowsy":
                    drowsy_complexity_sum += complexity
                    drowsy_complexity_count += 1
                    drowsy_scores.append(complexity)
                else:
                    alert_complexity_sum += complexity
                    alert_complexity_count += 1
                    alert_scores.append(complexity)

    print("\nTest Summary:")
    print(f"Passed: {passed} ({passed / (passed + failed):.2%})")
    print(f"Failed: {failed}")

    if drowsy_complexity_count:
        drowsy_avg = drowsy_complexity_sum / drowsy_complexity_count
        drowsy_med = statistics.median(drowsy_scores)
        drowsy_std = statistics.pstdev(drowsy_scores) if drowsy_complexity_count > 1 else 0.0
        print(
            f"Average DROWSY complexity: {drowsy_avg:.2f} "
            f"({drowsy_complexity_count} samples)"
        )
        print(f"Median DROWSY complexity: {drowsy_med:.2f}")
        print("----------------------------------------")
        # print(f"StdDev DROWSY complexity: {drowsy_std:.2f}")
    else:
        print("Average DROWSY complexity: N/A (no samples)")

    if alert_complexity_count:
        alert_avg = alert_complexity_sum / alert_complexity_count
        alert_med = statistics.median(alert_scores)
        alert_std = statistics.pstdev(alert_scores) if alert_complexity_count > 1 else 0.0
        print(
            f"Average ALERT complexity: {alert_avg:.2f} "
            f"({alert_complexity_count} samples)"
        )
        print(f"Median ALERT complexity: {alert_med:.2f}")
        # print(f"StdDev ALERT complexity: {alert_std:.2f}")
        print("----------------------------------------")
    else:
        print("Average ALERT complexity: N/A (no samples)")

    if drowsy_scores or alert_scores:
        try:
            import importlib

            plt = importlib.import_module("matplotlib.pyplot")

            out_dir = Path("data_out")
            out_dir.mkdir(parents=True, exist_ok=True)
            fig_path = out_dir / "complexity_distribution.png"

            fig, axes = plt.subplots(1, 2, figsize=(12, 5))

            if drowsy_scores:
                axes[0].hist(drowsy_scores, bins=25, alpha=0.65, label="Drowsy", color="tab:red")
            if alert_scores:
                axes[0].hist(alert_scores, bins=25, alpha=0.65, label="Alert", color="tab:blue")
            axes[0].set_title("Complexity Histogram")
            axes[0].set_xlabel("Complexity")
            axes[0].set_ylabel("Count")
            axes[0].legend()

            box_data = []
            box_labels = []
            if drowsy_scores:
                box_data.append(drowsy_scores)
                box_labels.append("Drowsy")
            if alert_scores:
                box_data.append(alert_scores)
                box_labels.append("Alert")

            if box_data:
                axes[1].boxplot(box_data, tick_labels=box_labels, showmeans=True)
            axes[1].set_title("Complexity Boxplot")
            axes[1].set_ylabel("Complexity")

            fig.suptitle("Drowsiness Complexity Statistics")
            fig.tight_layout()
            fig.savefig(fig_path, dpi=160)
            plt.close(fig)
            print(f"Saved visualization: {fig_path}")
        except Exception as e:
            print(f"Visualization skipped: {e}")


if __name__ == "__main__":
    main()
