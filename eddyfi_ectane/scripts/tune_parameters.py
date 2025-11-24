#!/usr/bin/env python3
"""
Parameter tuning script for defect sizing methods.
Tests different parameter combinations to find optimal settings.
"""

import numpy as np
import pandas as pd
import json
import sys
import os

# Add parent directory to path to import from defect_sizing
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the data loading and measurement functions
from defect_sizing import (
    read_in, zero_data_series, transform_defect_location,
    measure_db_drop_boundary, measure_by_clustering, measure_by_contour
)

# True defect size
TRUE_DIAMETER_MM = 9.7

# Fixed dB threshold levels (clustering and contour will be tested at these levels)
DB_THRESHOLD_LEVELS = [6.0, 12.0]

# Parameter ranges to test
DBSCAN_EPS_RANGE = [0.001, 0.0015, 0.002, 0.0025, 0.003, 0.0035, 0.004]  # meters
DBSCAN_MIN_SAMPLES_RANGE = [3, 4, 5, 6, 7, 8]

CONTOUR_REGION_RANGE = [0.010, 0.012, 0.015, 0.018, 0.020]  # meters
CONTOUR_RESOLUTION_RANGE = [0.0001, 0.00015, 0.0002, 0.00025, 0.0003]  # meters

def load_data():
    """Load and preprocess the data."""
    path = "/home/matthew/Downloads/"
    ax_file = "1756920955004018_axial.pcd"
    trans_file = "1756920955004018_transversal.pcd"

    ax_data = read_in(path, ax_file)
    trans_data = read_in(path, trans_file)
    data = pd.concat([ax_data, trans_data], ignore_index=True, sort=True)

    # True defect locations in original coordinates
    d_a  = (-0.771 , -0.978)
    d_c  = ( 0.518 , -1.349)
    d_e1 = ( 0.889 , -1.339)  # Swapped with d_e2
    d_e2 = ( 0.866 , -1.338)  # Swapped with d_e1
    d_f  = (-0.801 , -1.316)

    # Store original min values for coordinate transformation
    x_min_original = data['x'].min()
    y_min_original = data['y'].min()

    data['x'] = zero_data_series(data['x'])
    data['y'] = zero_data_series(data['y'])
    data['z'] = zero_data_series(data['z'])

    # bodge
    tx = 0.03 # m
    trimmed = data[data['x'] > tx].reset_index(drop=True)
    del(data)
    data = trimmed

    # Store the offset after first zeroing
    x_offset_after_first_zero = data['x'].min()
    y_offset_after_first_zero = data['y'].min()

    data['x'] = zero_data_series(data['x'])
    data['y'] = zero_data_series(data['y'])
    data['z'] = zero_data_series(data['z'])

    # Calculate dB
    data['Magnitude'] = np.sqrt(data['Imaginary']**2 + data['Real']**2)
    data['dB'] = 20 * np.log10(np.clip(data['Magnitude'], 1e-12, None))

    # Transform defect locations
    def transform(original_x, original_y):
        x_final = original_x + abs(x_min_original) + abs(x_offset_after_first_zero)
        y_final = original_y + abs(y_min_original) + abs(y_offset_after_first_zero)
        return x_final, y_final

    true_defects = {
        'A': transform(d_a[0], d_a[1]),
        'C': transform(d_c[0], d_c[1]),
        'E1': transform(d_e1[0], d_e1[1]),
        'E2': transform(d_e2[0], d_e2[1]),
        'F': transform(d_f[0], d_f[1])
    }

    return data, true_defects


def test_dbscan_params(data, true_defects, eps_values, min_samples_values, threshold_db_levels=[6.0, 12.0]):
    """Test DBSCAN parameter combinations at different dB threshold levels."""
    print("\n" + "="*80)
    print("TUNING DBSCAN PARAMETERS")
    print(f"Testing at threshold levels: {threshold_db_levels} dB")
    print("="*80)

    results = []

    for eps in eps_values:
        for min_samples in min_samples_values:
            # Track errors for both threshold levels
            all_errors = []
            success_count = 0
            total_measurements = 0

            for defect_name, (true_x, true_y) in true_defects.items():
                # Find local maximum around specified location (same as main script)
                search_radius = 0.005  # 5mm
                local_region = data[
                    (np.abs(data['x'] - true_x) < search_radius) &
                    (np.abs(data['y'] - true_y) < search_radius)
                ]

                if len(local_region) > 0:
                    max_idx = local_region['dB'].idxmax()
                    peak_x = data.loc[max_idx, 'x']
                    peak_y = data.loc[max_idx, 'y']
                    peak_db = data.loc[max_idx, 'dB']
                else:
                    distances = np.sqrt((data['x'] - true_x)**2 + (data['y'] - true_y)**2)
                    nearest_idx = distances.idxmin()
                    peak_x = data.loc[nearest_idx, 'x']
                    peak_y = data.loc[nearest_idx, 'y']
                    peak_db = data.loc[nearest_idx, 'dB']

                # Test at both threshold levels
                for drop_db in threshold_db_levels:
                    total_measurements += 1
                    result = measure_by_clustering(
                        data, peak_x, peak_y,
                        threshold_db=peak_db - drop_db,
                        eps=eps,
                        min_samples=min_samples
                    )

                    if result and result.get('diameter_mm'):
                        diameter = result['diameter_mm']
                        error = abs(diameter - TRUE_DIAMETER_MM)
                        all_errors.append(error)
                        success_count += 1

            if len(all_errors) > 0:
                mean_error = np.mean(all_errors)
                std_error = np.std(all_errors)

                results.append({
                    'eps': eps,
                    'min_samples': min_samples,
                    'mean_error_mm': mean_error,
                    'std_error_mm': std_error,
                    'success_rate': success_count / total_measurements,
                    'score': mean_error + std_error  # Lower is better
                })

    # Sort by score
    results.sort(key=lambda x: x['score'])

    print("\nTop 10 DBSCAN parameter combinations:")
    print(f"{'Rank':<6} {'eps (mm)':<10} {'min_samp':<10} {'Mean Err':<12} {'Std Err':<12} {'Success':<10} {'Score':<10}")
    print("-" * 80)

    for i, r in enumerate(results[:10]):
        print(f"{i+1:<6} {r['eps']*1000:<10.2f} {r['min_samples']:<10} "
              f"{r['mean_error_mm']:<12.3f} {r['std_error_mm']:<12.3f} "
              f"{r['success_rate']:<10.1%} {r['score']:<10.3f}")

    return results


def test_contour_params(data, true_defects, region_values, resolution_values, drop_db_levels=[6.0, 12.0]):
    """Test Contour parameter combinations at fixed dB threshold levels."""
    print("\n" + "="*80)
    print("TUNING CONTOUR PARAMETERS")
    print(f"Testing at threshold levels: {drop_db_levels} dB")
    print("="*80)

    results = []

    total_combos = len(region_values) * len(resolution_values)
    print(f"Testing {total_combos} parameter combinations at {len(drop_db_levels)} dB levels...")

    combo_idx = 0
    for region_size in region_values:
        for resolution in resolution_values:
            combo_idx += 1
            if combo_idx % 10 == 0:
                print(f"  Progress: {combo_idx}/{total_combos} ({combo_idx/total_combos*100:.1f}%)")

            # Track errors for both threshold levels
            all_errors = []
            success_count = 0
            total_measurements = 0

            for defect_name, (true_x, true_y) in true_defects.items():
                # Find local maximum around specified location (same as main script)
                search_radius = 0.005  # 5mm
                local_region = data[
                    (np.abs(data['x'] - true_x) < search_radius) &
                    (np.abs(data['y'] - true_y) < search_radius)
                ]

                if len(local_region) > 0:
                    max_idx = local_region['dB'].idxmax()
                    peak_x = data.loc[max_idx, 'x']
                    peak_y = data.loc[max_idx, 'y']
                    peak_db = data.loc[max_idx, 'dB']
                else:
                    distances = np.sqrt((data['x'] - true_x)**2 + (data['y'] - true_y)**2)
                    nearest_idx = distances.idxmin()
                    peak_x = data.loc[nearest_idx, 'x']
                    peak_y = data.loc[nearest_idx, 'y']
                    peak_db = data.loc[nearest_idx, 'dB']

                # Test at both threshold levels
                for drop_db in drop_db_levels:
                    total_measurements += 1
                    try:
                        result = measure_by_contour(
                            data, peak_x, peak_y, peak_db,
                            drop_db=drop_db,
                            region_size=region_size,
                            resolution=resolution
                        )

                        if result and result.get('diameter_mm'):
                            diameter = result['diameter_mm']
                            error = abs(diameter - TRUE_DIAMETER_MM)
                            all_errors.append(error)
                            success_count += 1
                    except Exception as e:
                        # Skip this measurement if it fails
                        pass

            if len(all_errors) > 0:
                mean_error = np.mean(all_errors)
                std_error = np.std(all_errors)

                results.append({
                    'region_size': region_size,
                    'resolution': resolution,
                    'mean_error_mm': mean_error,
                    'std_error_mm': std_error,
                    'success_rate': success_count / total_measurements,
                    'score': mean_error + std_error  # Lower is better
                })

    # Sort by score
    results.sort(key=lambda x: x['score'])

    print("\nTop 10 Contour parameter combinations:")
    print(f"{'Rank':<6} {'Region':<10} {'Res(mm)':<10} {'Mean Err':<12} {'Std Err':<12} {'Success':<10} {'Score':<10}")
    print("-" * 90)

    for i, r in enumerate(results[:10]):
        print(f"{i+1:<6} {r['region_size']*1000:<10.1f} {r['resolution']*1000:<10.2f} "
              f"{r['mean_error_mm']:<12.3f} {r['std_error_mm']:<12.3f} "
              f"{r['success_rate']:<10.1%} {r['score']:<10.3f}")

    return results


def main():
    print("Loading data...")
    data, true_defects = load_data()
    print(f"Loaded {len(data)} data points")
    print(f"Testing on {len(true_defects)} defects")
    print(f"Fixed dB threshold levels: {DB_THRESHOLD_LEVELS}")

    # Test DBSCAN parameters at both -6dB and -12dB thresholds
    dbscan_results = test_dbscan_params(
        data, true_defects,
        DBSCAN_EPS_RANGE,
        DBSCAN_MIN_SAMPLES_RANGE,
        threshold_db_levels=DB_THRESHOLD_LEVELS
    )

    # Test Contour parameters at both -6dB and -12dB thresholds
    contour_results = test_contour_params(
        data, true_defects,
        CONTOUR_REGION_RANGE,
        CONTOUR_RESOLUTION_RANGE,
        drop_db_levels=DB_THRESHOLD_LEVELS
    )

    # Save results
    output_dir = "/home/matthew/Downloads/"

    with open(f"{output_dir}dbscan_tuning_results.json", 'w') as f:
        json.dump(dbscan_results, f, indent=2)
    print(f"\nDBSCAN results saved to: {output_dir}dbscan_tuning_results.json")

    with open(f"{output_dir}contour_tuning_results.json", 'w') as f:
        json.dump(contour_results, f, indent=2)
    print(f"Contour results saved to: {output_dir}contour_tuning_results.json")

    # Print recommendations
    print("\n" + "="*80)
    print("RECOMMENDED PARAMETERS")
    print(f"(Optimized for performance across both -6dB and -12dB thresholds)")
    print("="*80)

    best_dbscan = dbscan_results[0]
    print(f"\nDBSCAN:")
    print(f"  DBSCAN_EPS = {best_dbscan['eps']:.4f}  # {best_dbscan['eps']*1000:.2f} mm")
    print(f"  DBSCAN_MIN_SAMPLES = {best_dbscan['min_samples']}")
    print(f"  Mean error across both thresholds: {best_dbscan['mean_error_mm']:.3f} mm")
    print(f"  Standard deviation: {best_dbscan['std_error_mm']:.3f} mm")

    best_contour = contour_results[0]
    print(f"\nContour:")
    print(f"  CONTOUR_REGION_SIZE = {best_contour['region_size']:.4f}  # {best_contour['region_size']*1000:.1f} mm")
    print(f"  CONTOUR_RESOLUTION = {best_contour['resolution']:.4f}  # {best_contour['resolution']*1000:.2f} mm")
    print(f"  Mean error across both thresholds: {best_contour['mean_error_mm']:.3f} mm")
    print(f"  Standard deviation: {best_contour['std_error_mm']:.3f} mm")

    print("\n" + "="*80)


if __name__ == "__main__":
    main()
