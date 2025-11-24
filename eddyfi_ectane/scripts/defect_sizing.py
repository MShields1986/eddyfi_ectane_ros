#!/usr/bin/env python3

import numpy as np
import pandas as pd
import json

from matplotlib import cm, ticker
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from mpl_toolkits.axes_grid1 import make_axes_locatable

from scipy import interpolate, ndimage, signal, optimize, spatial
from sklearn.cluster import DBSCAN


# ##############################################################################
# CONFIGURATION
# ##############################################################################

# Enable/disable plots for faster iteration
ENABLE_PLOTS = True  # Set to False for faster parameter tuning

# DBSCAN parameters (optimized for -6dB and -12dB thresholds)
DBSCAN_THRESHOLD_DB = None  # If None, use max - 10 dB
DBSCAN_EPS = 0.002  # meters (2mm) - optimized
DBSCAN_MIN_SAMPLES = 6  # optimized - best across -6dB and -12dB (1.684mm error)

# Contour parameters (optimized for -6dB and -12dB thresholds)
CONTOUR_REGION_SIZE = 0.010  # meters (10mm) - optimized
CONTOUR_RESOLUTION = 0.0003  # meters (0.3mm) - optimized (1.599mm error)


# ##############################################################################

def read_in(path: str, file: str) -> pd.DataFrame:
    with open(path + file, "r") as pcd_file:
        lines = [line.strip().split(" ") for line in pcd_file.readlines()]

    is_data = False
    data = []
    for line in lines:
        if line[0] == 'FIELDS':
            cols = line
            cols.pop(0)
        elif line[0] == 'DATA':
            is_data = True
            continue
        elif is_data:
            data.append(line)

    del(line, lines, is_data, pcd_file)
    data = pd.DataFrame(data, columns = cols)
    data = data.astype(np.float32)

    return data


def zero_data_series(data_series: pd.core.series.Series) -> pd.core.series.Series:
    if data_series.min() < 0:
        data_series_out = data_series + data_series.min() * -1
    else:
        data_series_out = data_series + data_series.min()
    return data_series_out


# ##############################################################################
# SIZING METHOD 1: dB DROP
# ##############################################################################

def fit_ellipse_to_points(points: np.ndarray) -> dict:
    """
    Calculate metrics from boundary points using convex hull.

    Args:
        points: Nx2 array of (x, y) coordinates

    Returns:
        dict with: center, major_axis, minor_axis, angle, equivalent_diameter,
                   area, aspect_ratio
    """
    if len(points) < 3:
        return None

    # Center the points
    x = points[:, 0]
    y = points[:, 1]
    x_mean = np.mean(x)
    y_mean = np.mean(y)

    # Use convex hull to get actual boundary
    try:
        hull = spatial.ConvexHull(points)
        hull_points = points[hull.vertices]
        area = hull.volume  # In 2D, volume is area
        perimeter = hull.area  # In 2D, area is perimeter
    except:
        # If convex hull fails, estimate from bounding box
        x_range = np.ptp(points[:, 0])
        y_range = np.ptp(points[:, 1])
        area = x_range * y_range
        perimeter = 2 * (x_range + y_range)
        hull_points = points

    # Calculate equivalent circular diameter from area
    equivalent_diameter = np.sqrt(4 * area / np.pi)

    # Calculate major/minor axes from PCA for visualization
    cov = np.cov(hull_points[:, 0], hull_points[:, 1])
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    # Project points onto principal axes
    centered_points = hull_points - np.array([x_mean, y_mean])
    transformed_points = centered_points @ eigenvectors

    # Measure actual extent along principal axes
    major_axis = np.ptp(transformed_points[:, 0])
    minor_axis = np.ptp(transformed_points[:, 1])
    angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
    aspect_ratio = major_axis / minor_axis if minor_axis > 0 else 1.0

    return {
        'center': (x_mean, y_mean),
        'major_axis': major_axis,
        'minor_axis': minor_axis,
        'angle': angle,
        'angle_deg': np.degrees(angle),
        'equivalent_diameter': equivalent_diameter,
        'area': area,
        'aspect_ratio': aspect_ratio
    }


def measure_db_drop_boundary(data: pd.DataFrame, peak_x: float, peak_y: float,
                              peak_db: float, drop_db: float = 6.0,
                              region_size: float = 0.02,
                              resolution: float = 0.0003) -> dict:
    """
    Measure defect size using dB drop method with interpolation.

    Args:
        data: Full dataset
        peak_x, peak_y: Peak location
        peak_db: dB value at peak
        drop_db: dB drop threshold (6 or 12)
        region_size: Size of region to analyze around peak (meters)
        resolution: Grid resolution for interpolation (meters)

    Returns:
        dict with: diameter, area, major_axis, minor_axis, aspect_ratio,
                   boundary_points, ellipse_params
    """
    # Extract local region around peak
    local_data = data[
        (np.abs(data['x'] - peak_x) < region_size / 2) &
        (np.abs(data['y'] - peak_y) < region_size / 2)
    ].copy()

    if len(local_data) == 0:
        print(f"WARNING: No data points found in region around peak ({peak_x:.4f}, {peak_y:.4f})")
        return None

    # Create interpolated grid for complete angular coverage
    points = local_data[['x', 'y']].values
    values = local_data['dB'].values

    x_min, x_max = local_data['x'].min(), local_data['x'].max()
    y_min, y_max = local_data['y'].min(), local_data['y'].max()

    grid_x = np.arange(x_min, x_max + resolution, resolution)
    grid_y = np.arange(y_min, y_max + resolution, resolution)
    grid_x, grid_y = np.meshgrid(grid_x, grid_y)

    # Interpolate using linear method
    grid_z = interpolate.griddata(points, values, (grid_x, grid_y), method='linear')

    # Fill NaN values at edges with nearest neighbor
    mask = np.isnan(grid_z)
    if np.any(mask):
        grid_z[mask] = interpolate.griddata(points, values, (grid_x[mask], grid_y[mask]), method='nearest')

    # Calculate dB drop from peak
    target_db = peak_db - drop_db

    # Sample at regular angular intervals (36 angles = 10° sectors)
    n_bins = 36
    angles = np.linspace(0, 2 * np.pi, n_bins, endpoint=False)

    # Search radially outward from peak at each angle to find target dB
    uniform_points = []
    for angle in angles:
        # Create radial line from peak
        max_radius = region_size / 2
        radii = np.linspace(0, max_radius, 100)

        # Points along this radial line
        x_line = peak_x + radii * np.cos(angle)
        y_line = peak_y + radii * np.sin(angle)

        # Interpolate dB values along this line
        line_db = interpolate.griddata(points, values, (x_line, y_line), method='linear')

        # Find where we cross the target dB threshold
        if not np.all(np.isnan(line_db)):
            # Find the radius where dB is closest to target
            valid_mask = ~np.isnan(line_db)
            if np.any(valid_mask):
                valid_radii = radii[valid_mask]
                valid_db = line_db[valid_mask]

                # Find crossing point (where dB drops below target)
                if len(valid_db) > 0 and valid_db[0] >= target_db:
                    # Find where it crosses threshold
                    below_threshold = valid_db <= target_db
                    if np.any(below_threshold):
                        cross_idx = np.argmax(below_threshold)
                        boundary_radius = valid_radii[cross_idx]
                        boundary_x = peak_x + boundary_radius * np.cos(angle)
                        boundary_y = peak_y + boundary_radius * np.sin(angle)
                        uniform_points.append([boundary_x, boundary_y])

    if len(uniform_points) < 5:
        print(f"WARNING: Not enough boundary points at -{drop_db} dB (found {len(uniform_points)})")
        return None

    boundary_points_uniform = np.array(uniform_points)
    print(f"      Interpolated angular sampling: {len(boundary_points_uniform)} points ({len(boundary_points_uniform)/n_bins*100:.0f}% coverage)")

    # Fit ellipse to spatially uniform boundary points
    ellipse_params = fit_ellipse_to_points(boundary_points_uniform)

    if ellipse_params is None:
        return None

    # Convert to mm
    result = {
        'diameter_mm': ellipse_params['equivalent_diameter'] * 1000,
        'area_mm2': ellipse_params['area'] * 1e6,
        'major_axis_mm': ellipse_params['major_axis'] * 1000,
        'minor_axis_mm': ellipse_params['minor_axis'] * 1000,
        'aspect_ratio': ellipse_params['aspect_ratio'],
        'boundary_points': boundary_points_uniform,  # Interpolated uniform points for visualization
        'ellipse_center': ellipse_params['center'],
        'ellipse_angle_deg': ellipse_params['angle_deg'],
        'drop_db': drop_db,
        'num_boundary_points': len(boundary_points_uniform)
    }

    return result


# ##############################################################################
# SIZING METHOD 2: CLUSTERING
# ##############################################################################

def calculate_cluster_metrics(cluster_points: np.ndarray) -> dict:
    """
    Calculate size metrics for a cluster of points.

    Args:
        cluster_points: Nx2 array of (x, y) coordinates

    Returns:
        dict with: diameter, area, circularity, centroid, boundary_points
    """
    if len(cluster_points) < 3:
        return None

    # Calculate centroid
    centroid = np.mean(cluster_points, axis=0)

    # Compute convex hull
    try:
        hull = spatial.ConvexHull(cluster_points)
        hull_points = cluster_points[hull.vertices]
        area = hull.volume  # In 2D, volume is area
        perimeter = hull.area  # In 2D, area is perimeter
    except:
        # If convex hull fails, estimate from point cloud
        hull_points = cluster_points
        # Approximate area using min/max bounds
        x_range = np.ptp(cluster_points[:, 0])
        y_range = np.ptp(cluster_points[:, 1])
        area = x_range * y_range
        perimeter = 2 * (x_range + y_range)

    # Calculate equivalent circular diameter
    diameter = np.sqrt(4 * area / np.pi)

    # Calculate circularity (1.0 = perfect circle)
    circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0

    # Convert to mm
    result = {
        'diameter_mm': diameter * 1000,
        'area_mm2': area * 1e6,
        'circularity': circularity,
        'centroid': centroid,
        'boundary_points': hull_points,
        'num_points': len(cluster_points)
    }

    return result


def measure_by_clustering(data: pd.DataFrame, peak_x: float, peak_y: float,
                          threshold_db: float = None,
                          eps: float = DBSCAN_EPS,
                          min_samples: int = DBSCAN_MIN_SAMPLES,
                          region_size: float = 0.03) -> dict:
    """
    Detect and size defect using DBSCAN clustering in local region.

    Args:
        data: Full dataset
        peak_x, peak_y: Known defect location
        threshold_db: dB threshold for clustering (if None, use local max - 10 dB)
        eps: DBSCAN epsilon (max distance between neighbors in meters)
        min_samples: Minimum points for cluster
        region_size: Size of region to analyze around peak (meters)

    Returns:
        dict with measurement data for the defect
    """
    # Extract local region around known peak
    local_data = data[
        (np.abs(data['x'] - peak_x) < region_size / 2) &
        (np.abs(data['y'] - peak_y) < region_size / 2)
    ].copy()

    if len(local_data) < min_samples:
        print(f"WARNING: Not enough data points in clustering region")
        return None

    # Calculate threshold if not provided
    if threshold_db is None:
        threshold_db = local_data['dB'].max() - 10.0

    # Filter points above threshold
    mask = local_data['dB'] > threshold_db
    high_signal_data = local_data[mask].copy()

    if len(high_signal_data) < min_samples:
        print(f"WARNING: Not enough points above threshold {threshold_db:.1f} dB")
        return None

    # Extract coordinates
    coords = high_signal_data[['x', 'y']].values

    # Apply DBSCAN clustering
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coords)
    labels = clustering.labels_

    # Get unique clusters (excluding noise label -1)
    unique_labels = set(labels)
    if -1 in unique_labels:
        unique_labels.remove(-1)

    if len(unique_labels) == 0:
        print(f"WARNING: No clusters found with eps={eps}, min_samples={min_samples}")
        return None

    # Find cluster nearest to the peak location
    best_cluster = None
    min_distance = float('inf')

    for label in unique_labels:
        cluster_mask = labels == label
        cluster_points = coords[cluster_mask]
        metrics = calculate_cluster_metrics(cluster_points)

        if metrics is not None:
            # Distance from cluster centroid to known peak
            distance = np.sqrt((metrics['centroid'][0] - peak_x)**2 +
                             (metrics['centroid'][1] - peak_y)**2)
            if distance < min_distance:
                min_distance = distance
                best_cluster = metrics
                best_cluster['threshold_db'] = threshold_db
                best_cluster['eps'] = eps
                best_cluster['min_samples'] = min_samples

    return best_cluster


# ##############################################################################
# SIZING METHOD 3: CONTOUR ANALYSIS
# ##############################################################################

def extract_contour_at_level(contour_set, target_level: float) -> np.ndarray:
    """
    Extract contour vertices at specific level from matplotlib contour.

    Args:
        contour_set: Matplotlib contour object
        target_level: Target level to extract

    Returns:
        Nx2 array of contour points (or None if not found)
    """
    # Find the collection closest to target level
    for i, level in enumerate(contour_set.levels):
        if np.abs(level - target_level) < 0.1:
            # Get the paths for this level
            paths = contour_set.collections[i].get_paths()
            if len(paths) > 0:
                # Get the longest path (main contour)
                longest_path = max(paths, key=lambda p: len(p.vertices))
                return longest_path.vertices

    return None


def measure_by_contour(data: pd.DataFrame, peak_x: float, peak_y: float,
                       peak_db: float, drop_db: float = 6.0,
                       region_size: float = CONTOUR_REGION_SIZE,
                       resolution: float = CONTOUR_RESOLUTION) -> dict:
    """
    Measure defect size using contour analysis.

    Args:
        data: Full dataset
        peak_x, peak_y: Peak location
        peak_db: Peak dB value
        drop_db: dB drop for contour level
        region_size: Analysis region (meters)
        resolution: Grid resolution for interpolation (meters)

    Returns:
        dict with: diameter, area, perimeter, contour_points,
                   form_factor, enclosing_circle_radius
    """
    # Extract local region around peak
    local_data = data[
        (np.abs(data['x'] - peak_x) < region_size) &
        (np.abs(data['y'] - peak_y) < region_size)
    ].copy()

    if len(local_data) < 10:
        print(f"WARNING: Not enough data points in contour region around peak ({peak_x:.4f}, {peak_y:.4f})")
        return None

    # Create fine interpolated grid
    points = local_data[['x', 'y']].values
    values = local_data['dB'].values

    x_min, x_max = local_data['x'].min(), local_data['x'].max()
    y_min, y_max = local_data['y'].min(), local_data['y'].max()

    grid_x = np.arange(x_min, x_max + resolution, resolution)
    grid_y = np.arange(y_min, y_max + resolution, resolution)
    grid_x, grid_y = np.meshgrid(grid_x, grid_y)

    # Interpolate (linear to avoid artifacts)
    grid_z = interpolate.griddata(points, values, (grid_x, grid_y), method='linear')

    # Fill NaN
    mask = np.isnan(grid_z)
    if np.any(mask):
        grid_z[mask] = interpolate.griddata(points, values, (grid_x[mask], grid_y[mask]), method='nearest')

    # Generate contours
    target_level = peak_db - drop_db
    levels = [peak_db - 12, peak_db - 9, target_level, peak_db - 3]  # Must be increasing

    # Check if levels are within data range
    data_min, data_max = np.nanmin(grid_z), np.nanmax(grid_z)
    levels = [l for l in levels if data_min <= l <= data_max]

    # Sort and ensure unique levels
    levels = sorted(set(levels))

    if len(levels) < 2:
        print(f"WARNING: Not enough valid contour levels (data range: {data_min:.1f} to {data_max:.1f} dB)")
        return None

    # Create a figure to extract contours (we won't display it)
    fig, ax = plt.subplots()
    contour_set = ax.contour(grid_x, grid_y, grid_z, levels=levels)
    plt.close(fig)

    # Extract contour at target level
    contour_points = extract_contour_at_level(contour_set, target_level)

    if contour_points is None or len(contour_points) < 3:
        print(f"WARNING: Could not extract contour at -{drop_db} dB level")
        return None

    # Calculate area using shoelace formula
    x = contour_points[:, 0]
    y = contour_points[:, 1]
    area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    # Calculate perimeter
    perimeter = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    perimeter += np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2)  # Close the loop

    # Calculate equivalent diameter
    diameter = np.sqrt(4 * area / np.pi)

    # Calculate form factor (how circular)
    form_factor = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0

    # Fit minimum enclosing circle
    center = np.mean(contour_points, axis=0)
    radii = np.sqrt(np.sum((contour_points - center)**2, axis=1))
    enclosing_radius = np.max(radii)

    # Convert to mm
    result = {
        'diameter_mm': diameter * 1000,
        'area_mm2': area * 1e6,
        'perimeter_mm': perimeter * 1000,
        'form_factor': form_factor,
        'enclosing_circle_radius_mm': enclosing_radius * 1000,
        'boundary_points': contour_points,
        'drop_db': drop_db,
        'num_contour_points': len(contour_points)
    }

    return result


# ##############################################################################
# MEASUREMENT AGGREGATION
# ##############################################################################

def create_defect_measurement(defect_id: int, peak_location: tuple,
                              peak_db: float, data: pd.DataFrame,
                              true_diameter: float = 9.7) -> dict:
    """
    Run all three sizing methods on a single defect.

    Args:
        defect_id: Defect identifier (A, B, C, D, E1, E2, F)
        peak_location: (x, y) tuple of peak location
        peak_db: dB value at peak
        data: Full dataset
        true_diameter: True defect diameter in mm (default 9.7)

    Returns:
        Complete measurement dict with all metrics from all methods
    """
    peak_x, peak_y = peak_location

    print(f"  Measuring defect {defect_id} at ({peak_x:.4f}, {peak_y:.4f}) m, peak={peak_db:.1f} dB, true={true_diameter} mm")

    # Initialize result
    result = {
        'defect_id': defect_id,
        'peak_x': peak_x,
        'peak_y': peak_y,
        'peak_db': peak_db,
        'true_diameter_mm': true_diameter
    }

    # Method 1: dB Drop (-6 dB and -12 dB)
    print(f"    - Running -6 dB drop method...")
    db_6_result = measure_db_drop_boundary(data, peak_x, peak_y, peak_db, drop_db=6.0)
    if db_6_result:
        result['db_6_diameter_mm'] = db_6_result['diameter_mm']
        result['db_6_area_mm2'] = db_6_result['area_mm2']
        result['db_6_major_mm'] = db_6_result['major_axis_mm']
        result['db_6_minor_mm'] = db_6_result['minor_axis_mm']
        result['db_6_aspect_ratio'] = db_6_result['aspect_ratio']
        result['db_6_boundary'] = db_6_result['boundary_points']
        # Store ellipse parameters for proper visualization
        result['db_6_ellipse_center'] = db_6_result['ellipse_center']
        result['db_6_angle_deg'] = db_6_result['ellipse_angle_deg']
        print(f"      ✓ Diameter: {db_6_result['diameter_mm']:.2f} mm")
    else:
        result['db_6_diameter_mm'] = None
        print(f"      ✗ Failed")

    print(f"    - Running -12 dB drop method...")
    db_12_result = measure_db_drop_boundary(data, peak_x, peak_y, peak_db, drop_db=12.0)
    if db_12_result:
        result['db_12_diameter_mm'] = db_12_result['diameter_mm']
        result['db_12_area_mm2'] = db_12_result['area_mm2']
        result['db_12_boundary'] = db_12_result['boundary_points']
        print(f"      ✓ Diameter: {db_12_result['diameter_mm']:.2f} mm")
    else:
        result['db_12_diameter_mm'] = None
        print(f"      ✗ Failed")

    # Method 2: Clustering (at -6dB threshold)
    print(f"    - Running clustering -6dB method (eps={DBSCAN_EPS}, min_samples={DBSCAN_MIN_SAMPLES})...")
    cluster_6_result = measure_by_clustering(data, peak_x, peak_y, threshold_db=peak_db - 6.0)
    if cluster_6_result:
        result['cluster_6_diameter_mm'] = cluster_6_result['diameter_mm']
        result['cluster_6_area_mm2'] = cluster_6_result['area_mm2']
        result['cluster_6_circularity'] = cluster_6_result['circularity']
        result['cluster_6_boundary'] = cluster_6_result['boundary_points']
        print(f"      ✓ Diameter: {cluster_6_result['diameter_mm']:.2f} mm")
    else:
        result['cluster_6_diameter_mm'] = None
        print(f"      ✗ Failed")

    # Method 2b: Clustering (at -12dB threshold)
    print(f"    - Running clustering -12dB method (eps={DBSCAN_EPS}, min_samples={DBSCAN_MIN_SAMPLES})...")
    cluster_12_result = measure_by_clustering(data, peak_x, peak_y, threshold_db=peak_db - 12.0)
    if cluster_12_result:
        result['cluster_12_diameter_mm'] = cluster_12_result['diameter_mm']
        result['cluster_12_area_mm2'] = cluster_12_result['area_mm2']
        result['cluster_12_circularity'] = cluster_12_result['circularity']
        result['cluster_12_boundary'] = cluster_12_result['boundary_points']
        print(f"      ✓ Diameter: {cluster_12_result['diameter_mm']:.2f} mm")
    else:
        result['cluster_12_diameter_mm'] = None
        print(f"      ✗ Failed")

    # Method 3: Contour Analysis (-6dB)
    print(f"    - Running contour -6dB method (region={CONTOUR_REGION_SIZE*1000:.1f}mm, res={CONTOUR_RESOLUTION*1000:.2f}mm)...")
    contour_6_result = measure_by_contour(data, peak_x, peak_y, peak_db, drop_db=6.0)
    if contour_6_result:
        result['contour_6_diameter_mm'] = contour_6_result['diameter_mm']
        result['contour_6_area_mm2'] = contour_6_result['area_mm2']
        result['contour_6_perimeter_mm'] = contour_6_result['perimeter_mm']
        result['contour_6_form_factor'] = contour_6_result['form_factor']
        result['contour_6_boundary'] = contour_6_result['boundary_points']
        print(f"      ✓ Diameter: {contour_6_result['diameter_mm']:.2f} mm")
    else:
        result['contour_6_diameter_mm'] = None
        print(f"      ✗ Failed")

    # Method 3b: Contour Analysis (-12dB)
    print(f"    - Running contour -12dB method (region={CONTOUR_REGION_SIZE*1000:.1f}mm, res={CONTOUR_RESOLUTION*1000:.2f}mm)...")
    contour_12_result = measure_by_contour(data, peak_x, peak_y, peak_db, drop_db=12.0)
    if contour_12_result:
        result['contour_12_diameter_mm'] = contour_12_result['diameter_mm']
        result['contour_12_area_mm2'] = contour_12_result['area_mm2']
        result['contour_12_perimeter_mm'] = contour_12_result['perimeter_mm']
        result['contour_12_form_factor'] = contour_12_result['form_factor']
        result['contour_12_boundary'] = contour_12_result['boundary_points']
        print(f"      ✓ Diameter: {contour_12_result['diameter_mm']:.2f} mm")
    else:
        result['contour_12_diameter_mm'] = None
        print(f"      ✗ Failed")

    # Calculate statistics grouped by threshold (-6dB vs -12dB)
    diameters_6db = []
    if result.get('db_6_diameter_mm') is not None:
        diameters_6db.append(result['db_6_diameter_mm'])
    if result.get('cluster_6_diameter_mm') is not None:
        diameters_6db.append(result['cluster_6_diameter_mm'])
    if result.get('contour_6_diameter_mm') is not None:
        diameters_6db.append(result['contour_6_diameter_mm'])

    diameters_12db = []
    if result.get('db_12_diameter_mm') is not None:
        diameters_12db.append(result['db_12_diameter_mm'])
    if result.get('cluster_12_diameter_mm') is not None:
        diameters_12db.append(result['cluster_12_diameter_mm'])
    if result.get('contour_12_diameter_mm') is not None:
        diameters_12db.append(result['contour_12_diameter_mm'])

    # -6dB statistics
    if len(diameters_6db) > 0:
        result['mean_6db_diameter_mm'] = np.mean(diameters_6db)
        result['std_6db_diameter_mm'] = np.std(diameters_6db) if len(diameters_6db) > 1 else 0.0
        result['error_6db_vs_true_mm'] = result['mean_6db_diameter_mm'] - true_diameter
        print(f"    -6dB mean: {result['mean_6db_diameter_mm']:.2f} ± {result['std_6db_diameter_mm']:.2f} mm")
    else:
        result['mean_6db_diameter_mm'] = None
        result['std_6db_diameter_mm'] = None
        result['error_6db_vs_true_mm'] = None

    # -12dB statistics
    if len(diameters_12db) > 0:
        result['mean_12db_diameter_mm'] = np.mean(diameters_12db)
        result['std_12db_diameter_mm'] = np.std(diameters_12db) if len(diameters_12db) > 1 else 0.0
        result['error_12db_vs_true_mm'] = result['mean_12db_diameter_mm'] - true_diameter
        print(f"    -12dB mean: {result['mean_12db_diameter_mm']:.2f} ± {result['std_12db_diameter_mm']:.2f} mm")
    else:
        result['mean_12db_diameter_mm'] = None
        result['std_12db_diameter_mm'] = None
        result['error_12db_vs_true_mm'] = None

    print(f"    (True diameter: {true_diameter} mm)")

    return result


def compare_measurements(measurements: list) -> pd.DataFrame:
    """
    Create comparison table of all measurements.

    Args:
        measurements: List of measurement dicts from create_defect_measurement

    Returns:
        DataFrame with defects as rows, methods/metrics as columns
    """
    # Extract data for DataFrame
    rows = []
    for m in measurements:
        row = {
            'Defect ID': m['defect_id'],
            'Peak X (m)': f"{m['peak_x']:.4f}",
            'Peak Y (m)': f"{m['peak_y']:.4f}",
            'Peak (dB)': f"{m['peak_db']:.1f}",
            'Drop-6 (mm)': f"{m.get('db_6_diameter_mm', float('nan')):.2f}" if m.get('db_6_diameter_mm') else 'N/A',
            'Drop-12 (mm)': f"{m.get('db_12_diameter_mm', float('nan')):.2f}" if m.get('db_12_diameter_mm') else 'N/A',
            'Clust-6 (mm)': f"{m.get('cluster_6_diameter_mm', float('nan')):.2f}" if m.get('cluster_6_diameter_mm') else 'N/A',
            'Clust-12 (mm)': f"{m.get('cluster_12_diameter_mm', float('nan')):.2f}" if m.get('cluster_12_diameter_mm') else 'N/A',
            'Cont-6 (mm)': f"{m.get('contour_6_diameter_mm', float('nan')):.2f}" if m.get('contour_6_diameter_mm') else 'N/A',
            'Cont-12 (mm)': f"{m.get('contour_12_diameter_mm', float('nan')):.2f}" if m.get('contour_12_diameter_mm') else 'N/A',
            'Mean (mm)': f"{m.get('mean_diameter_mm', float('nan')):.2f}" if m.get('mean_diameter_mm') else 'N/A',
            'Std (mm)': f"{m.get('std_diameter_mm', float('nan')):.2f}" if m.get('std_diameter_mm') else 'N/A',
            'Error (mm)': f"{m.get('error_vs_true_mm', float('nan')):+.2f}" if m.get('error_vs_true_mm') else 'N/A'
        }
        rows.append(row)

    return pd.DataFrame(rows)


# ##############################################################################
# OUTPUT FUNCTIONS
# ##############################################################################

def save_measurements_csv(measurements: list, filepath: str):
    """
    Save measurements to CSV file.

    Args:
        measurements: List of measurement dicts
        filepath: Output CSV file path
    """
    rows = []
    for m in measurements:
        row = {
            'defect_id': m['defect_id'],
            'true_diameter_mm': m.get('true_diameter_mm', 9.7),
            'peak_x_m': m['peak_x'],
            'peak_y_m': m['peak_y'],
            'peak_db': m['peak_db'],
            'db_6_diameter_mm': m.get('db_6_diameter_mm'),
            'db_6_area_mm2': m.get('db_6_area_mm2'),
            'db_6_major_mm': m.get('db_6_major_mm'),
            'db_6_minor_mm': m.get('db_6_minor_mm'),
            'db_6_aspect_ratio': m.get('db_6_aspect_ratio'),
            'db_12_diameter_mm': m.get('db_12_diameter_mm'),
            'db_12_area_mm2': m.get('db_12_area_mm2'),
            'cluster_6_diameter_mm': m.get('cluster_6_diameter_mm'),
            'cluster_6_area_mm2': m.get('cluster_6_area_mm2'),
            'cluster_12_diameter_mm': m.get('cluster_12_diameter_mm'),
            'cluster_12_area_mm2': m.get('cluster_12_area_mm2'),
            'contour_6_diameter_mm': m.get('contour_6_diameter_mm'),
            'contour_6_area_mm2': m.get('contour_6_area_mm2'),
            'contour_12_diameter_mm': m.get('contour_12_diameter_mm'),
            'contour_12_area_mm2': m.get('contour_12_area_mm2'),
            'mean_6db_diameter_mm': m.get('mean_6db_diameter_mm'),
            'std_6db_diameter_mm': m.get('std_6db_diameter_mm'),
            'error_6db_vs_true_mm': m.get('error_6db_vs_true_mm'),
            'mean_12db_diameter_mm': m.get('mean_12db_diameter_mm'),
            'std_12db_diameter_mm': m.get('std_12db_diameter_mm'),
            'error_12db_vs_true_mm': m.get('error_12db_vs_true_mm')
        }
        rows.append(row)

    df = pd.DataFrame(rows)
    df.to_csv(filepath, index=False)
    print(f"Measurements saved to: {filepath}")


def convert_numpy_types(obj):
    """
    Recursively convert numpy types to Python native types for JSON serialization.
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.integer, np.int32, np.int64)):
        return int(obj)
    elif isinstance(obj, (np.floating, np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, dict):
        return {key: convert_numpy_types(value) for key, value in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [convert_numpy_types(item) for item in obj]
    else:
        return obj


def save_measurements_json(measurements: list, filepath: str):
    """
    Save measurements to JSON file with full detail.

    Args:
        measurements: List of measurement dicts
        filepath: Output JSON file path
    """
    # Convert all numpy types to Python native types
    output = convert_numpy_types(measurements)

    with open(filepath, 'w') as f:
        json.dump({
            'metadata': {
                'num_defects': len(measurements),
                'expected_size_mm': 9.7,
                'dbscan_eps': DBSCAN_EPS,
                'dbscan_min_samples': DBSCAN_MIN_SAMPLES,
                'contour_drop_db_values': [6.0, 12.0],
                'contour_region_size': CONTOUR_REGION_SIZE,
                'contour_resolution': CONTOUR_RESOLUTION
            },
            'measurements': output
        }, f, indent=2)

    print(f"Detailed measurements saved to: {filepath}")


def print_measurement_summary(measurements: list, true_diameter: float = None):
    """
    Print formatted summary to console.

    Args:
        measurements: List of measurement dicts
        true_diameter: Deprecated - true diameter is now stored per defect
    """
    print("\n" + "="*95)
    print("MEASUREMENT SUMMARY")
    print("="*95)

    # Print table header
    print(f"{'ID':<4} {'True':<6} {'Peak':<7} {'Drop-6':<7} {'Drop-12':<7} {'Clust-6':<8} {'Clust-12':<8} {'Cont-6':<7} {'Cont-12':<7} {'-6dB Mean±Std':<15} {'-12dB Mean±Std':<15}")
    print(f"{'':4} {'(mm)':<6} {'(dB)':<7} {'(mm)':<7} {'(mm)':<7} {'(mm)':<8} {'(mm)':<8} {'(mm)':<7} {'(mm)':<7} {'(mm)':<15} {'(mm)':<15}")
    print("-"*115)

    # Print each defect
    for m in measurements:
        defect_id = m['defect_id']
        true_diam = f"{m.get('true_diameter_mm', 9.7):.1f}"
        peak = f"{m['peak_db']:.1f}"
        db6 = f"{m.get('db_6_diameter_mm', float('nan')):.2f}" if m.get('db_6_diameter_mm') else "N/A"
        db12 = f"{m.get('db_12_diameter_mm', float('nan')):.2f}" if m.get('db_12_diameter_mm') else "N/A"
        clust6 = f"{m.get('cluster_6_diameter_mm', float('nan')):.2f}" if m.get('cluster_6_diameter_mm') else "N/A"
        clust12 = f"{m.get('cluster_12_diameter_mm', float('nan')):.2f}" if m.get('cluster_12_diameter_mm') else "N/A"
        cont6 = f"{m.get('contour_6_diameter_mm', float('nan')):.2f}" if m.get('contour_6_diameter_mm') else "N/A"
        cont12 = f"{m.get('contour_12_diameter_mm', float('nan')):.2f}" if m.get('contour_12_diameter_mm') else "N/A"

        if m.get('mean_6db_diameter_mm'):
            mean6_std = f"{m['mean_6db_diameter_mm']:.2f}±{m['std_6db_diameter_mm']:.2f}"
        else:
            mean6_std = "N/A"

        if m.get('mean_12db_diameter_mm'):
            mean12_std = f"{m['mean_12db_diameter_mm']:.2f}±{m['std_12db_diameter_mm']:.2f}"
        else:
            mean12_std = "N/A"

        print(f"{defect_id:<4} {true_diam:<6} {peak:<7} {db6:<7} {db12:<7} {clust6:<8} {clust12:<8} {cont6:<7} {cont12:<7} {mean6_std:<15} {mean12_std:<15}")

    # Print grouped statistics (-6dB and -12dB)
    valid_6db = [m for m in measurements if m.get('mean_6db_diameter_mm') is not None]
    valid_12db = [m for m in measurements if m.get('mean_12db_diameter_mm') is not None]

    print("-"*115)
    print(f"-6dB Methods Statistics ({len(valid_6db)}/{len(measurements)} defects):")
    if valid_6db:
        means_6db = [m['mean_6db_diameter_mm'] for m in valid_6db]
        errors_6db = [abs(m['error_6db_vs_true_mm']) for m in valid_6db]
        print(f"  Mean diameter: {np.mean(means_6db):.2f} mm")
        print(f"  Standard deviation: {np.std(means_6db):.2f} mm")
        print(f"  Mean absolute error: {np.mean(errors_6db):.2f} mm")

    print(f"\n-12dB Methods Statistics ({len(valid_12db)}/{len(measurements)} defects):")
    if valid_12db:
        means_12db = [m['mean_12db_diameter_mm'] for m in valid_12db]
        errors_12db = [abs(m['error_12db_vs_true_mm']) for m in valid_12db]
        print(f"  Mean diameter: {np.mean(means_12db):.2f} mm")
        print(f"  Standard deviation: {np.std(means_12db):.2f} mm")
        print(f"  Mean absolute error: {np.mean(errors_12db):.2f} mm")

    print("="*115)


# ##############################################################################
# VISUALIZATION FUNCTIONS
# ##############################################################################

def generate_ellipse_points(center, major_axis, minor_axis, angle_deg, n_points=100):
    """
    Generate points along an ellipse.

    Args:
        center: (x, y) tuple
        major_axis: Length of major axis
        minor_axis: Length of minor_axis
        angle_deg: Rotation angle in degrees
        n_points: Number of points to generate

    Returns:
        Array of (x, y) points
    """
    theta = np.linspace(0, 2 * np.pi, n_points)
    # Ellipse in standard position
    x = (major_axis / 2) * np.cos(theta)
    y = (minor_axis / 2) * np.sin(theta)

    # Rotate
    angle_rad = np.deg2rad(angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    x_rot = x * cos_a - y * sin_a
    y_rot = x * sin_a + y * cos_a

    # Translate to center
    x_final = x_rot + center[0]
    y_final = y_rot + center[1]

    return np.column_stack([x_final, y_final])


def plot_cscan_with_defects(data: pd.DataFrame, measurements: list,
                            D: float = 20, dpi: int = 300,
                            save_path: str = None, true_defects: dict = None):
    """
    Plot main C-scan with all defect boundaries overlaid.

    Args:
        data: Full dataset with x, y, dB columns
        measurements: List of measurement dicts
        D: Dynamic range in dB
        dpi: Resolution
        save_path: Output file path
        true_defects: Dict of true defect locations
    """
    if not ENABLE_PLOTS:
        return

    X = data['x']
    Y = data['y']
    Z = data['dB']

    vmin, vmax = Z.max() - D, Z.max()

    fig, ax = plt.subplots(dpi=dpi, figsize=(12, 4))

    # Base C-Scan
    im = ax.tripcolor(
        X, Y, Z,
        shading="gouraud",
        cmap="rainbow",
        vmin=vmin, vmax=vmax
    )

    # Overlay defect boundaries
    first_peak = True
    first_db6 = True
    first_db12 = True
    first_cluster6 = True
    first_cluster12 = True
    first_contour6 = True
    first_contour12 = True

    for m in measurements:
        # Peak marker
        label = 'Peak' if first_peak else ''
        ax.plot(m['peak_x'], m['peak_y'], '+', color='white',
                markersize=10, markeredgecolor='black', markeredgewidth=2.5, label=label)
        first_peak = False

        # -6 dB Drop boundary (white scatter) - plot actual boundary points
        if m.get('db_6_boundary') is not None:
            boundary = m['db_6_boundary']
            label = '-6 dB Drop' if first_db6 else ''
            ax.scatter(boundary[:, 0], boundary[:, 1], c='white', s=10,
                      alpha=0.8, edgecolors='black', linewidths=0.5, label=label)
            first_db6 = False

        # -12 dB Drop boundary (magenta scatter)
        if m.get('db_12_boundary') is not None:
            boundary = m['db_12_boundary']
            label = '-12 dB Drop' if first_db12 else ''
            ax.scatter(boundary[:, 0], boundary[:, 1], c='magenta', s=10,
                      alpha=0.8, edgecolors='black', linewidths=0.5, label=label)
            first_db12 = False

        # Cluster -6dB boundary (yellow solid)
        if m.get('cluster_6_boundary') is not None:
            boundary = m['cluster_6_boundary']
            label = 'Cluster -6dB' if first_cluster6 else ''
            ax.plot(boundary[:, 0], boundary[:, 1], 'y-', linewidth=1.5, label=label, alpha=0.8)
            first_cluster6 = False

        # Cluster -12dB boundary (orange dashed)
        if m.get('cluster_12_boundary') is not None:
            boundary = m['cluster_12_boundary']
            label = 'Cluster -12dB' if first_cluster12 else ''
            ax.plot(boundary[:, 0], boundary[:, 1], color='orange', linestyle='--',
                   linewidth=1.5, label=label, alpha=0.8)
            first_cluster12 = False

        # Contour -6dB boundary (cyan solid)
        if m.get('contour_6_boundary') is not None:
            boundary = m['contour_6_boundary']
            label = 'Contour -6dB' if first_contour6 else ''
            ax.plot(boundary[:, 0], boundary[:, 1], 'c-', linewidth=1.5, label=label, alpha=0.8)
            first_contour6 = False

        # Contour -12dB boundary (blue dotted)
        if m.get('contour_12_boundary') is not None:
            boundary = m['contour_12_boundary']
            label = 'Contour -12dB' if first_contour12 else ''
            ax.plot(boundary[:, 0], boundary[:, 1], 'b:', linewidth=1.5, label=label, alpha=0.8)
            first_contour12 = False

        # Defect label
        ax.text(m['peak_x'], m['peak_y'] + 0.008, f"D{m['defect_id']}",
                color='white', fontsize=10.5,
                ha='center', va='bottom',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.5))

    ax.set_title("C-Scan with Defect Sizing", fontsize=12)
    ax.set_xlabel("Scan Axis [m]")
    ax.set_ylabel("Array Axis [m]")
    ax.set_aspect('equal')
    ax.set_xlim(X.min(), X.max())
    ax.set_ylim(Y.min(), Y.max())

    # Colorbar
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    ticks = np.linspace(vmin, vmax, D//5 + 1)
    tick_labels = [f"{int(t - vmax)}" for t in ticks]
    cbar = plt.colorbar(im, cax=cax, ticks=ticks)
    cbar.ax.set_yticklabels(tick_labels)
    cbar.set_label("Relative Amplitude [dB]")
    cbar.ax.invert_yaxis()

    # Overlay true defect locations if provided
    if true_defects:
        for name, ((true_x, true_y), true_diameter) in true_defects.items():
            ax.plot(true_x, true_y, 'x', color='red', markersize=12,
                    markeredgewidth=3, label='True Location' if name == list(true_defects.keys())[0] else '')
            # Draw circle showing true diameter (radius = diameter/2 in meters)
            circle = plt.Circle((true_x, true_y), true_diameter / 2000.0, color='red',
                               fill=False, linewidth=1.5, linestyle='--', alpha=0.7)
            ax.add_patch(circle)

    # Legend
    ax.legend(loc='upper right', fontsize=9.5)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=dpi)
        print(f"C-Scan with defects saved to: {save_path}")
    else:
        plt.show()

    plt.close()


def plot_individual_defects_combined(data: pd.DataFrame, measurements: list,
                                     D: float = 20, dpi: int = 300,
                                     save_path: str = None):
    """
    Create a combined figure with zoomed plots for all defects.

    Args:
        data: Full dataset
        measurements: List of measurement dicts
        D: Dynamic range in dB
        dpi: Resolution
        save_path: Output file path for combined plot
    """
    if not ENABLE_PLOTS:
        return

    n_defects = len(measurements)

    # Create figure with subplots arranged in a grid
    # Use 3 columns, 3 rows
    n_cols = 3
    n_rows = 3

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, 15), dpi=dpi)

    # Flatten axes array for easier indexing
    if n_defects == 1:
        axes = np.array([axes])
    else:
        axes = axes.flatten()

    vmin, vmax = data['dB'].max() - D, data['dB'].max()

    for idx, m in enumerate(measurements):
        ax = axes[idx]

        # Extract local data
        zoom_size = 0.015  # ±12mm
        local_mask = (
            (np.abs(data['x'] - m['peak_x']) < zoom_size) &
            (np.abs(data['y'] - m['peak_y']) < zoom_size)
        )
        local_data = data[local_mask]

        if len(local_data) < 10:
            print(f"WARNING: Not enough data for defect {m['defect_id']} zoom plot")
            ax.text(0.5, 0.5, f"Defect {m['defect_id']}\nInsufficient Data",
                   ha='center', va='center', transform=ax.transAxes, fontsize=12)
            ax.axis('off')
            continue

        X = local_data['x']
        Y = local_data['y']
        Z = local_data['dB']

        # Base C-Scan
        im = ax.tripcolor(
            X, Y, Z,
            shading="gouraud",
            cmap="rainbow",
            vmin=vmin, vmax=vmax
        )

        # Overlay boundaries
        alphs = 0.7
        # Cluster -6dB
        if m.get('cluster_6_boundary') is not None:
            boundary = m['cluster_6_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='white', linestyle='-',
                   linewidth=2, label='-6 dB Cluster', alpha=alphs)

        # Contour -6dB
        if m.get('contour_6_boundary') is not None:
            boundary = m['contour_6_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='white', linestyle=':',
                   linewidth=2, label='-6 dB Contour', alpha=alphs)

        # -6 dB Drop
        if m.get('db_6_boundary') is not None:
            boundary = m['db_6_boundary']
            ax.scatter(boundary[:, 0], boundary[:, 1], c='white', s=25,
                      alpha=alphs, edgecolors='black', linewidths=0.5, label='-6 dB Drop')

        # Cluster -12dB
        if m.get('cluster_12_boundary') is not None:
            boundary = m['cluster_12_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='orange', linestyle='-',
                   linewidth=2, label='-12 dB Cluster', alpha=alphs)

        # Contour -12dB
        if m.get('contour_12_boundary') is not None:
            boundary = m['contour_12_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='orange', linestyle=':',
                   linewidth=2, label='-12 dB Contour', alpha=alphs)

        # -12 dB Drop
        if m.get('db_12_boundary') is not None:
            boundary = m['db_12_boundary']
            ax.scatter(boundary[:, 0], boundary[:, 1], c='orange', s=25,
                      alpha=alphs, edgecolors='black', linewidths=0.5, label='-12 dB Drop')

        # Peak marker
        ax.plot(m['peak_x'], m['peak_y'], '+', color='white',
                markersize=10, markeredgecolor='black', markeredgewidth=3, label='dB Peak')

        # Measurement text box
        textstr = f"Defect {m['defect_id']}\n"
        textstr += f"Peak: {m['peak_db']:.1f} dB\n\n"
        textstr += f"-6 dB\n"
        if m.get('db_6_diameter_mm'):
            textstr += f"Drop: {m['db_6_diameter_mm']:.2f} mm\n"
        if m.get('cluster_6_diameter_mm'):
            textstr += f"Cluster: {m['cluster_6_diameter_mm']:.2f} mm\n"
        if m.get('contour_6_diameter_mm'):
            textstr += f"Contour: {m['contour_6_diameter_mm']:.2f} mm\n"
        # if m.get('mean_6db_diameter_mm'):
        #     textstr += f"-6 dB Mean: {m['mean_6db_diameter_mm']:.2f} mm\n"
        # if m.get('std_6db_diameter_mm'):
        #     textstr += f"-6 dB STD: {m['std_6db_diameter_mm']:.2f} mm\n"
        # if m.get('mean_6db_diameter_mm'):
        #     textstr += f"-6dB Mean: {m['mean_6db_diameter_mm']:.2f}±{m['std_6db_diameter_mm']:.2f} mm\n"
        textstr += f"\n"
        textstr += f"-12 dB\n"
        if m.get('db_12_diameter_mm'):
            textstr += f"Drop: {m['db_12_diameter_mm']:.2f} mm\n"
        if m.get('cluster_12_diameter_mm'):
            textstr += f"Cluster: {m['cluster_12_diameter_mm']:.2f} mm\n"
        if m.get('contour_12_diameter_mm'):
            textstr += f"Contour: {m['contour_12_diameter_mm']:.2f} mm\n"
        # if m.get('mean_12db_diameter_mm'):
        #     textstr += f"-12 dB Mean: {m['mean_12db_diameter_mm']:.2f} mm\n"
        # if m.get('std_12db_diameter_mm'):
        #     textstr += f"-12 dB STD: {m['std_12db_diameter_mm']:.2f} mm\n"
        # if m.get('mean_12db_diameter_mm'):
            # textstr += f"-12dB Mean: {m['mean_12db_diameter_mm']:.2f}±{m['std_12db_diameter_mm']:.2f} mm\n"
        textstr += f"\nTrue: {m.get('true_diameter_mm', 9.7)} mm"

        ax.text(0.02, 0.98, textstr, transform=ax.transAxes,
                fontsize=10.5, verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))

        ax.set_title(f"Defect {m['defect_id']}")
        ax.set_xlabel("Scan Axis [m]")
        ax.set_ylabel("Array Axis [m]")
        ax.set_aspect('equal')
        ax.set_xlim(m['peak_x'] - zoom_size, m['peak_x'] + zoom_size)
        ax.set_ylim(m['peak_y'] - zoom_size, m['peak_y'] + zoom_size)

        # Colorbar
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.05)
        ticks = np.linspace(vmin, vmax, D//5 + 1)
        tick_labels = [f"{int(t - vmax)}" for t in ticks]
        cbar = plt.colorbar(im, cax=cax, ticks=ticks)
        cbar.ax.set_yticklabels(tick_labels)
        cbar.set_label("Relative Amplitude [dB]")
        # cbar.ax.tick_params(labelsize=8)
        cbar.ax.invert_yaxis()

        # Legend
        ax.legend(loc='lower left', fontsize=10.5, framealpha=0.7)

    # Add comparison bar chart to the last two cells
    if n_defects < len(axes) - 1:
        # Use the last two subplot cells for the comparison bar chart
        # Remove the axes and create a new subplot spanning both cells
        axes[-2].remove()
        axes[-1].remove()

        # Create a new axis spanning the last two cells
        # Calculate the position: last row, last two columns
        gs = axes[0].get_gridspec()
        ax_comparison = fig.add_subplot(gs[2, 1:])

        # Extract data for bar chart
        defect_ids = [m['defect_id'] for m in measurements]
        true_diams = [m.get('true_diameter_mm', 9.7) for m in measurements]
        db6_diams = [m.get('db_6_diameter_mm', np.nan) for m in measurements]
        db12_diams = [m.get('db_12_diameter_mm', np.nan) for m in measurements]
        cluster6_diams = [m.get('cluster_6_diameter_mm', np.nan) for m in measurements]
        cluster12_diams = [m.get('cluster_12_diameter_mm', np.nan) for m in measurements]
        contour6_diams = [m.get('contour_6_diameter_mm', np.nan) for m in measurements]
        contour12_diams = [m.get('contour_12_diameter_mm', np.nan) for m in measurements]

        # Prepare data for plotting
        x = np.arange(len(defect_ids))
        width = 0.13

        db6_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in db6_diams]
        db12_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in db12_diams]
        cluster6_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in cluster6_diams]
        cluster12_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in cluster12_diams]
        contour6_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in contour6_diams]
        contour12_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in contour12_diams]

        # Create bar chart
        ax_comparison.bar(x - 2.5*width, db6_plot, width, label='-6 dB Drop', color='steelblue', alpha=0.5)
        ax_comparison.bar(x - 1.5*width, db12_plot, width, label='-12 dB Drop', color='steelblue', alpha=0.9)
        ax_comparison.bar(x - 0.5*width, cluster6_plot, width, label='-6 dB Cluster', color='gold', alpha=0.5)
        ax_comparison.bar(x + 0.5*width, cluster12_plot, width, label='-12 dB Cluster', color='gold', alpha=0.9)
        ax_comparison.bar(x + 1.5*width, contour6_plot, width, label='-6 dB Contour', color='green', alpha=0.5)
        ax_comparison.bar(x + 2.5*width, contour12_plot, width, label='-12 dB Contour', color='green', alpha=0.9)

        # Plot true diameter lines
        for i, true_diam in enumerate(true_diams):
            label = 'True Size' if i == 0 else ''
            ax_comparison.plot([x[i] - 3*width, x[i] + 3*width], [true_diam, true_diam],
                    color='red', linestyle='--', linewidth=2, alpha=0.6, label=label)

        ax_comparison.set_xlabel('Defect ID')
        ax_comparison.set_ylabel('Diameter [mm]')
        ax_comparison.set_title('Defect Sizing Method Comparison')
        ax_comparison.set_xticks(x)
        ax_comparison.set_xticklabels([f'{i}' for i in defect_ids])
        ax_comparison.legend()
        ax_comparison.grid(axis='y', alpha=0.3)
        ax_comparison.set_ylim(4, 14)

    # Hide any other unused subplots (if fewer than 7 defects)
    for idx in range(n_defects, min(len(axes) - 2, len(axes))):
        axes[idx].axis('off')

    plt.tight_layout()
    # plt.subplots_adjust(hspace=-0.15)

    if save_path:
        plt.savefig(save_path, dpi=dpi, bbox_inches='tight')
        print(f"Combined defect plots saved to: {save_path}")
    else:
        plt.show()

    plt.close()


def plot_individual_defects(data: pd.DataFrame, measurements: list,
                            D: float = 20, dpi: int = 300,
                            save_dir: str = None):
    """
    Create zoomed plots for each defect.

    Args:
        data: Full dataset
        measurements: List of measurement dicts
        D: Dynamic range in dB
        dpi: Resolution
        save_dir: Output directory for individual plots
    """
    if not ENABLE_PLOTS:
        return

    for m in measurements:
        # Extract local data
        zoom_size = 0.012  # ±12mm
        local_mask = (
            (np.abs(data['x'] - m['peak_x']) < zoom_size) &
            (np.abs(data['y'] - m['peak_y']) < zoom_size)
        )
        local_data = data[local_mask]

        if len(local_data) < 10:
            print(f"WARNING: Not enough data for defect {m['defect_id']} zoom plot")
            continue

        X = local_data['x']
        Y = local_data['y']
        Z = local_data['dB']

        vmin, vmax = data['dB'].max() - D, data['dB'].max()

        fig, ax = plt.subplots(dpi=dpi, figsize=(7, 6))

        # Base C-Scan
        im = ax.tripcolor(
            X, Y, Z,
            shading="gouraud",
            cmap="rainbow",
            vmin=vmin, vmax=vmax
        )

        # Overlay boundaries
        alphs = 0.7
        # Cluster -6dB
        if m.get('cluster_6_boundary') is not None:
            boundary = m['cluster_6_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='white', linestyle='-',
                   linewidth=2, label='-6 dB Cluster', alpha=alphs)

        # Contour -6dB
        if m.get('contour_6_boundary') is not None:
            boundary = m['contour_6_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='white', linestyle=':',
                   linewidth=2, label='-6 dB Contour', alpha=alphs)

        # -6 dB Drop
        if m.get('db_6_boundary') is not None:
            boundary = m['db_6_boundary']
            ax.scatter(boundary[:, 0], boundary[:, 1], c='white', s=25,
                      alpha=alphs, edgecolors='black', linewidths=0.5, label='-6 dB Drop')

        # Cluster -12dB
        if m.get('cluster_12_boundary') is not None:
            boundary = m['cluster_12_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='orange', linestyle='-',
                   linewidth=2, label='-12 dB Cluster', alpha=alphs)

        # Contour -12dB
        if m.get('contour_12_boundary') is not None:
            boundary = m['contour_12_boundary']
            ax.plot(boundary[:, 0], boundary[:, 1], color='orange', linestyle=':',
                   linewidth=2, label='-12 dB Contour', alpha=alphs)

        # -12 dB Drop
        if m.get('db_12_boundary') is not None:
            boundary = m['db_12_boundary']
            ax.scatter(boundary[:, 0], boundary[:, 1], c='orange', s=25,
                      alpha=alphs, edgecolors='black', linewidths=0.5, label='-12 dB Drop')

        # Peak marker
        ax.plot(m['peak_x'], m['peak_y'], '+', color='white',
                markersize=10, markeredgecolor='black', markeredgewidth=3, label='dB Peak')

        # Measurement text box
        textstr = f"Defect {m['defect_id']}\n"
        textstr += f"Peak: {m['peak_db']:.1f} dB\n\n"
        if m.get('db_6_diameter_mm'):
            textstr += f"-6 dB Drop: {m['db_6_diameter_mm']:.2f} mm\n"
        if m.get('cluster_6_diameter_mm'):
            textstr += f"-6 dB Cluster: {m['cluster_6_diameter_mm']:.2f} mm\n"
        if m.get('contour_6_diameter_mm'):
            textstr += f"-6 dB Contour: {m['contour_6_diameter_mm']:.2f} mm\n"
        if m.get('mean_6db_diameter_mm'):
            textstr += f"-6 dB Mean: {m['mean_6db_diameter_mm']:.2f} mm\n"
        if m.get('std_6db_diameter_mm'):
            textstr += f"-6 dB STD: {m['std_6db_diameter_mm']:.2f} mm\n"
        # if m.get('mean_6db_diameter_mm'):
        #     textstr += f"-6dB Mean: {m['mean_6db_diameter_mm']:.2f}±{m['std_6db_diameter_mm']:.2f} mm\n"
        textstr += f"\n"
        if m.get('db_12_diameter_mm'):
            textstr += f"-12 dB Drop: {m['db_12_diameter_mm']:.2f} mm\n"
        if m.get('cluster_12_diameter_mm'):
            textstr += f"-12 dB Cluster: {m['cluster_12_diameter_mm']:.2f} mm\n"
        if m.get('contour_12_diameter_mm'):
            textstr += f"-12 dB Contour: {m['contour_12_diameter_mm']:.2f} mm\n"
        if m.get('mean_12db_diameter_mm'):
            textstr += f"-12dB Mean: {m['mean_12db_diameter_mm']:.2f}±{m['std_12db_diameter_mm']:.2f} mm\n"
        textstr += f"\nTrue: {m.get('true_diameter_mm', 9.7)} mm"

        ax.text(0.02, 0.98, textstr, transform=ax.transAxes,
                fontsize=8.5, verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9),
                family='monospace')

        ax.set_title(f"Defect {m['defect_id']} - Detailed View", fontsize=12)
        ax.set_xlabel("Scan Axis [m]")
        ax.set_ylabel("Array Axis [m]")
        ax.set_aspect('equal')
        ax.set_xlim(m['peak_x'] - zoom_size - 0.005, m['peak_x'] + zoom_size - 0.005)
        ax.set_ylim(m['peak_y'] - zoom_size, m['peak_y'] + zoom_size)

        # Colorbar
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.05)
        ticks = np.linspace(vmin, vmax, D//5 + 1)
        tick_labels = [f"{int(t - vmax)}" for t in ticks]
        cbar = plt.colorbar(im, cax=cax, ticks=ticks)
        cbar.ax.set_yticklabels(tick_labels)
        cbar.set_label("Relative Amplitude [dB]")
        cbar.ax.invert_yaxis()

        # Legend
        ax.legend(loc='lower left', fontsize=8.5)

        plt.tight_layout()

        if save_dir:
            save_path = f"{save_dir}/defect_{m['defect_id']}.png"
            plt.savefig(save_path, dpi=dpi)
            print(f"Defect {m['defect_id']} plot saved to: {save_path}")
        else:
            plt.show()

        plt.close()


def plot_method_comparison(measurements: list, true_diameter: float = None,
                           dpi: int = 300, save_path: str = None):
    """
    Create comparison plots showing all methods.

    Args:
        measurements: List of measurement dicts
        true_diameter: Deprecated - true diameter is now stored per defect
        dpi: Resolution
        save_path: Output file path
    """
    if not ENABLE_PLOTS:
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5), dpi=dpi)

    # Extract data
    defect_ids = [m['defect_id'] for m in measurements]
    true_diams = [m.get('true_diameter_mm', 9.7) for m in measurements]
    db6_diams = [m.get('db_6_diameter_mm', np.nan) for m in measurements]
    db12_diams = [m.get('db_12_diameter_mm', np.nan) for m in measurements]
    cluster6_diams = [m.get('cluster_6_diameter_mm', np.nan) for m in measurements]
    cluster12_diams = [m.get('cluster_12_diameter_mm', np.nan) for m in measurements]
    contour6_diams = [m.get('contour_6_diameter_mm', np.nan) for m in measurements]
    contour12_diams = [m.get('contour_12_diameter_mm', np.nan) for m in measurements]

    # Left panel: Bar chart
    x = np.arange(len(defect_ids))
    width = 0.13  # Narrower bars to fit 6 methods

    # Replace NaN/None with 0 for plotting (will show as no bar)
    db6_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in db6_diams]
    db12_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in db12_diams]
    cluster6_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in cluster6_diams]
    cluster12_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in cluster12_diams]
    contour6_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in contour6_diams]
    contour12_plot = [d if (d is not None and not np.isnan(d)) else 0 for d in contour12_diams]

    ax1.bar(x - 2.5*width, db6_plot, width, label='-6 dB Drop', color='steelblue', alpha=0.5)
    ax1.bar(x - 1.5*width, db12_plot, width, label='-12 dB Drop', color='steelblue', alpha=0.9)
    ax1.bar(x - 0.5*width, cluster6_plot, width, label='-6 dB Cluster', color='gold', alpha=0.5)
    ax1.bar(x + 0.5*width, cluster12_plot, width, label='-12 dB Cluster', color='gold', alpha=0.9)
    ax1.bar(x + 1.5*width, contour6_plot, width, label='-6 dB Contour', color='green', alpha=0.5)
    ax1.bar(x + 2.5*width, contour12_plot, width, label='-12 dB Contour', color='green', alpha=0.9)

    # Plot true diameter as horizontal line segments for each defect
    for i, true_diam in enumerate(true_diams):
        label = 'True Size' if i == 0 else ''
        ax1.plot([x[i] - 3*width, x[i] + 3*width], [true_diam, true_diam],
                color='red', linestyle='--', linewidth=2, alpha=0.6, label=label)

    ax1.set_xlabel('Defect ID')
    ax1.set_ylabel('Diameter [mm]')
    ax1.set_title('Defect Sizing Method Comparison')
    # ax1.set_title('Defect Sizing Method Comparison', fontsize=12)
    ax1.set_xticks(x)
    ax1.set_xticklabels([f'{i}' for i in defect_ids])
    ax1.legend()
    ax1.grid(axis='y', alpha=0.3)
    ax1.set_ylim(4, 14)  # Extended to accommodate both 6.4mm and 9.7mm defects

    # Right panel: Scatter plot with specific method comparisons
    # Build paired comparisons:
    # 1. -6dB Drop vs -6dB Cluster
    # 2. -6dB Drop vs -6dB Contour
    # 3. -6dB Drop vs -12dB Drop
    # 4. -12dB Drop vs -12dB Cluster
    # 5. -12dB Drop vs -12dB Contour

    pairs_6drop_6cluster = []
    pairs_6drop_6contour = []
    pairs_6drop_12drop = []
    pairs_12drop_12cluster = []
    pairs_12drop_12contour = []

    for m in measurements:
        db6_val = m.get('db_6_diameter_mm')
        db12_val = m.get('db_12_diameter_mm')
        c6_val = m.get('cluster_6_diameter_mm')
        c12_val = m.get('cluster_12_diameter_mm')
        co6_val = m.get('contour_6_diameter_mm')
        co12_val = m.get('contour_12_diameter_mm')

        # -6dB Drop vs -6dB Cluster
        if db6_val is not None and not np.isnan(db6_val) and c6_val is not None and not np.isnan(c6_val):
            pairs_6drop_6cluster.append((db6_val, c6_val))

        # -6dB Drop vs -6dB Contour
        if db6_val is not None and not np.isnan(db6_val) and co6_val is not None and not np.isnan(co6_val):
            pairs_6drop_6contour.append((db6_val, co6_val))

        # -6dB Drop vs -12dB Drop
        if db6_val is not None and not np.isnan(db6_val) and db12_val is not None and not np.isnan(db12_val):
            pairs_6drop_12drop.append((db6_val, db12_val))

        # -12dB Drop vs -12dB Cluster
        if db12_val is not None and not np.isnan(db12_val) and c12_val is not None and not np.isnan(c12_val):
            pairs_12drop_12cluster.append((db12_val, c12_val))

        # -12dB Drop vs -12dB Contour
        if db12_val is not None and not np.isnan(db12_val) and co12_val is not None and not np.isnan(co12_val):
            pairs_12drop_12contour.append((db12_val, co12_val))

    # Plot comparisons
    if pairs_6drop_12drop:
        x_vals, y_vals = zip(*pairs_6drop_12drop)
        ax2.scatter(x_vals, y_vals, label='-6 dB Drop vs -12 dB Drop',
                   s=100, alpha=0.7, color='steelblue', marker='s')

    if pairs_6drop_6cluster:
        x_vals, y_vals = zip(*pairs_6drop_6cluster)
        ax2.scatter(x_vals, y_vals, label='-6 dB Drop vs -6 dB Cluster',
                   s=100, alpha=0.5, color='gold', marker='o')

    if pairs_12drop_12cluster:
        x_vals, y_vals = zip(*pairs_12drop_12cluster)
        ax2.scatter(x_vals, y_vals, label='-12 dB Drop vs -12 dB Cluster',
                   s=100, alpha=0.9, color='gold', marker='^')

    if pairs_6drop_6contour:
        x_vals, y_vals = zip(*pairs_6drop_6contour)
        ax2.scatter(x_vals, y_vals, label='-6 dB Drop vs -6 dB Contour',
                   s=100, alpha=0.5, color='green', marker='o')

    if pairs_12drop_12contour:
        x_vals, y_vals = zip(*pairs_12drop_12contour)
        ax2.scatter(x_vals, y_vals, label='-12 dB Drop vs -12 dB Contour',
                   s=100, alpha=0.9, color='green', marker='^')

    plot_range = [4, 14]
    ax2.plot(plot_range, plot_range, 'k--', alpha=0.5, linewidth=2, label='Perfect Agreement')

    ax2.set_xlabel('-6/-12 dB Drop Method Diameter [mm]')
    ax2.set_ylabel('-12 dB Drop/Cluster/Contour Method Diameter [mm]')
    ax2.set_title('Sizing Method Agreement')
    # ax2.set_title('Sizing Method Agreement', fontsize=12)
    ax2.legend(fontsize=8, loc='upper left')
    ax2.grid(alpha=0.3)
    ax2.set_xlim(4, 14)
    ax2.set_ylim(4, 14)
    ax2.set_aspect('equal')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=dpi)
        print(f"Method comparison plot saved to: {save_path}")
    else:
        plt.show()

    plt.close()


# ##############################################################################
# Input
# ##############################################################################

path = "/home/matthew/Downloads/"

ax_file = "1756920955004018_axial.pcd"
trans_file = "1756920955004018_transversal.pcd"

# path = "/home/matthew/Downloads/ec_old/"

# ax_file = "1727973611228394_axial.pcd"
# trans_file = "1727973611228394_transversal.pcd"

# ax_file = "1756920955004018_axial.pcd"
# trans_file = "1756920955004018_transversal.pcd"

# Dynamic range in dB
D = 20

dpi = 300

# ##############################################################################

ax_data = read_in(path, ax_file)
trans_data = read_in(path, trans_file)
data = pd.concat([ax_data, trans_data], ignore_index=True, sort=True)

# True defect locations in original coordinates
d_a  = (-0.771 , -0.978) # 9.7 mm
d_b  = (-0.181 , -1.412) # 6.4 mm
d_c  = ( 0.518 , -1.349) # 9.7 mm
d_d  = ( 0.834 , -1.407) # 6.4 mm
d_e1 = ( 0.889 , -1.339) # 9.7 mm
d_e2 = ( 0.866 , -1.338) # 9.7 mm
d_f  = (-0.801 , -1.316) # 9.7 mm

# if ax_file == "1727973611228394_axial.pcd" and trans_file == "1727973611228394_transversal.pcd":
#     d_e1 = (0.902, -1.135)
# if ax_file == "1756920955004018_axial.pcd" and trans_file == "1756920955004018_transversal.pcd":
#     d_a = (-0.772, -0.980)

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

# Store the offset after first zeroing (for translating true defect locations)
x_offset_after_first_zero = data['x'].min()
y_offset_after_first_zero = data['y'].min()

data['x'] = zero_data_series(data['x'])
data['y'] = zero_data_series(data['y'])
data['z'] = zero_data_series(data['z'])

# Transform true defect locations to match zero'd coordinates
def transform_defect_location(original_x, original_y):
    """
    Transform original coordinates to match the zero'd data coordinates.
    zero_data_series adds abs(min) to shift everything to start at 0.
    """
    # Apply transformations: original + abs(first_min) + abs(second_min_after_trim)
    x_final = original_x + abs(x_min_original) + abs(x_offset_after_first_zero)
    y_final = original_y + abs(y_min_original) + abs(y_offset_after_first_zero)

    return x_final, y_final

# Transform all true defect locations with their true diameters
true_defects = {
    'A': (transform_defect_location(d_a[0], d_a[1]), 9.7),
    'B': (transform_defect_location(d_b[0], d_b[1]), 6.4),
    'C': (transform_defect_location(d_c[0], d_c[1]), 9.7),
    'D': (transform_defect_location(d_d[0], d_d[1]), 6.4),
    'E1': (transform_defect_location(d_e1[0], d_e1[1]), 9.7),
    'E2': (transform_defect_location(d_e2[0], d_e2[1]), 9.7),
    'F': (transform_defect_location(d_f[0], d_f[1]), 9.7)
}

print(f"\nTrue defect locations (transformed to zero'd coordinates):")
for name, ((x, y), diameter) in true_defects.items():
    print(f"  Defect {name}: ({x:.4f}, {y:.4f}) m, true diameter: {diameter} mm")

try:
    data['Magnitude'] = np.sqrt(data['Imaginary']**2 + data['Real']**2)

    # Clip Z to avoid log(0) errors
    data['dB'] = 20 * np.log10(np.clip(data['Magnitude'], 1e-12, None))

    print(f'dB Nans = {np.isnan(data["dB"]).sum()}')
    print(f'dB Infs = {np.isinf(data["dB"]).sum()}')
    print(f'dB Negatives = {(data["dB"] < 0).sum()}')
    print(f'dB Min = {data["dB"].min():.1f}, Max = {data["dB"].max():.1f}')
    print(f'dB Mean = {data["dB"].mean():.1f}, Median = {data["dB"].median():.1f}')
    print(f'dB 95th percentile = {np.percentile(data["dB"], 95):.1f}')

except Exception as e:
    print(f"Unable to convert to dB scale: {e}")
    pass

X = data['x']
Y = data['y']

###############################################################################
# DATA AUGMENTATION FOR PARTIAL DEFECTS
###############################################################################

def mirror_defect_data(data: pd.DataFrame, defect_location: tuple,
                       search_radius: float, axis: str = 'vertical',
                       keep_side: str = 'bottom',
                       position_jitter: float = 0.0001,
                       signal_jitter: float = 0.5) -> pd.DataFrame:
    """
    Mirror defect data about the defect location to complete partial data.
    Removes points on one side, then mirrors points from the other side with randomization.

    Args:
        data: Full dataset
        defect_location: (x, y) tuple of defect location (used as mirror axis)
        search_radius: Radius around defect to extract and mirror (meters)
        axis: 'vertical' to mirror up/down, 'horizontal' to mirror left/right
        keep_side: For vertical: 'bottom' or 'top'. For horizontal: 'left' or 'right'
        position_jitter: Random perturbation to add to x,y positions (meters, default 0.1mm)
        signal_jitter: Random perturbation to add to signal values (dB, default 0.5dB)

    Returns:
        DataFrame with mirrored defect data
    """
    mirror_x, mirror_y = defect_location

    # Extract local region around defect
    local_mask = (
        (np.abs(data['x'] - mirror_x) < search_radius) &
        (np.abs(data['y'] - mirror_y) < search_radius)
    )
    local_data = data[local_mask].copy()

    if len(local_data) == 0:
        print(f"  Warning: No data found within {search_radius*1000:.1f} mm for mirroring")
        return data

    print(f"  Mirror axis at defect location: ({mirror_x:.4f}, {mirror_y:.4f}) m")
    print(f"  Original points in region: {len(local_data)}")

    if axis == 'vertical':
        # Vertical mirroring (up/down) - mirror about horizontal line at mirror_y
        if keep_side == 'bottom':
            # Keep points below mirror line, remove points above
            source_data = local_data[local_data['y'] < mirror_y].copy()
            remove_mask = local_mask & (data['y'] >= mirror_y)
            print(f"  Keeping {len(source_data)} points below mirror line, removing {remove_mask.sum()} points above")
        else:  # keep_side == 'top'
            # Keep points above mirror line, remove points below
            source_data = local_data[local_data['y'] >= mirror_y].copy()
            remove_mask = local_mask & (data['y'] < mirror_y)
            print(f"  Keeping {len(source_data)} points above mirror line, removing {remove_mask.sum()} points below")

        # Mirror the source data
        mirrored_data = source_data.copy()
        mirrored_data['y'] = 2 * mirror_y - mirrored_data['y']

    elif axis == 'horizontal':
        # Horizontal mirroring (left/right) - mirror about vertical line at mirror_x
        if keep_side == 'left':
            # Keep points left of mirror line, remove points right
            source_data = local_data[local_data['x'] < mirror_x].copy()
            remove_mask = local_mask & (data['x'] >= mirror_x)
            print(f"  Keeping {len(source_data)} points left of mirror line, removing {remove_mask.sum()} points right")
        else:  # keep_side == 'right'
            # Keep points right of mirror line, remove points left
            source_data = local_data[local_data['x'] >= mirror_x].copy()
            remove_mask = local_mask & (data['x'] < mirror_x)
            print(f"  Keeping {len(source_data)} points right of mirror line, removing {remove_mask.sum()} points left")

        # Mirror the source data
        mirrored_data = source_data.copy()
        mirrored_data['x'] = 2 * mirror_x - mirrored_data['x']

    else:
        raise ValueError(f"Invalid axis: {axis}. Must be 'vertical' or 'horizontal'")

    # Apply second mirror to create rotational symmetry instead of mirror symmetry
    # This mirrors about the opposite axis to create 180° rotation
    double_mirrored_data = mirrored_data.copy()

    if axis == 'vertical':
        # Already mirrored vertically, now mirror horizontally
        double_mirrored_data['x'] = 2 * mirror_x - double_mirrored_data['x']
    elif axis == 'horizontal':
        # Already mirrored horizontally, now mirror vertically
        double_mirrored_data['y'] = 2 * mirror_y - double_mirrored_data['y']

    # Add randomization to both mirrored datasets to make them look more natural
    n_mirrored = len(mirrored_data)
    n_double_mirrored = len(double_mirrored_data)

    # Add position jitter (small random perturbations)
    np.random.seed(42)  # For reproducibility

    # Jitter for first mirror
    mirrored_data['x'] += np.random.normal(0, position_jitter, n_mirrored)
    mirrored_data['y'] += np.random.normal(0, position_jitter, n_mirrored)
    mirrored_data['z'] += np.random.normal(0, position_jitter, n_mirrored)

    # Jitter for second mirror (rotational complement)
    np.random.seed(43)  # Different seed for different pattern
    double_mirrored_data['x'] += np.random.normal(0, position_jitter, n_double_mirrored)
    double_mirrored_data['y'] += np.random.normal(0, position_jitter, n_double_mirrored)
    double_mirrored_data['z'] += np.random.normal(0, position_jitter, n_double_mirrored)

    # Add signal jitter to magnitude/dB values - first mirror
    if 'Magnitude' in mirrored_data.columns:
        mag_jitter_factor = np.random.normal(1.0, signal_jitter * 0.01, n_mirrored)
        mirrored_data['Magnitude'] *= mag_jitter_factor

    if 'Real' in mirrored_data.columns and 'Imaginary' in mirrored_data.columns:
        # Add jitter to complex components
        real_jitter = np.random.normal(0, signal_jitter * 0.01, n_mirrored)
        imag_jitter = np.random.normal(0, signal_jitter * 0.01, n_mirrored)
        mirrored_data['Real'] *= (1 + real_jitter)
        mirrored_data['Imaginary'] *= (1 + imag_jitter)

        # Recalculate magnitude from perturbed complex values
        mirrored_data['Magnitude'] = np.sqrt(mirrored_data['Imaginary']**2 + mirrored_data['Real']**2)

    # Recalculate dB from perturbed magnitude
    if 'dB' in mirrored_data.columns:
        mirrored_data['dB'] = 20 * np.log10(np.clip(mirrored_data['Magnitude'], 1e-12, None))

    # Add signal jitter to magnitude/dB values - second mirror (rotational)
    if 'Magnitude' in double_mirrored_data.columns:
        mag_jitter_factor = np.random.normal(1.0, signal_jitter * 0.01, n_double_mirrored)
        double_mirrored_data['Magnitude'] *= mag_jitter_factor

    if 'Real' in double_mirrored_data.columns and 'Imaginary' in double_mirrored_data.columns:
        # Add jitter to complex components
        real_jitter = np.random.normal(0, signal_jitter * 0.01, n_double_mirrored)
        imag_jitter = np.random.normal(0, signal_jitter * 0.01, n_double_mirrored)
        double_mirrored_data['Real'] *= (1 + real_jitter)
        double_mirrored_data['Imaginary'] *= (1 + imag_jitter)

        # Recalculate magnitude from perturbed complex values
        double_mirrored_data['Magnitude'] = np.sqrt(double_mirrored_data['Imaginary']**2 + double_mirrored_data['Real']**2)

    # Recalculate dB from perturbed magnitude
    if 'dB' in double_mirrored_data.columns:
        double_mirrored_data['dB'] = 20 * np.log10(np.clip(double_mirrored_data['Magnitude'], 1e-12, None))

    print(f"  Added {len(mirrored_data)} mirrored points + {len(double_mirrored_data)} rotationally symmetric points")
    print(f"  Randomization: pos ±{position_jitter*1000:.2f}mm, signal ±{signal_jitter:.1f}dB")

    # Remove points from target region
    data_without_target = data[~remove_mask].copy()

    # Combine: original data (minus removed region) + source data + single mirror + double mirror (rotational)
    combined_data = pd.concat([data_without_target, source_data, mirrored_data, double_mirrored_data], ignore_index=True)

    print(f"  Total dataset size: {len(data)} -> {len(combined_data)} points")

    return combined_data


# Mirror defect A (vertical - keep bottom, mirror to top)
print("\n" + "="*80)
print("MIRRORING PARTIAL DEFECTS")
print("="*80)

print("\nDefect A: Mirroring vertically - keeping bottom, mirroring to top")
defect_a_loc, defect_a_diam = true_defects['A']
data = mirror_defect_data(data, defect_a_loc, search_radius=0.010, axis='vertical', keep_side='bottom')

print("\nDefect E1: Mirroring horizontally - keeping left, mirroring to right")
defect_e1_loc, defect_e1_diam = true_defects['E1']
data = mirror_defect_data(data, defect_e1_loc, search_radius=0.010, axis='horizontal', keep_side='left')

print("="*80)

X = data['x']
Y = data['y']

###############################################################################
# DEFECT SIZING (At Known Locations)
###############################################################################

print("\n" + "="*80)
print("DEFECT SIZING AT KNOWN LOCATIONS")
print("="*80)
print(f"DBSCAN params: eps={DBSCAN_EPS}m, min_samples={DBSCAN_MIN_SAMPLES}")
print(f"Contour params: drop=[6.0, 12.0]dB, region={CONTOUR_REGION_SIZE}m, res={CONTOUR_RESOLUTION}m")
print(f"All methods run at both -6dB and -12dB thresholds")
print("="*80)

measurements = []
for defect_id, (defect_name, ((true_x, true_y), true_diameter)) in enumerate(true_defects.items()):
    print(f"\n--- Processing Defect {defect_name} ---")

    # Find local maximum around the specified location (within 5mm)
    search_radius = 0.005  # 5mm
    local_region = data[
        (np.abs(data['x'] - true_x) < search_radius) &
        (np.abs(data['y'] - true_y) < search_radius)
    ]

    if len(local_region) > 0:
        # Find the point with maximum dB in the local region
        max_idx = local_region['dB'].idxmax()
        peak_x = data.loc[max_idx, 'x']
        peak_y = data.loc[max_idx, 'y']
        peak_db = data.loc[max_idx, 'dB']

        # Report if peak was offset from specified location
        offset_distance = np.sqrt((peak_x - true_x)**2 + (peak_y - true_y)**2)
        if offset_distance > 0.001:  # More than 1mm offset
            print(f"  Note: Local maximum found {offset_distance*1000:.2f} mm from specified location")
    else:
        # Fallback to nearest point if no data in search region
        print(f"  Warning: No data found within {search_radius*1000:.1f} mm, using nearest point")
        distances = np.sqrt((data['x'] - true_x)**2 + (data['y'] - true_y)**2)
        nearest_idx = distances.idxmin()
        peak_x = data.loc[nearest_idx, 'x']
        peak_y = data.loc[nearest_idx, 'y']
        peak_db = data.loc[nearest_idx, 'dB']

    # Create measurement using all three methods
    measurement = create_defect_measurement(defect_name, (peak_x, peak_y), peak_db, data, true_diameter)
    measurements.append(measurement)

###############################################################################
# RESULTS OUTPUT
###############################################################################

print_measurement_summary(measurements)

# Save to files
output_dir = "/home/matthew/Downloads/"
save_measurements_csv(measurements, f"{output_dir}defect_measurements.csv")
save_measurements_json(measurements, f"{output_dir}defect_measurements.json")

# Create comparison dataframe
comparison_df = compare_measurements(measurements)

print("\n" + "="*80)
print("COMPARISON TABLE")
print("="*80)
print(comparison_df.to_string(index=False))

###############################################################################
# VISUALIZATION
###############################################################################

if ENABLE_PLOTS:
    print("\n" + "="*80)
    print("GENERATING VISUALIZATIONS")
    print("="*80)

    # Main C-scan with all defects
    # print("\nGenerating C-Scan with defect boundaries...")
    # plot_cscan_with_defects(data, measurements, D=D, dpi=dpi,
    #                         save_path=f"{path}cscan_defects.png",
    #                         true_defects=true_defects)

    # Combined defect plots (all in one figure)
    print("\nGenerating combined defect plots...")
    plot_individual_defects_combined(data, measurements, D=D, dpi=dpi,
                                     save_path=f"{path}defect_sizing_defects_combined.png")

    # Individual defect plots (separate files) - uncomment if needed
    # print("\nGenerating individual defect plots...")
    # plot_individual_defects(data, measurements, D=D, dpi=dpi,
    #                         save_dir=f"{path}")

    # Method comparison
    print("\nGenerating method comparison plot...")
    plot_method_comparison(measurements, dpi=dpi,
                          save_path=f"{path}defect_sizing_method_comparison.png")

print("\n" + "="*80)
print("DEFECT SIZING COMPLETE")
print("="*80)
