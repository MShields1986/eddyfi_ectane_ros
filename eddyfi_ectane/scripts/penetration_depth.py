#!/usr/bin/env python3

import numpy as np
import pandas as pd

from matplotlib import cm, ticker
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from mpl_toolkits.axes_grid1 import make_axes_locatable

# ##############################################################################
# Input
# ##############################################################################

# δ = 1 / (π f µ σ)^0.5

f_min = 1 * 10**3
f_max = 10 * 10**3
f_step = 0.5 * 10**3

sig_min = 2.8 * 10**7
sig_max = 4.2 * 10**7
sig_step = 0.1 * 10**7

# Create frequency and conductivity ranges
f = np.arange(f_min, f_max + f_step, f_step)
sig = np.arange(sig_min, sig_max + sig_step, sig_step)

# Create mesh grid for 2D plotting
F, SIG = np.meshgrid(f, sig)

# Permeability of free space (H/m)
mu = 4 * np.pi * 10**(-7)

# Calculate penetration depth in meters
pen_depth = 1 / np.sqrt(np.pi * F * mu * SIG)

# Convert to millimeters for better readability
pen_depth_mm = pen_depth * 1000

# ##############################################################################
# Plotting
# ##############################################################################

fig, ax = plt.subplots(figsize=(10, 8))

# Create contour plot
contour = ax.contourf(F / 1000, SIG, pen_depth_mm, levels=20, cmap='viridis')

# Add contour lines
contour_lines = ax.contour(F / 1000, SIG, pen_depth_mm, levels=20, colors='black', alpha=0.5, linewidths=0.5)
ax.clabel(contour_lines, inline=True, fontsize=12, fmt='%.2f mm')

# Add colorbar
divider = make_axes_locatable(ax)
cax = divider.append_axes("right", size="5%", pad=0.1)
cbar = plt.colorbar(contour, cax=cax)
cbar.set_label('Penetration Depth (mm)', rotation=270, labelpad=20)

# Labels and title
ax.set_xlabel('Frequency (kHz)', fontsize=12)
ax.set_ylabel('Conductivity (MS/m)', fontsize=12)
ax.set_title('Penetration Depth vs Frequency and Conductivity', fontsize=14)

# Grid
ax.grid(True, alpha=0.3, linestyle='--')

plt.tight_layout()
plt.show()

print(f"Frequency range: {f_min/1000:.1f} - {f_max/1000:.1f} kHz")
print(f"Conductivity range: {sig_min:.1f} - {sig_max:.1f} MS/m")
print(f"Penetration depth range: {pen_depth_mm.min():.2f} - {pen_depth_mm.max():.2f} mm")
print(f"Penetration depth @ 5 kHz and 30 MS/m: {1 / np.sqrt(np.pi * 5 * 10**3 * mu * 3.0 * 10**7) * 1000} mm")
print(f"Penetration depth @ 5 kHz and 38 MS/m: {1 / np.sqrt(np.pi * 5 * 10**3 * mu * 3.8 * 10**7) * 1000} mm")

# ##############################################################################
