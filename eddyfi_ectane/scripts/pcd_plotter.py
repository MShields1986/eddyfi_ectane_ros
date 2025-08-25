#!/usr/bin/env python3

import numpy as np
import pandas as pd

from matplotlib import cm, ticker
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from mpl_toolkits.axes_grid1 import make_axes_locatable


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
# Input
# ##############################################################################

# path = "/media/matthew/b2603e49-ec47-413e-94db-d319afa80e0a/home/matthew/Desktop/phd_data/ec/"
# ax_file = "1755002810444365_axial.pcd"
# trans_file = "1755002810444365_transversal.pcd"

path = "/home/matthew/Downloads/ec_old/"
ax_file = "1756920955004018_axial.pcd"
trans_file = "1756920955004018_transversal.pcd"

# Dynamic range in dB
D = 20 #20 #40 #60 #etc.

dpi = 300

# ##############################################################################

ax_data = read_in(path, ax_file)
trans_data = read_in(path, trans_file)
data = pd.concat([ax_data, trans_data], ignore_index=True, sort=True)

data['x'] = zero_data_series(data['x'])
data['y'] = zero_data_series(data['y'])
data['z'] = zero_data_series(data['z'])

# bodge
tx = 0.03 # m
trimmed = data[data['x'] > tx].reset_index(drop=True)
del(data)
data = trimmed

data['x'] = zero_data_series(data['x'])
data['y'] = zero_data_series(data['y'])
data['z'] = zero_data_series(data['z'])

try:
    data['Magnitude'] = np.sqrt(data['Imaginary']**2 + data['Real']**2)

    # Clip Z to avoid log(0) errors
    data['dB'] = 20 * np.log10(np.clip(data['Magnitude'], 1e-12, None))
    
    print(f'dB Nans = {np.isnan(data["dB"]).sum()}')
    print(f'dB Infs = {np.isinf(data["dB"]).sum()}')
    print(f'dB Negatives = {(data["dB"] < 0).sum()}')

except Exception as e:
    print(f"Unable to convert to dB scale: {e}")
    pass

X = data['x']
Y = data['y']

###############################################################################
# dB Plot
###############################################################################

try:
    print(f'--- Plotting dB C-Scan ---')

    Z = data['dB']

    vmin, vmax = Z.max() - D, Z.max()
    levels = np.linspace(vmin - 1, vmax + 1, 800)
    print(f'vmin = {vmin}')
    print(f'vmax = {vmax}')

    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 3))
    # im = ax.tricontourf(
    im = ax.tripcolor(
        X, Y, Z,
        # levels=levels,
        shading="gouraud",
        cmap="rainbow",
        vmin=vmin, vmax=vmax
    )

    ax.set_title("C-Scan Reconstruction")
    ax.set_xlabel("Scan Axis [m]")
    ax.set_ylabel("Array Axis [m]")
    ax.set_aspect('equal')
    ax.set_xlim(X.min(), X.max())
    ax.set_ylim(Y.min(), Y.max())

    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)

    ticks = np.linspace(vmin, vmax, D//5 + 1)
    tick_labels = [f"{int(t - vmax)}" for t in ticks]
    cbar = plt.colorbar(im, cax=cax, ticks=ticks)
    # cbar = plt.colorbar(im, cax=cax, ticks=ticks, boundaries=[vmin, vmax])
    cbar.ax.set_yticklabels(tick_labels)
    cbar.set_label("Relative Amplitude [dB]")
    cbar.ax.invert_yaxis()

    plt.tight_layout()
    #plt.show()
    path = "/home/matthew/Desktop/PhD/process_to_part_eddy_current_paper/"
    plt.savefig(f'{path}images/plot_ec_db.png', dpi=dpi)
    #plt.savefig(f'{path}images/{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot C Scan: {e}")
    pass
#plt.close()


###############################################################################
# dB Defect Plots
###############################################################################

# defects = ['A',
#            'C',
#            'E',
#            'F']

# for defect in defects:
#     try:
#         print(f'--- Plotting dB Defect {defect} ---')

#         Z = data['dB']

#         vmin, vmax = Z.max() - D, Z.max()
#         levels = np.linspace(vmin - 1, vmax + 1, 800)
#         print(f'vmin = {vmin}')
#         print(f'vmax = {vmax}')

#         fig, ax = plt.subplots(dpi=dpi, figsize=(5, 3))
#         im = ax.tripcolor(
#             X, Y, Z,
#             shading="gouraud",
#             cmap="rainbow",
#             vmin=vmin, vmax=vmax
#         )

#         ax.set_title(f"C-Scan Reconstruction - Defect {defect}")
#         ax.set_xlabel("Scan Axis [m]")
#         ax.set_ylabel("Array Axis [m]")
#         ax.set_aspect('equal')
#         if defect == 'A':
#             ax.set_xlim(0.1, 0.125)
#             ax.set_ylim(0.44, 0.455)
#         elif defect == 'C':
#             ax.set_xlim(1.39, 1.41)
#             ax.set_ylim(0.07, 0.09)
#         elif defect == 'E':
#             ax.set_xlim(1.74, X.max())
#             ax.set_ylim(0.07, 0.11)
#         elif defect == 'F':
#             ax.set_xlim(X.min(), 0.1)
#             ax.set_ylim(0.1, 0.13)

#         divider = make_axes_locatable(ax)
#         cax = divider.append_axes("right", size="5%", pad=0.05)

#         ticks = np.linspace(vmin, vmax, D//5 + 1)
#         tick_labels = [f"{int(t - vmax)}" for t in ticks]
#         cbar = plt.colorbar(im, cax=cax, ticks=ticks)
#         cbar.ax.set_yticklabels(tick_labels)
#         cbar.set_label("Relative Amplitude [dB]")
#         cbar.ax.invert_yaxis()

#         plt.tight_layout()
#         path = "/home/matthew/Desktop/PhD/process_to_part_eddy_current_paper/"
#         plt.savefig(f'{path}images/plot_ec_db_defect_{defect.lower()}.png', dpi=dpi)
#     except Exception as e:
#         print(f"Unable to plot C Scan: {e}")
#         pass

###############################################################################
# Imaginary Defect Plots
###############################################################################

# defects = ['A',
#            'C',
#            'E',
#            'F']

# for defect in defects:
#     try:
#         print(f'--- Plotting Imaginary Defect {defect} ---')

#         Z = data['Imaginary']

#         vmin, vmax = Z.min(), Z.max()
#         print(f'vmin = {vmin}')
#         print(f'vmax = {vmax}')

#         fig, ax = plt.subplots(dpi=dpi, figsize=(5, 3))
#         im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=vmin, vmax=vmax)

#         ax.set_title(f"C-Scan Reconstruction - Defect {defect}")
#         ax.set_xlabel("Scan Axis [m]")
#         ax.set_ylabel("Array Axis [m]")
#         ax.set_aspect('equal')
#         if defect == 'A':
#             ax.set_xlim(0.1, 0.125)
#             ax.set_ylim(0.44, 0.455)
#         elif defect == 'C':
#             ax.set_xlim(1.39, 1.41)
#             ax.set_ylim(0.07, 0.09)
#         elif defect == 'E':
#             ax.set_xlim(1.74, X.max())
#             ax.set_ylim(0.07, 0.11)
#         elif defect == 'F':
#             ax.set_xlim(X.min(), 0.1)
#             ax.set_ylim(0.1, 0.13)

#         divider = make_axes_locatable(ax)
#         cax = divider.append_axes("right", size="5%", pad=0.05)
#         cbar = plt.colorbar(im,
#                             cax=cax,
#                             label="Impedance",
#                             ticks=np.arange(-1000.0, 1000.0, 50.0, dtype=np.float32),
#                             boundaries=[Z.min(), Z.max()]
#                             )
#         plt.tight_layout()
#         path = "/home/matthew/Desktop/PhD/process_to_part_eddy_current_paper/"
#         plt.savefig(f'{path}images/plot_ec_imaginary_defect_{defect.lower()}.png', dpi=dpi)
#     except Exception as e:
#         print(f"Unable to plot C Scan: {e}")
#         pass

###############################################################################
# Imaginary Plot
###############################################################################

try:
    print(f'--- Plotting Imaginary C-Scan ---')

    Z = data['Imaginary']
    Z = Z - Z.min()
    Z = Z / Z.max()
    Z = (Z - 0.5) * 2

    vmin, vmax = Z.min(), Z.max()
    print(f'vmin = {vmin}')
    print(f'vmax = {vmax}')
    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 3))
#    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow')
    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=vmin, vmax=vmax)
    ax.set_title("Imaginary Component C-Scan Reconstruction")
    ax.set_xlabel("Scan Axis [m]")
    ax.set_ylabel("Array Axis [m]")
    ax.set_aspect('equal')
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(im, 
                        cax=cax, 
                        label="Normalised Impedance [-]",
                        ticks=np.arange(-10.0, 10.0, 0.5, dtype=np.float32),
                        boundaries=[Z.min(), Z.max()]
                        )
    plt.tight_layout()
    #plt.show()
    path = "/home/matthew/Desktop/PhD/process_to_part_eddy_current_paper/"
    plt.savefig(f'{path}images/plot_ec_imaginary.png', dpi=dpi)
    #plt.savefig(f'{path}images/{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot C Scan: {e}")
    pass
#plt.close()

###############################################################################
# Real Plot
###############################################################################

try:
    print(f'--- Plotting Real C-Scan ---')

    Z = data['Real']
    Z = Z - Z.min()
    Z = Z / Z.max()
    Z = (Z - 0.5) * 2

    vmin, vmax = Z.min(), Z.max()
    print(f'vmin = {vmin}')
    print(f'vmax = {vmax}')
    fig, ax = plt.subplots(dpi=dpi, figsize=(9, 3))
#    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow')
    im = ax.tricontourf(X, Y, Z, 800, cmap='rainbow', vmin=vmin, vmax=vmax)
    ax.set_title("Real Component C-Scan Reconstruction")
    ax.set_xlabel("Scan Axis [m]")
    ax.set_ylabel("Array Axis [m]")
    ax.set_aspect('equal')
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(im, 
                        cax=cax, 
                        label="Normalised Impedance [-]",
                        ticks=np.arange(-10.0, 10.0, 0.5, dtype=np.float32),
                        boundaries=[Z.min(), Z.max()]
                        )
#    ax.axvline(b_scan_pos,
#               color='grey',   
#               linestyle='--',
#               linewidth=2,
#               label=f"B-Scan Position ({str(round(b_scan_pos, 1))} m)"
#               )
#    ax.legend(loc='upper right')
    plt.tight_layout()
    #plt.show()
    path = "/home/matthew/Desktop/PhD/process_to_part_eddy_current_paper/"
    plt.savefig(f'{path}images/plot_ec_real.png', dpi=dpi)
    #plt.savefig(f'{path}images/{file.split(".")[0]}.png', dpi=dpi)
except Exception as e:
    print(f"Unable to plot C Scan: {e}")
    pass
#plt.close()
