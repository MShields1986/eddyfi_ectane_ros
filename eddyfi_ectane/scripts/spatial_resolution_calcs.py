import yaml

geom_ax_f = open("/home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/src/eddyfi_ectane_ros/eddyfi_ectane/config/probes/lf_axial_geometry.yaml")
geom_trans_f = open("/home/matthew/Desktop/phd_workspaces/kuka_kmr_driver/catkin_ws/src/eddyfi_ectane_ros/eddyfi_ectane/config/probes/lf_transversal_geometry.yaml")

geom_ax = yaml.safe_load(geom_ax_f)
geom_trans = yaml.safe_load(geom_trans_f)

del geom_ax["geometry"]["A01"]
del geom_ax["geometry"]["A31"]

prev_y = 0
for channel in geom_ax["geometry"]:
    y = geom_ax["geometry"][channel]["y"]
    print(round(prev_y - y, 4))
    prev_y = y

prev_y = 0
for channel in geom_trans["geometry"]:
    y = geom_trans["geometry"][channel]["y"]
    print(round(prev_y - y, 4))
    prev_y = y

for ch_ax, ch_tr in zip(geom_ax["geometry"], geom_trans["geometry"]):
    y_ax = geom_ax["geometry"][ch_ax]["y"]
    y_tr = geom_trans["geometry"][ch_tr]["y"]
    print(ch_ax + " || " + ch_tr)
    print(round(y_ax - y_tr, 4))

base_vel = 0.05 # m/s
f = 100 # Hz
dt = 1 / f
dx = base_vel * dt

print("NDT X Spatial Resolution: " + str(dx * 1000) + " mm")