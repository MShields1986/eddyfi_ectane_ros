# eddyfi_ectane_ros
ROS 2 driver for use with the Eddyfi Ectane SDK, targeting ROS 2 Lyrical Luth on Ubuntu 26.04. The ROS 1 Noetic driver is on `main`.


## Build
Clone into a colcon workspace and build:
```bash
cd ~/ros2_ws/src
git clone -b lyrical https://github.com/MShields1986/eddyfi_ectane_ros.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select eddyfi_ectane
source install/setup.bash
```


## Usage
Start the driver, then the C SDK from [eddyfi_ectane_sdk_ros](https://github.com/MShields1986/eddyfi_ectane_sdk_ros), which connects to the driver on start up:
```bash
ros2 launch eddyfi_ectane init.launch.py configuration:=lf_probe_axial_and_transversal_1_channel
```
The configurations are listed at the top of `init.launch.py`. Add `no_robot_mode:=true` to broadcast a dummy transform for the probe instead of using a robot, which also opens RViz, and `logging:=true` to record a bag.

The reconstruction can run on another machine, with the same configuration as the driver:
```bash
ros2 launch eddyfi_ectane distributed_reconstruction.launch.py configuration:=lf_probe_axial_and_transversal_1_channel
```
or in the driver's container with `reconstruction:=true`, so the whole pipeline passes messages within one process.

Start and stop streaming, and balance, with the services:
```bash
ros2 service call /ectane/stream_data eddyfi_ectane/srv/StreamData "{stream_data: true}"
ros2 service call /ectane/balance_device eddyfi_ectane/srv/BalanceDevice "{balance: true}"
```

Replay a recording in RViz with `ros2 launch eddyfi_ectane replay.launch.py bag:=<bag>`. Bags recorded with ROS 1 need converting first, e.g. with [rosbags](https://pypi.org/project/rosbags/) (`rosbags-convert --src <bag>.bag --dst <bag>`).


## Changes from ROS 1
- The nodelets are components, which `init.launch.py` loads into one multi-threaded container. Each can also run on its own, e.g. `ros2 run eddyfi_ectane ectane_node`.
- The components use intra-process communication, publishing messages by moving them and taking ownership of the messages they subscribe to, so components in the same container pass messages without copying them. A subscriber in another process, such as RViz or a bag recording, gets its own copy as usual.
- There is no parameter server, so each node loads the YAML files it needs. The configuration files are in the ROS 2 format, under `/**: ros__parameters:`. A missing or invalid setting stops the node loading with an error, rather than it waiting for the parameter.
- The reconstruction launch takes the configuration, as the reconstruction reads its crosstalk configuration to find how many frequencies to reconstruct.
- The Ectane node reads the C SDK's data stream on its own thread, and validates its settings before waiting for the C SDK to connect.
- The data topics are reliable and volatile. The reconstructions are transient local, so a late subscriber such as RViz gets the latest reconstruction straight away.
- Bags are recorded with `ros2 bag record`, to `rosbag_name` relative to where the launch is run.


## Multi-frequency acquisition
The `hf_probe_axial_and_transversal_4_channels` and `lf_probe_axial_and_transversal_4_channels` configurations acquire four frequencies in a single pass. The excitation for each frequency is set in `eddyfi_ectane/config/settings.yaml`:
```yaml
frequencies: [5000, 10000, 20000, 50000]
voltages: [7.0, 7.0, 7.0, 7.0]
```
Single frequency configurations use only the first entry.

Crosstalk channels for these configurations are named `F<n><coil>`, e.g. `F2A05` is coil `A05` at the second frequency, and share the coil's geometry entry. Each frequency gets its own axial and transversal clouds:

| Single frequency | Multi-frequency |
| --- | --- |
| `/ectane/axial_data_at_probe` | `/ectane/frequency_<n>/axial_data_at_probe` |
| `/ectane/transversal_data_at_probe` | `/ectane/frequency_<n>/transversal_data_at_probe` |
| `/ectane/distributed_axial_reconstruction` | `/ectane/frequency_<n>/distributed_axial_reconstruction` |
| `/ectane/distributed_transversal_reconstruction` | `/ectane/frequency_<n>/distributed_transversal_reconstruction` |

`/ectane/eddy_current_data` carries every channel, with each channel's `frequency` set to its excitation frequency.

`init.launch.py` opens `rviz/config_multi_frequency.rviz` for multi-frequency configurations, with a display group per frequency. Only frequency 1 is enabled initially, as the frequencies overlap.


## TODO
1. Enable `bInterpolation` in `inst_stETConfiguration` (Configurations.h and InstDefsEctIris.h) via the C SDK to interpolate to correct for the time difference in timeslots.
