# eddyfi_ectane_ros
ROS 2 driver for use with the Eddyfi Ectane SDK, targeting ROS 2 Jazzy Jalisco on Ubuntu 24.04. The ROS 2 Lyrical Luth driver is on `main`, and the ROS 1 Noetic driver is on the `noetic` branch.


## Citation
If you use this repository for your research, please cite the following paper where it was first shared.

    @misc{shields2026ssrn,
        title = "``Process-to-Part'' Eddy Current Array Inspection of an Aluminium Fuselage Section",
        author = "Matthew Shields and Randika K. W. Vithanage and Gareth Pierce and Ehsan Mohseni and Rebecca Moos and Gordon Dobie and Charles Macleod and Michael Grosser and Tom O'Hare and Gavin Munro and Janet O'Brien-O'Reilly",
        year = "2026",
        note = "SSRN preprint",
        doi = "10.2139/ssrn.6303235",
        url = "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=6303235",
    }


## Build
Clone into a colcon workspace and build:
```bash
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/MShields1986/eddyfi_ectane_ros.git
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

To run the driver without an Ectane, start `eddyfi_ectane/scripts/fake_sdk.py` with the same configuration in place of the C SDK:
```bash
python3 eddyfi_ectane/scripts/fake_sdk.py lf_probe_axial_and_transversal_1_channel
```
It streams the channels the configuration enables in the C SDK's `Configurations.h`, read from `eddyfi_ectane_sdk_ros` alongside this repository or from `--configurations-h`, and prints the commands it receives. The values are not eddy current signals, but each one encodes the channel it came from.


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
