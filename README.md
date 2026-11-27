[![docker_ci](https://github.com/MShields1986/eddyfi_ectane_ros/actions/workflows/industrial_ci_action.yml/badge.svg?branch=noetic)](https://github.com/MShields1986/eddyfi_ectane_ros/actions/workflows/industrial_ci_action.yml?query=branch%3Anoetic)

# eddyfi_ectane_ros
ROS driver for use with the Eddyfi Ectane SDK ([video of it in use](https://vimeo.com/1123786342)), targeting ROS 1 Noetic on Ubuntu 20.04. The ROS 2 Lyrical Luth driver is on `main`, and the ROS 2 Jazzy Jalisco and Humble Hawksbill drivers are on the `jazzy` and `humble` branches.


## Citation
If you use this repository for your research, please cite the following conference talk where it was first shared.

    @conference{shields2025roscon,
        title = "{ROS} Enabled Non-Destructive Testing for Aerospace Manufacturing",
        author = "Matthew Shields",
        booktitle = "ROSCon UK 2025",
        address = "Edinburgh, UK",
        year = "2025",
        month = sep,
        note = "Conference date: 15-09-2025 Through 17-09-2025",
        url = "https://vimeo.com/1123786342",
    }


## Build
Clone into a catkin workspace and build:
```bash
cd ~/catkin_ws/src
git clone -b noetic https://github.com/MShields1986/eddyfi_ectane_ros.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
```
The socket classes come from [boost_asio_socket_wrappers](https://github.com/MShields1986/boost_asio_socket_wrappers), which CMake clones at a pinned release when the package is first configured, so that needs network access.

Or build it in Docker, and run the driver on the host's network so the C SDK can reach it:
```bash
docker compose -f docker/docker-compose.yaml run --rm eddyfi-ectane roslaunch eddyfi_ectane init.launch configuration:=lf_probe_axial_and_transversal_1_channel
```


## Multi-frequency acquisition
The `hf_probe_axial_and_transversal_4_channels` and `lf_probe_axial_and_transversal_4_channels` configurations acquire four frequencies in a single pass. The excitation for each frequency is set in `eddyfi_ectane/config/settings.yaml`:
```yaml
frequencies: [5000, 10000, 20000, 50000]
voltages: [7, 7, 7, 7]
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

`init.launch` opens `rviz/config_multi_frequency.rviz` for multi-frequency configurations, with a display group per frequency. Only frequency 1 is enabled initially, as the frequencies overlap.


## Tests
`eddyfi_ectane/scripts/fake_sdk.py` stands in for the C SDK, so the driver can run without an Ectane. Start it with the same configuration as the driver:
```bash
python3 eddyfi_ectane/scripts/fake_sdk.py lf_probe_axial_and_transversal_1_channel
```
It streams the channels the configuration enables in the C SDK's `Configurations.h`, read from `eddyfi_ectane_sdk_ros` alongside this repository or from `--configurations-h`, or without the C SDK, the channels in the configuration's `config/probes/crosstalk_<configuration>.yaml`. It prints the commands it receives. The values are not eddy current signals, but each one encodes the channel it came from.

The tests in `eddyfi_ectane/test` run the driver against it:
- `pipeline_test.sh` checks that every channel of the observations and point clouds holds the data streamed for it, and that Ctrl-C disconnects from the SDK
- `shutdown_test.sh` checks that the driver shuts down promptly when data stops arriving
- `failure_test.sh` checks that bad settings are reported and the Ectane is not started

`./test.sh` builds the Docker image in `docker/` and runs them all in it, as GitHub Actions does on every push. To run one in a built workspace instead:
```bash
source devel/setup.bash
src/eddyfi_ectane_ros/eddyfi_ectane/test/pipeline_test.sh hf_probe_axial_and_transversal_4_channels 4
```
`eddyfi_ectane/test/run_all.sh` lists every case.


## TODO
1. Enable `bInterpolation` in `inst_stETConfiguration` (Configurations.h and InstDefsEctIris.h) via the C SDK to interpolate to correct for the time difference in timeslots.
