# eddyfi_ectane_ros
ROS driver for use with the Eddyfi Ectane SDK


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


## TODO
1. Enable `bInterpolation` in `inst_stETConfiguration` (Configurations.h and InstDefsEctIris.h) via the C SDK to interpolate to correct for the time difference in timeslots.
