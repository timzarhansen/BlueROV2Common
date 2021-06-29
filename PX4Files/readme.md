# PX4 extension for BlueROV2

only saves files that are important for BlueROV2

Instructions to integrate files in PX4:

* add `manualControl.cpp` content to `uuv_att_control.cpp` after line:`if (_vcontrol_mode.flag_control_manual_enabled && !_vcontrol_mode.flag_control_rates_enabled) {`
* change `vectored6dof_sitl.main.mix` in `mixers-sitl`
* change `default.cmake` in `boards/px4/fmu-v2`


