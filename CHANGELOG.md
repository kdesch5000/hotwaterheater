# Changelog

## 2026-10-30: v1.9.2, motion sensing off by default

- **Changed:** Wi-Fi CSI motion screen-off now defaults to disabled, and its config option warns that it may cause reboots.
- **Why:** boots rose from 1 in 14 days to about 19 in 4 days after CSI arrived in v1.8.0, and v1.9.1 still crashed. The decoded dump shows a jump to address 0 from `wDev_ProcessFiq` (Wi-Fi driver interrupt) while `ppTask` was in `wDev_IndicateFrame`. None of this project's code was on the stack, so this is a strong correlation, not a proven root cause.
- **Impact:** the display stays on all the time. Devices that already saved the setting keep their saved value.
