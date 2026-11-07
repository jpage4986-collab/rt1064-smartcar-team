# SeekFree MT9V03X configuration bridge

Source: the user's `Desktop/一些文件/镜头组例程/libraries` reference,
SeekFree RT1064 library, dated 2022-09-21. The supplied `zf_device_config.lib`
implements MT9V03X sensor setup via SCCB. SHA256:
`D8F07E0EE2FE1BD4DBC68EE9E17B6FA48973C718CBDB02CF61369A316A8271E8`.
The supplied header declares GPL-3.0-or-later; its license and original
copyright headers are retained. The configuration implementation is supplied
as a binary archive, not source; this project does not claim to author it.

`USER/src/seekfree_soft_iic.c` and `USER/inc/seekfree_soft_iic.h` are copies
of the reference's `zf_driver_soft_iic` with includes adapted to the legacy
GPIO API. `seekfree_sccb_compat.h` maps those GPIO calls without changing the
SCCB transaction order. `camera_sccb_config.c` supplies the new configuration
table order and delay wrapper. Initialization tries SCCB on C29/C28 first,
then falls back to the existing UART5 implementation, as in the reference.

The binary uses Cortex-M7 Thumb-2, hard FPv5 ABI, compatible with this build.
Its only external functions are the two SCCB register helpers and delay.
