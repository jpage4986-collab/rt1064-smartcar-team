/* Adapter to SeekFree's supplied 2022 MT9V03X configuration library.
 * See vendor/seekfree-camera-config/README.md for provenance and license. */
#include "seekfree_soft_iic.h"
#include "zf_systick.h"
#include "SEEKFREE_MT9V03X_CSI.h"

extern unsigned char mt9v03x_set_config_sccb(void *, short int [10][2]);
static soft_iic_info_struct camera_iic;

/* NEW library order: INIT first. Legacy UART table has INIT last. */
uint8 camera_sccb_configure(void)
{
    int16 settings[10][2] = {
        {0, 0}, {1, 0}, {2, 512}, {3, 50},
        {4, MT9V03X_CSI_W}, {5, MT9V03X_CSI_H},
        {6, 0}, {7, 0}, {8, 32}, {9, 0}
    };
    systick_delay_ms(200U);
    soft_iic_init(&camera_iic, 0U, 800U, C29, C28);
    return mt9v03x_set_config_sccb(&camera_iic, settings);
}

void system_delay_ms(uint32 ms)
{
    systick_delay_ms(ms);
}
