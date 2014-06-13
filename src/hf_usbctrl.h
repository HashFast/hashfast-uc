/* hf_usbctrl.h */

#ifndef _hf_usbctrl_h
#define _hf_usbctrl_h

#ifdef __cplusplus
extern "C" {
#endif


#define HF_USBCTRL_REBOOT               0x60
#define HF_USBCTRL_VERSION              0x61
#define HF_USBCTRL_CONFIG               0x62
#define HF_USBCTRL_STATUS               0x63
/* 0x64 - 0x66 reserved (used by loader) */
#define HF_USBCTRL_SERIAL               0x67

#define HF_USBCTRL_NAME                 0x70
#define HF_USBCTRL_FAN                  0x71
#define HF_USBCTRL_POWER                0x72
#define HF_USBCTRL_FAN_PARMS            0x73
#define HF_USBCTRL_ASIC_PARMS           0x74
#define HF_USBCTRL_VOLTAGE              0x75

#define HF_USBCTRL_ASIC_CTRL            0x90
#define HF_USBCTRL_MODE                 0x91

/* core control */
#define HF_USBCTRL_CORE_OVERVIEW        0xa0
#define HF_USBCTRL_CORE_ENABLE          0xa1
#define HF_USBCTRL_CORE_DISABLE         0xa2
#define HF_USBCTRL_CORE_CLEAR           0xa3
#define HF_USBCTRL_CORE_STATUS          0xa4
#define HF_USBCTRL_CORE_DIE_STATUS      0xa5
#define HF_USBCTRL_CORE_ASIC_STATUS     0xa6

/*
 * control/status still to be added
 */

/* debug control */
#define HF_USBCTRL_DEBUG_BUFFER         0xd0
#define HF_USBCTRL_DEBUG_STREAM         0xd1
#define HF_USBCTRL_DEBUG_CLI            0xd2



#define HF_USBCTRL_ASIC_CTRL_VALUE_RESET          0x8000
#define HF_USBCTRL_ASIC_CTRL_VALUE_PLL_BYPASS     0x4000



#ifdef __cplusplus
}
#endif

#endif /* _hf_usbctrl_h */

