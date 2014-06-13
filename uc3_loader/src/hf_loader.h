/* hf_loader.h */

/*
    Copyright (c) 2013, 2014 HashFast Technologies LLC
*/

#ifndef _hf_loader_h
#define _hf_loader_h

#ifdef __cplusplus
extern "C" {
#endif

#define HF_LOADER_USB_VENDOR_ID      0x297c
#define HF_LOADER_USB_PRODUCT_ID     0x8001

#define HF_LOADER_USB_REBOOT           0x60
#define HF_LOADER_USB_VERSION          0x61
#define HF_LOADER_USB_CONFIG           0x62
#define HF_LOADER_USB_STATUS           0x63
#define HF_LOADER_USB_START            0x64
#define HF_LOADER_USB_FINISH           0x65
#define HF_LOADER_USB_RESTART_ADDR     0x66
#define HF_LOADER_USB_SERIAL           0x67
#define HF_LOADER_USB_DEBUG            0xd0



#define HF_LOADER_CHAIN_UNCONFIGURED     0
#define HF_LOADER_CHAIN_NONE             1
#define HF_LOADER_CHAIN_MIDDLE           2
#define HF_LOADER_CHAIN_OPEN_UP          3
#define HF_LOADER_CHAIN_OPEN_DOWN        4
#define HF_LOADER_CHAIN_LOOPBACK         5


#define HF_LOADER_STATUS_OK               0
#define HF_LOADER_STATUS_BUSY             1
#define HF_LOADER_STATUS_ERR_STREAM       2
#define HF_LOADER_STATUS_ERR_TARGET       3
#define HF_LOADER_STATUS_ERR_ADDRESS      4
#define HF_LOADER_STATUS_ERR_PROGRAM      5
#define HF_LOADER_STATUS_ERR_VERIFY       6
#define HF_LOADER_STATUS_ERR_NOT_DONE     7
#define HF_LOADER_STATUS_ERR_FIRMWARE     8
#define HF_LOADER_STATUS_ERR_OVERRUN      9


#ifdef __cplusplus
}
#endif

#endif /* _hf_loader_h */


