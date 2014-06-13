/* ir3566b.h */

#ifndef _ir3566b_h
#define _ir3566b_h


#define IR3566B_REG_I2C                        0x12
#define IR3566B_REG_I2C_ENABLE                     0x80
#define IR3566B_REG_I2C_ADDR_POS                      0
#define IR3566B_REG_I2C_ADDR_MASK                  0x7f
#define IR3566B_REG_L1_VBOOT                   0x17
#define IR3566B_REG_PMBUS                      0x27
#define IR3566B_REG_PMBUS_ADDR_MASK                0xf0
#define IR3566B_REG_PMBUS_ADDR_POS                    4
#define IR3566B_REG_PMBUS_DELAY_MODE               0x08
#define IR3566B_REG_PMBUS_DELAY_TIME_MASK          0x07
#define IR3566B_REG_PMBUS_DELAY_TIME_POS              0
// XXX come up with better name
#define IR3566B_REG_5D                         0x5d
#define IR3566B_REG_5D_VAUX_ENABLE                 0x40
#define IR3566B_REG_LOOP_1_MANUAL_VID          0x6a
#define IR3566B_REG_VIN_SUPPLY                 0x98
#define IR3566B_REG_L1_VOUT                    0x9a
#define IR3566B_REG_TEMP1                      0x9e
#define IR3566B_REG_TEMP2                      0x9f
#define IR3566B_L1_IIN                         0xcb


#endif /* _ir3566b_h */

