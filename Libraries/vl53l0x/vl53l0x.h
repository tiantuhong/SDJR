#ifndef _VL53L0X_H
#define _VL53L0X_H

extern uint16_t vl53l0x_Results;
extern uint8_t vl53l0x_Mesure_En, vl53l0x_init_En, vl53l0x_data_valid;

//extern void vl53l0x_Init(void);
extern void vl53l0x_test(void);
extern void VL53L0X_IRQ(void);
#endif
