#ifndef _ADC_H
#define _ADC_H
/*----------------------------------------------*
 * ∫Í∂®“Â                                       *
 *----------------------------------------------*/
#define ADC_Channel_Num		10
#define ADC1_DR_Address    ((u32)0x4001244C)
#define ADC_DELT			150


extern uint16_t ADC_ConvertedValue_Val[ADC_Channel_Num];
extern FunctionalState ADCScanEn,ADCDataValid[ADC_Channel_Num];
extern uint8_t ADC_Ready;

extern void ADC_Conf(void);
extern void ADC_Deal(void);
extern uint16_t Get_DataDiffABS(uint16_t Num1 , uint16_t Num2);

#endif
