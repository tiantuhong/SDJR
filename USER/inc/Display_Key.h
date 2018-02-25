#ifndef _DISPLAY_KEY_H
#define _DISPLAY_KEY_H

#define LED1(STATUS)	GPIO_WriteBit(GPIOE,GPIO_Pin_9,(BitAction)(STATUS))
#define LED2(STATUS)	GPIO_WriteBit(GPIOE,GPIO_Pin_10,(BitAction)(STATUS))
#define LED3(STATUS)	GPIO_WriteBit(GPIOE,GPIO_Pin_11,(BitAction)(STATUS))
#define LED4(STATUS)	GPIO_WriteBit(GPIOB,GPIO_Pin_13,(BitAction)(STATUS))
#define LED5(STATUS)	GPIO_WriteBit(GPIOB,GPIO_Pin_14,(BitAction)(STATUS))
#define LED6(STATUS)	GPIO_WriteBit(GPIOB,GPIO_Pin_15,(BitAction)(STATUS))

#define LED1_SW			GPIO_WriteBit(GPIOE,GPIO_Pin_9,(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOE,GPIO_Pin_9)))
#define LED2_SW			GPIO_WriteBit(GPIOE,GPIO_Pin_10,(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOE,GPIO_Pin_10)))
#define LED3_SW			GPIO_WriteBit(GPIOE,GPIO_Pin_11,(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOE,GPIO_Pin_11)))
#define LED4_SW			GPIO_WriteBit(GPIOB,GPIO_Pin_13,(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_13)))
#define LED5_SW			GPIO_WriteBit(GPIOB,GPIO_Pin_14,(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_14)))
#define LED6_SW			GPIO_WriteBit(GPIOB,GPIO_Pin_15,(BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_15)))

//#define LCDPOWERON		GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_RESET)
//#define LCDPOWEROFF		GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_SET)
//#define LCDBGON			GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET)
//#define LCDBGOFF		GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET)
#define BIAS 0x28		//1/2 偏置电压 4个公共口
#define SYSEN 0x01
#define LCDOFF 0x02
#define LCDON 0x03


#define B0	0x01
#define B1	0x02
#define B2	0x04
#define B3	0x08
#define B4	0x10
#define B5	0x20
#define B6	0x40
#define B7	0x80

#define		a_		B0
#define		b_		B2
#define		c_		B6
#define		d_		B4
#define		e_		B3
#define		f_		B1
#define		g_		B7
#define		h_		B5

#define		led_no	0
#define		led_all	0xff
#define		led_1	B0
#define		led_2	B1
#define		led_3	B2
#define		led_4	B3
#define		led_5	B4
#define		led_6	B5
#define		led_7	B6
#define		led_8	B7

#define		led_9	B0
#define		led_10	B1
#define		led_11	B2
#define		led_12	B3
#define		led_13	B4
#define		led_14	B5
#define		led_15	B6
#define		led_16	B7

#define		MI_00	( a_ + b_ + c_ + d_ + e_ + f_ )
#define		MI_01	( b_ + c_ )
#define		MI_02	( a_ + b_ + d_ + e_ + g_ )
#define		MI_03	( a_ + b_ + d_ + c_ + g_ )
#define		MI_04	( b_ + c_ + f_ + g_ )
#define		MI_05	( a_ + c_ + d_ + f_ + g_ )
#define		MI_06	( a_ + c_ + d_ + e_ + f_ + g_ )
#define		MI_07	( a_ + b_ + c_ )
#define		MI_08	( a_ + b_ + c_ + d_ + e_ + f_ + g_ )
#define		MI_09	( a_ + b_ + c_ + d_ + f_ + g_ )
#define		MI_0a	( a_ + b_ + c_ + e_ + f_ + g_ )
#define		MI_0b	( c_ + d_ + e_ + f_ + g_ )
#define		MI_0c	( a_ + d_ + e_ + f_ )
#define		MI_0d	( b_ + c_ + d_ + e_ + g_ )
#define		MI_0e	( a_ + d_ + e_ + f_ + g_ )
#define		MI_0f	( a_ + e_ + f_ + g_ )
#define		MI__	( g_ )

#define		MI_0u	( b_ + c_ + d_ + e_ + f_ )
#define		MI_0t	( d_ + e_ + f_ + g_ ) 
#define		MI_0o	( c_ + d_ + e_ + g_ )

#define CD_1628_com_1		0xC4
#define CD_1628_com_2		0xC6
#define CD_1628_com_3		0xCA
//#define CD_1628_com_4		0xCC
//#define CD_1628_com_5		0xC8
#define CD_1628_com_5		0xCC
#define CD_1628_com_4		0xC8
#define CD_1628_com_6		0xC2
#define CD_1628_com_7		0xC0

//#define	b_1628_Data_Out()	P0CR|=B2
//#define	b_1628_Data_In()	P0CR&=~B2
//#define b_1628_Data_c       P0_2
#define	b_1628_Data_High()	GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET)
#define	b_1628_Data_Low()	GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET)

//#define b_1628_Clk_Out()	P0CR|=B3
//#define b_1628_Clk_c        P0_3
#define	b_1628_Clk_High()	GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_SET)
#define	b_1628_Clk_Low()	GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_RESET)

//#define	b_1628_Stb_Out()	P0CR|=B4
//#define b_1628_Stb_c        P0_4
#define	b_1628_Stb_High()	GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET)
#define	b_1628_Stb_Low()	GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET)

#define b_1628_Power_On()	GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_SET)
#define b_1628_Power_Off()	GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET)

extern FunctionalState KeyScanEn;
extern FunctionalState DisplayScanEn;
extern uint8_t Key_Code;

extern void Key_Scan(void);
extern void Display(void);
extern void Display_Conf(void);

#endif

