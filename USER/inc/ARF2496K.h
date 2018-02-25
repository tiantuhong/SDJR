#ifndef _ARF2496K_H
#define _ARF2496K_H

/*----------------------------------------------*
 * ºê¶¨Òå                                       *
 *----------------------------------------------*/
#define	PSB(Sta)		GPIO_WriteBit(GPIOE,GPIO_Pin_11,BitAction(Sta))
#define	SPIEN(Sta)		GPIO_WriteBit(GPIOE,GPIO_Pin_12,BitAction(Sta))
#define TRADAT(Sta)		GPIO_WriteBit(GPIOE,GPIO_Pin_13,BitAction(Sta))
#define TRACLK(Sta)		GPIO_WriteBit(GPIOE,GPIO_Pin_14,BitAction(Sta))
#define	TRRDY(Sta)		GPIO_WriteBit(GPIOE,GPIO_Pin_15,BitAction(Sta))

#define TRADAT_Port		GPIOE
#define TRADAT_Pin		GPIO_Pin_13

#define TRADAT_Out()	TRADAT_Port->CRH &= ~GPIO_CRH_CNF13;TRADAT_Port->CRH |= GPIO_CRH_MODE13
#define TRADAT_In()		TRADAT_Port->CRH |= GPIO_CRH_CNF13_0;TRADAT_Port->CRH &= ~GPIO_CRH_MODE13

extern void RF_Conf(void);

#endif
