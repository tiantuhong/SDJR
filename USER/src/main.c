
#include "stm32f10x.h"


int main(void)
{		
    Device_Init();
    
	while(1)
    {	
        UART_ResponseMaster();
        UART2_SendData();
        handshakepresec();
        Uart_RcvData_Proc();
        Uart2_RcvData_Proc();
        CalSpeed();
        MotorStartupCtrl();
        AccCtrl();
        Robit_RouteMoving();
        MotorRunCtrl();
        MotorBlockCtrl();
        Key_Scan();
        Display();
        ADC_Deal();	
        CarpetCheck();
        BatterySocCheck();	
        Actuator_Deal();       				
        MotorStopCtrl();		
        RobitMode_Ctrl();
        SignalCheck();
        Ultra_Check();
        PathPlan();
        
        ICM_Init();
        Gyro_Calibration();
//        ChargeManageInit();
        vl53l0x_test();
//        Voice_Deal();
        I2C_Deal();     
    }
}
#ifdef  USE_FULL_ASSERT                

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
	
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
