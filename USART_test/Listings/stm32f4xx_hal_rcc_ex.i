#line 1 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"







































  

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"




































  

 







 
#line 1 ".\\Inc\\stm32f4xx_hal_conf.h"





































  

 







 
 

 


 
#line 100 ".\\Inc\\stm32f4xx_hal_conf.h"

 




 












 






 







 












 





 

 


      
#line 165 ".\\Inc\\stm32f4xx_hal_conf.h"

 



 
 

 

 

 
#line 184 ".\\Inc\\stm32f4xx_hal_conf.h"

    





 

  

  

 





 



 
#line 218 ".\\Inc\\stm32f4xx_hal_conf.h"




  
 




 










 




 



 


 

#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"



































 

 







 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"




































 

 







 
#line 1 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"













































 



 



 
    






   


 
  


 






 
#line 110 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"
   


 
#line 122 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 
#line 134 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 



 

#line 1 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"









































 



 



 
    






  



 



 








 
   


 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,        
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,        
  DMA1_Stream7_IRQn           = 47,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  FPU_IRQn                    = 81,       
  SPI4_IRQn                   = 84        
} IRQn_Type;



 

#line 1 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"
 




 

























 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"

















 




 



 

 













#line 120 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"



 
#line 135 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"

#line 209 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"

#line 1 ".\\Drivers\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 ".\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 ".\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 ".\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"











 


#line 54 ".\\Drivers\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 ".\\Drivers\\CMSIS\\Include\\core_cmInstr.h"

   

#line 211 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"
#line 1 ".\\Drivers\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 ".\\Drivers\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 ".\\Drivers\\CMSIS\\Include\\core_cmFunc.h"

 

#line 212 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"
#line 1 ".\\Drivers\\CMSIS\\Include\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 ".\\Drivers\\CMSIS\\Include\\core_cmSimd.h"

 
#line 88 ".\\Drivers\\CMSIS\\Include\\core_cmSimd.h"

 






#line 213 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"
















 
#line 256 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"

 






 
#line 272 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1541 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"

#line 1550 ".\\Drivers\\CMSIS\\Include\\core_cm4.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4U)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 160 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
#line 1 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\system_stm32f4xx.h"



































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           




 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 161 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
#line 162 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;           
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;




 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
  volatile uint32_t OPTCR1;    
} FLASH_TypeDef;



 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint32_t BSRR;      
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED[2];    
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t OAR1;        
  volatile uint32_t OAR2;        
  volatile uint32_t DR;          
  volatile uint32_t SR1;         
  volatile uint32_t SR2;         
  volatile uint32_t CCR;         
  volatile uint32_t TRISE;       
  volatile uint32_t FLTR;        
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
  uint32_t      RESERVED7[1];   
  volatile uint32_t DCKCFGR;        
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;




 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t CRCPR;       
  volatile uint32_t RXCRCR;      
  volatile uint32_t TXCRCR;      
  volatile uint32_t I2SCFGR;     
  volatile uint32_t I2SPR;       
} SPI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR;           
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 
typedef struct
{
  volatile uint32_t GOTGCTL;               
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  uint32_t  Reserved40[48];            
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
}
USB_OTG_GlobalTypeDef;





 
typedef struct 
{
  volatile uint32_t DCFG;          
  volatile uint32_t DCTL;          
  volatile uint32_t DSTS;          
  uint32_t Reserved0C;         
  volatile uint32_t DIEPMSK;       
  volatile uint32_t DOEPMSK;       
  volatile uint32_t DAINT;         
  volatile uint32_t DAINTMSK;      
  uint32_t  Reserved20;        
  uint32_t Reserved9;          
  volatile uint32_t DVBUSDIS;      
  volatile uint32_t DVBUSPULSE;    
  volatile uint32_t DTHRCTL;       
  volatile uint32_t DIEPEMPMSK;    
  volatile uint32_t DEACHINT;      
  volatile uint32_t DEACHMSK;        
  uint32_t Reserved40;         
  volatile uint32_t DINEP1MSK;     
  uint32_t  Reserved44[15];    
  volatile uint32_t DOUTEP1MSK;       
}
USB_OTG_DeviceTypeDef;




 
typedef struct 
{
  volatile uint32_t DIEPCTL;         
  uint32_t Reserved04;           
  volatile uint32_t DIEPINT;         
  uint32_t Reserved0C;           
  volatile uint32_t DIEPTSIZ;        
  volatile uint32_t DIEPDMA;         
  volatile uint32_t DTXFSTS;         
  uint32_t Reserved18;            
}
USB_OTG_INEndpointTypeDef;




 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
}
USB_OTG_OUTEndpointTypeDef;




 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;              
  volatile uint32_t HFNUM;             
  uint32_t Reserved40C;            
  volatile uint32_t HPTXSTS;           
  volatile uint32_t HAINT;             
  volatile uint32_t HAINTMSK;          
}
USB_OTG_HostTypeDef;




 
typedef struct
{
  volatile uint32_t HCCHAR;
  volatile uint32_t HCSPLT;
  volatile uint32_t HCINT;
  volatile uint32_t HCINTMSK;
  volatile uint32_t HCTSIZ;
  volatile uint32_t HCDMA;
  uint32_t Reserved[2];
}
USB_OTG_HostChannelTypeDef;



 
#line 672 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 





 
#line 701 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 716 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 745 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


#line 764 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
  


   
#line 828 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 861 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 887 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
  
 
#line 913 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 951 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 993 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 


 


 


 


 
#line 1042 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1080 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1118 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1147 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 


 


 



 
#line 1183 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 1210 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 
 
 
 
 
 



 



 


 
 
 
 
 

 
 
 
 
 
 
#line 1277 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 1299 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1310 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1332 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1354 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1376 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1398 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 
 
 
 
 
 
#line 1429 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1454 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1479 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1504 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1529 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1554 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 
#line 1570 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 1578 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1587 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1604 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






#line 1637 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
                                             
 
#line 1652 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 
































































 
#line 1740 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
































































 
































































 
#line 1888 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 1905 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1923 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 1940 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1974 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1993 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 
#line 2014 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2023 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 



#line 2044 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 



 


 
#line 2069 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2079 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 


 



 
 
 
 
 
 


 





 


 




 
 
 
 
 
 











 
#line 2140 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2149 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 2161 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
 
 
 
 
 



#line 2180 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2190 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2199 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2208 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2219 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"















 
 








 








 






#line 2269 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 











 











 
#line 2301 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




















 
#line 2330 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2338 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2345 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 2358 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 

 
#line 2377 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2390 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 2405 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 

 
#line 2424 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2437 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2452 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 

 
#line 2472 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2485 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 











 
#line 2509 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 





 
#line 2527 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"






 


 
 
 
 
 
 
#line 2569 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2599 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2627 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2645 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 


 



 
#line 2698 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2740 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 



 
#line 2779 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2799 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 2817 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2839 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2847 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2855 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 




 
 
 
 
 
 




 












 


 






#line 2958 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 3028 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3043 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3069 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 
 
 
 
 
 









#line 3101 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3110 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3121 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 


 


 





















 




 
 
 
 
 
   





 


 






  
#line 3189 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3199 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3209 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3219 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






  
#line 3234 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3244 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3254 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3264 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 




           


  
#line 3281 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3291 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3301 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3311 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






  
#line 3326 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3336 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3346 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



  
#line 3356 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

   



 
 
 
 
 
 
















 









#line 3401 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

























 
#line 3444 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3458 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3468 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




























 





















 




























 





















 
#line 3587 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 


 


 


 


 


 


 
#line 3622 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





#line 3633 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3641 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3648 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 3659 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 
 
 
 
 
 
#line 3677 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 



 
#line 3701 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3710 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 3730 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3741 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
 
 
 
 
 
#line 3758 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 3766 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 3778 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 3786 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"




 





 



 
 
 
 
 
 



 









 
#line 3837 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 


 






 
 
 
 
 
 
#line 3863 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






 






#line 3886 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"









 




 
#line 3907 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 





#line 3923 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 



 








 


#line 3951 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 3961 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3980 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






#line 3996 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4006 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


#line 4019 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4029 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 4041 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4069 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4097 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 


 





 



 

#line 4123 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"























 

#line 4154 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"























 


 


 





 


 


#line 4204 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4213 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



#line 4229 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4241 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 



 
#line 4256 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 


 
#line 4274 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4285 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
















 
#line 4313 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 










#line 4340 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


#line 4351 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"









#line 4371 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 4382 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4391 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 4410 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4423 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4436 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 4449 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 


 


 



 

#line 4479 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4487 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 








 






  



 



 
 
 

 
 


 
#line 4537 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4545 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 



 





 


 





 






 
#line 4586 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4596 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4604 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






 






 


 






 






 






 






 






 
#line 4664 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4672 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 






 




 
#line 4730 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






 




 




 




 




 





 


 


 


 


 


 







  



 



 









 
#line 163 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"
#line 188 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 



  
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum 
{
  ERROR = 0U, 
  SUCCESS = !ERROR
} ErrorStatus;



 




 



















 

#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"




































  

 
#line 264 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

 
#line 246 ".\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"









 



 
  



 
#line 49 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




































 

 







 
 
 



 








 
  


 
#line 107 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 115 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


  
  




    
   


 
#line 146 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 202 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 
  





 



 

#line 238 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 261 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
  



  
  
  


 



 
  
#line 347 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 
  
#line 364 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  



 
#line 383 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
  




 



 






















#line 428 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 435 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"










 



 
#line 459 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
   
#line 468 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 491 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 
   


 










 



 





                                              















                                                                      



                                                        


 



 






 



 

 
#line 636 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 












 
  


 
  









#line 673 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
















 

  


 















 

  


 
#line 727 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 











 
  


 


  
#line 780 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 790 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 809 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 



 























 

  


 








 



 




 



 
#line 887 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 

#line 904 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 916 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 
#line 947 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 











   
  
#line 994 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 

 



 



   



  
#line 1022 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 













 
  


 
#line 1057 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 1071 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 

 



 






 

 



 
#line 1108 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1116 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




#line 1130 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
 

   
  


 





 



 



   



 






 
   


  



 
  


  



   
   
  


 
  


 

 



 





   
  


 
#line 1219 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"









 

   


 
#line 1247 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1268 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1279 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1288 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1302 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1311 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 







 
   


 
#line 1347 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1362 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


#line 1388 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 1555 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



#line 1565 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 

#line 1579 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




  


 



 

#line 1602 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 
  
#line 1626 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 
  





 



 
  













 




 




 




 







 
  
  


 
#line 1700 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




 
#line 1744 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1758 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


   
  
  


 
  






#line 2273 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2422 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 



#line 2449 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2472 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2589 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2606 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2621 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
























#line 2667 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



#line 2678 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2711 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2729 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"












#line 2747 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2764 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 
  


 
  




#line 2813 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2828 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




#line 2864 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 




#line 2894 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 

#line 2908 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 
#line 2929 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 
  


 













 




 











 



 












#line 3002 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3011 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3020 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 








#line 3053 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 



 
  
#line 3070 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 



 



 
#line 3103 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 
  


 







 

#line 50 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 51 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"

 



   
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;



 
typedef enum 
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U  
} HAL_LockTypeDef;

 



























 


#line 120 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"







#line 135 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"


 
#line 157 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"




  









 


#line 190 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"



  



 


#line 207 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

 
 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       
 

  uint32_t PLLN;       

 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 
#line 92 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
}RCC_PLLInitTypeDef;

#line 193 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 219 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 283 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 368 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"





 
typedef struct
{




                                
  uint32_t PLLI2SN;    


 

  uint32_t PLLI2SR;    

 

}RCC_PLLI2SInitTypeDef;
 


 
typedef struct
{
  uint32_t PeriphClockSelection; 
 

  RCC_PLLI2SInitTypeDef PLLI2S;  
 

  uint32_t RTCClockSelection;      
 

  uint8_t TIMPresSelection;        
 

}RCC_PeriphCLKInitTypeDef;



  

 


 



 
 
#line 436 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 446 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 463 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 
    
 
#line 477 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 489 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 501 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 


 







 




 






 
#line 532 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 



 
#line 546 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 



 
#line 559 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 

#line 584 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      
#line 613 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 706 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 785 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 819 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"








 




 




#line 850 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"








 






 




#line 882 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"



 
     
 


 
 
#line 1935 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 2803 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 







 
#line 2849 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"








 







 













  
  






 



                                        



 







 




   
  






 
#line 2957 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 







 













  
  






 
#line 3011 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"






 
  






   





 



   











 




 







 




 
#line 3072 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 3079 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 




 











 




  




 








 
#line 3124 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 3131 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 








 





 








 













 








 









 

 

 
#line 3459 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 3845 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 4649 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 5389 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 5435 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"




























 
#line 5470 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 
                             
 








 



#line 5552 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"














 





#line 5597 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 5620 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 5710 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 5731 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 5743 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 











 


                                 
#line 5793 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 5958 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      
#line 6007 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6138 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6193 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      

















      





 

#line 6243 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6256 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"



 

 


 



 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);












  



 
 
 
 


 




 
   
#line 6308 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"






 






 





 
#line 6334 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

 





 


      



      
#line 6358 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6368 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"




 



 

 


 


 
#line 6395 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      






























#line 6445 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6453 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6473 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6525 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6546 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6580 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"






      













 



 



  



   






 
#line 52 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"



 



 

 


 
   


 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 

  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  
 

  uint32_t LSIState;             
 

  RCC_PLLInitTypeDef PLL;         
}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 







 



 





 



 





 



 






 



 




 



 





 



 






 



 




 



 






 
  


 






 



 






 



 
#line 250 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


  
  


 







  



 
#line 301 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



 




 



 






 



 







 



 
#line 347 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 
  








 
 





 


 
#line 378 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


  



 

 


 







 
#line 440 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 447 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 464 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 471 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


   
  






 
#line 531 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 539 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 557 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 565 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


   
  






 
#line 632 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 641 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 660 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 669 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


   
  



   
#line 684 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 692 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 




 
#line 708 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 717 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 




 
#line 734 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 744 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 








 
#line 762 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 769 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 








 
#line 788 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 796 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 








 
#line 816 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 825 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



  
                                      














 









 




 



  








 




 



  





















 



 



 


















 




 



 



 























 


                                                   








 




 



 







 










 










 



 



 








 









      







 



 



  
  














 




















 




 
  



 











 












 













 













 




 



















 





 
     


 

 
 

 



                              
 
void HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);


 



 
 
void     HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void HAL_RCC_NMI_IRQHandler(void);

  
void HAL_RCC_CSSCallback(void);



 



 

 
 
 


 




 

 
 



 


 



 
 



 



 
 




 


 


 


 











 



 

 


 
    


   






















#line 1373 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"
 






























 



 



  



 







 
#line 255 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"



 



  

 


 



  
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 

  uint32_t Pull;      
 

  uint32_t Speed;     
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


 

 



  



 
#line 121 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"




 










  







    



 





 




 






 

 


   





 
  


 

 


 






 







 







 







 







 



 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"



 



  

 
 


 
  


 

 
#line 184 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 299 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 405 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 501 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 

 



  








  





  






  






  






  







  





  






  




  







  




  




  


 

 
#line 699 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 
#line 790 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 
#line 864 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 
#line 984 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1107 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 


  



 

 


 


 

 


 


 

 
 
 


 


 

 


 


 
#line 1160 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

#line 1174 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

#line 1181 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

#line 1189 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

#line 1199 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"



 



   
 
#line 1233 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1260 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1284 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1307 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 

 
#line 1325 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"


 
 




 
#line 1352 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1384 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1413 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 



 



  



 

 


 



 



  



  
  






 
#line 233 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"

 


 



 
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 



 
 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



  



  
 
 
 


 



 

 


 
#line 300 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"


 

 


 



 



  



 







 
#line 259 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"



 



  

 




 
   


 
typedef struct
{
  uint32_t Channel;              
 

  uint32_t Direction;            

 

  uint32_t PeriphInc;            
 

  uint32_t MemInc;               
 

  uint32_t PeriphDataAlignment;  
 

  uint32_t MemDataAlignment;     
 

  uint32_t Mode;                 


 

  uint32_t Priority;             
 

  uint32_t FIFOMode;             


 

  uint32_t FIFOThreshold;        
 

  uint32_t MemBurst;             



 

  uint32_t PeriphBurst;          



 
}DMA_InitTypeDef;




 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
  HAL_DMA_STATE_ERROR             = 0x04U,   
  HAL_DMA_STATE_ABORT             = 0x05U,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00U,     
  HAL_DMA_HALF_TRANSFER      = 0x01U      
}HAL_DMA_LevelCompleteTypeDef;



 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,     
  HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,     
  HAL_DMA_XFER_M1CPLT_CB_ID        = 0x02U,     
  HAL_DMA_XFER_M1HALFCPLT_CB_ID    = 0x03U,     
  HAL_DMA_XFER_ERROR_CB_ID         = 0x04U,     
  HAL_DMA_XFER_ABORT_CB_ID         = 0x05U,     
  HAL_DMA_XFER_ALL_CB_ID           = 0x06U      
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                     

  DMA_InitTypeDef            Init;                                                           

  HAL_LockTypeDef            Lock;                                                            

  volatile HAL_DMA_StateTypeDef  State;                                                         

  void                       *Parent;                                                        

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);      

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);  

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);     
  
  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);       

 volatile uint32_t               ErrorCode;                                                     
  
 uint32_t                    StreamBaseAddress;                                             

 uint32_t                    StreamIndex;                                                   
 
}DMA_HandleTypeDef;



 

 




 




  
#line 212 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"


 




  
#line 228 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"


 




  





 
        



  




  




  




 




  





  




 





 




  





 




 






  




 




  




 






  




  






  




  






 




 







 




  
#line 391 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"


 



 
 
 




 













 






 






 


 





 
#line 456 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





       
#line 476 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





 
#line 496 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





 
#line 516 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





 
#line 536 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"













 

















 
















 














 














 




















 







 



 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma_ex.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma_ex.h"



 



  

 



 
   


  
typedef enum
{
  MEMORY0      = 0x00U,     
  MEMORY1      = 0x01U      
}HAL_DMA_MemoryTypeDef;



 

 



 




 

 
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);



 


 
         
 



 


 



 



 







 
#line 649 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"

 




 




 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma); 
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);


 




 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_CleanCallbacks(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



  




 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


  


  
 



 


  

 



 
#line 719 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"

















































  

 



 


 



  



 







 
#line 263 ".\\Inc\\stm32f4xx_hal_conf.h"

   
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"



 



  
 


 





 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
          
  uint8_t                TypeExtField;          
                  
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
#line 118 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 



 





 




 






 



 




 



 




 



 




 



 




 



 




 



 





 



 
#line 230 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 
   


 
#line 243 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 



 
#line 258 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 




 


 

 


 
  


 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



 
 

void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);

uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


 



 

 
 
 
 


 



































#line 361 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"

#line 370 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"

#line 399 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"






                                                                             
                                                                                   
    



 





 
static __inline void HAL_MPU_Disable(void)
{
   
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);
  
   
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL  &= ~(1UL );
}











 
static __inline void HAL_MPU_Enable(uint32_t MPU_Control)
{
   
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL   = MPU_Control | (1UL );
  
   
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);
}




 



  



 
  





 

 
#line 267 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"



 



  

 


 













 
typedef struct
{
  uint32_t ClockPrescaler;        

 
  uint32_t Resolution;            
 
  uint32_t DataAlign;             

 
  uint32_t ScanConvMode;          





 
  uint32_t EOCSelection;          





 
  uint32_t ContinuousConvMode;    

 
  uint32_t NbrOfConversion;       

 
  uint32_t DiscontinuousConvMode; 


 
  uint32_t NbrOfDiscConversion;   

 
  uint32_t ExternalTrigConv;      


 
  uint32_t ExternalTrigConvEdge;  

 
  uint32_t DMAContinuousRequests; 



 
}ADC_InitTypeDef;







  
typedef struct 
{
  uint32_t Channel;                
 
  uint32_t Rank;                   
 
  uint32_t SamplingTime;           







 
  uint32_t Offset;                  
}ADC_ChannelConfTypeDef;



  
typedef struct
{
  uint32_t WatchdogMode;      
 
  uint32_t HighThreshold;     
      
  uint32_t LowThreshold;      
 
  uint32_t Channel;           

       
  uint32_t ITMode;            

 
  uint32_t WatchdogNumber;     
}ADC_AnalogWDGConfTypeDef;



  
 





 




 





 




 




 





  
typedef struct
{
  ADC_TypeDef                   *Instance;                    

  ADC_InitTypeDef               Init;                         

  volatile uint32_t                 NbrOfCurrentConversionRank;   

  DMA_HandleTypeDef             *DMA_Handle;                  

  HAL_LockTypeDef               Lock;                         

  volatile uint32_t                 State;                        

  volatile uint32_t                 ErrorCode;                    
}ADC_HandleTypeDef;


 

 


 



 







 




  






  



  
#line 276 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  



  






  



  






  



 
 
 
#line 324 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  



  




  



  
#line 359 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"





  



  
#line 377 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  

  

  





  



  




 



  
#line 410 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  
    


  






  
    


  
#line 434 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  



  





 



  

 


 




 






 






 







 







 






 







 







 




 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"



 



  

 


 
   











 
typedef struct 
{
  uint32_t InjectedChannel;               

 
  uint32_t InjectedRank;                  

 
  uint32_t InjectedSamplingTime;          







 
  uint32_t InjectedOffset;                


 
  uint32_t InjectedNbrOfConversion;       



 
  uint32_t InjectedDiscontinuousConvMode; 





 
  uint32_t AutoInjectedConv;              






 
  uint32_t ExternalTrigInjecConv;         






 
  uint32_t ExternalTrigInjecConvEdge;     



 
}ADC_InjectionConfTypeDef; 



  
typedef struct
{
  uint32_t Mode;              
 
  uint32_t DMAAccessMode;     
 
  uint32_t TwoSamplingDelay;  
 
}ADC_MultiModeTypeDef;



 

 


 



  
#line 168 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"


  



  






  



  






  



  
#line 214 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"


  



  






 



 
#line 238 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"








  




  

 


 
#line 271 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"


  

 


 



 

 
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef* hadc, uint32_t InjectedRank);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef* hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);

 
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc,ADC_InjectionConfTypeDef* sConfigInjected);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef* hadc, ADC_MultiModeTypeDef* multimode);



  



 
 
 
 


 



 

 


 
#line 328 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"







#line 375 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"







 




 

 


 



 



  



 








 
#line 522 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"

 


 



 
 
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);


 



 
 
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);

void HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);


 



 
 
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);


 



 
 
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc);


 



 
 
 
 


 
 
 
 

 
 
 



 

 



 

 





 









 








 









 






 



    
#line 733 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"

#line 742 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"





 







 







 







 







 







 






 






 






 






 






 






 




 

 


 



 



  



 








 
#line 271 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_can.h"



































 

 







#line 761 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_can.h"



 



 








 
#line 275 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_crc.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_crc.h"



 




 

 


 



 
typedef enum
{
  HAL_CRC_STATE_RESET     = 0x00U,   
  HAL_CRC_STATE_READY     = 0x01U,   
  HAL_CRC_STATE_BUSY      = 0x02U,   
  HAL_CRC_STATE_TIMEOUT   = 0x03U,   
  HAL_CRC_STATE_ERROR     = 0x04U    

}HAL_CRC_StateTypeDef;


 



  
typedef struct
{
  CRC_TypeDef                 *Instance;   

  HAL_LockTypeDef             Lock;        

  volatile HAL_CRC_StateTypeDef   State;       

}CRC_HandleTypeDef;


 



  

 
 


 




 






 







 






 



  

 


 



 
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit (CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);


  



 
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);


  



 
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);


  



 
 


 



  

 


 



  
          
 


 



  

 


 



  

 


 



 

 


 



 

 


 



 



  



  







 
#line 279 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cryp.h"



































  

 







#line 521 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cryp.h"



  



  
  






 
#line 283 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma2d.h"



































 

 







#line 570 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma2d.h"







 
#line 287 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dac.h"



































 

 







#line 398 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dac.h"



 



 
  






 
#line 291 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dcmi.h"



































 

 







#line 522 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dcmi.h"



 
    


 







 
#line 295 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_eth.h"



































  

 







#line 2175 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_eth.h"
  







 
#line 299 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"



 



  

 


 
 


 
typedef enum 
{
  FLASH_PROC_NONE = 0U, 
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;



 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;    
  
  volatile uint32_t               NbSectorsToErase;    
  
  volatile uint8_t                VoltageForErase;     
  
  volatile uint32_t               Sector;              
  
  volatile uint32_t               Bank;                
  
  volatile uint32_t               Address;             
  
  HAL_LockTypeDef             Lock;                

  volatile uint32_t               ErrorCode;           

}FLASH_ProcessTypeDef;



 

 


   



  
#line 115 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"


 
  


  






 




  
#line 142 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"


 
  



  




   



 







  



  







  



  
  
 


 





  






  





  





  





  





  





  





  






 








 










   









   















 














 



 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t TypeErase;   
 

  uint32_t Banks;       
 

  uint32_t Sector;      
 

  uint32_t NbSectors;   
 

  uint32_t VoltageRange;
 

} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;   
 

  uint32_t WRPState;     
 

  uint32_t WRPSector;         
 

  uint32_t Banks;        
         

  uint32_t RDPLevel;     
 

  uint32_t BORLevel;     
 

  uint8_t  USERConfig;    

} FLASH_OBProgramInitTypeDef;



 





typedef struct
{
  uint32_t OptionType;     
 

  uint32_t PCROPState;     
 



  uint16_t Sectors;        
 



#line 148 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
}FLASH_AdvOBProgramInitTypeDef;



 

 



 



  




 
  


  






 
  


  




 
  


  






 
  


 






  
  


  




  
  


  




  




  




     



   






 








  




 







  













 



 
   
#line 309 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  




     
#line 326 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 



  
  



 
#line 342 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

#line 349 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


  
    


 





#line 367 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


  



 
    
#line 402 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
   

  
#line 420 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  
#line 431 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  
#line 441 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 454 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 



  



 
   
#line 492 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  
#line 511 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 523 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 
 
 
#line 534 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 548 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 


 
  


 
    
#line 585 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 597 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 608 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 623 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 



 
  


 







 



 
#line 654 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


 



  
  
 

 


 



 
 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);






HAL_StatusTypeDef HAL_FLASHEx_AdvOBProgram (FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
void              HAL_FLASHEx_AdvOBGetConfig(FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
HAL_StatusTypeDef HAL_FLASHEx_OB_SelectPCROP(void);
HAL_StatusTypeDef HAL_FLASHEx_OB_DeSelectPCROP(void);










 



 
 
 
 


 
  




  





  




  




 






  






 

 


 



 



























#line 790 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"







#line 803 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
  
#line 823 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

#line 838 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"







#line 852 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 
#line 867 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

#line 877 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"













#line 896 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"



  




















   






















#line 952 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


 



 

 


 
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
void FLASH_FlushCaches(void);


  



  



 







 
#line 312 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ramfunc.h"



































  

 



#line 93 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ramfunc.h"




 
#line 313 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"

 


 


 
 
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void HAL_FLASH_IRQHandler(void);
  
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 



 
 
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
 
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);


 



 
 
uint32_t HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);


 



  
 
 


 



 
 


 



  



  



  



  



  




 

 


 



 






 



 

 


 



 



  



 







 
#line 303 ".\\Inc\\stm32f4xx_hal_conf.h"

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sram.h"



































  

 







 












 

#line 198 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sram.h"


 






 
#line 307 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_nor.h"



































  

 







 












 



  

#line 291 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_nor.h"


  



 







 
#line 311 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_nand.h"



































 

 







 












 



  

#line 300 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_nand.h"
    


 


  



  







 
#line 315 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pccard.h"



































  

 







 










 

#line 254 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pccard.h"




 
  






 
#line 319 ".\\Inc\\stm32f4xx_hal_conf.h"

  
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sdram.h"



































  

 







#line 186 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sdram.h"



 







 
#line 323 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_hash.h"



































 

 







#line 436 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_hash.h"


  



 
  







 
#line 327 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"



 



 

 


 
   


 
typedef struct
{
  uint32_t ClockSpeed;       
 

  uint32_t DutyCycle;        
 

  uint32_t OwnAddress1;      
 

  uint32_t AddressingMode;   
 

  uint32_t DualAddressMode;  
 

  uint32_t OwnAddress2;      
 

  uint32_t GeneralCallMode;  
 

  uint32_t NoStretchMode;    
 

}I2C_InitTypeDef;


























 
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,    
  HAL_I2C_STATE_READY             = 0x20U,    
  HAL_I2C_STATE_BUSY              = 0x24U,    
  HAL_I2C_STATE_BUSY_TX           = 0x21U,    
  HAL_I2C_STATE_BUSY_RX           = 0x22U,    
  HAL_I2C_STATE_LISTEN            = 0x28U,    
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   
 
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   
 
  HAL_I2C_STATE_ABORT             = 0x60U,    
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,    
  HAL_I2C_STATE_ERROR             = 0xE0U     

}HAL_I2C_StateTypeDef;

















 
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,    
  HAL_I2C_MODE_MASTER             = 0x10U,    
  HAL_I2C_MODE_SLAVE              = 0x20U,    
  HAL_I2C_MODE_MEM                = 0x40U     

}HAL_I2C_ModeTypeDef;



 
typedef struct
{
  I2C_TypeDef                *Instance;       
                                             
  I2C_InitTypeDef            Init;            
                                             
  uint8_t                    *pBuffPtr;       
                                             
  uint16_t                   XferSize;        
                                             
  volatile uint16_t              XferCount;       
                                             
  volatile uint32_t              XferOptions;     
                                             
  volatile uint32_t              PreviousState;  
 
                                             
  DMA_HandleTypeDef          *hdmatx;         
                                             
  DMA_HandleTypeDef          *hdmarx;         
                                             
  HAL_LockTypeDef            Lock;            
                                             
  volatile HAL_I2C_StateTypeDef  State;           
                                             
  volatile HAL_I2C_ModeTypeDef   Mode;            
                                             
  volatile uint32_t              ErrorCode;       

  volatile uint32_t              Devaddress;      

  volatile uint32_t              Memaddress;      

  volatile uint32_t              MemaddSize;      

  volatile uint32_t              EventCount;      
	
}I2C_HandleTypeDef;



 

 


 




  
#line 225 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 






 



 





 



 
#line 337 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"


 



 

 


 





 











 












 






























 
















 






 
#line 442 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"





 
#line 455 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"
    




 






 




 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c_ex.h"



































  

 










 
#line 51 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c_ex.h"



 



  

  
 


 



 




 
  


  
  
 
 


 



 
 
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);


 



  
 
 
 


 



 

 


 





 



  



 











 
#line 476 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit (I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);


 



 
 
 
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

 
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);


 



 
 
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);



 



 
 
 
 


 



 

 


 
    
#line 586 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"













 
#line 619 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"


 



 

 


 



 



 



 








 
#line 331 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s.h"



 



  

 


 
  


 
typedef struct
{
  uint32_t Mode;         
 

  uint32_t Standard;     
 

  uint32_t DataFormat;   
 

  uint32_t MCLKOutput;   
 

  uint32_t AudioFreq;    
 

  uint32_t CPOL;         
 

  uint32_t ClockSource;  
 

  uint32_t FullDuplexMode;  
 

}I2S_InitTypeDef;



  
typedef enum
{
  HAL_I2S_STATE_RESET      = 0x00U,   
  HAL_I2S_STATE_READY      = 0x01U,   
  HAL_I2S_STATE_BUSY       = 0x02U,   
  HAL_I2S_STATE_BUSY_TX    = 0x12U,   
  HAL_I2S_STATE_BUSY_RX    = 0x22U,   
  HAL_I2S_STATE_BUSY_TX_RX = 0x32U,   
  HAL_I2S_STATE_TIMEOUT    = 0x03U,   
  HAL_I2S_STATE_ERROR      = 0x04U    

}HAL_I2S_StateTypeDef;



 
typedef struct
{
  SPI_TypeDef                *Instance;     

  I2S_InitTypeDef            Init;          
  
  uint16_t                   *pTxBuffPtr;   
  
  volatile uint16_t              TxXferSize;    
  
  volatile uint16_t              TxXferCount;   
  
  uint16_t                   *pRxBuffPtr;   
  
  volatile uint16_t              RxXferSize;    
  
  volatile uint16_t              RxXferCount;   

  DMA_HandleTypeDef          *hdmatx;       

  DMA_HandleTypeDef          *hdmarx;       
  
  volatile HAL_LockTypeDef       Lock;          
  
  volatile HAL_I2S_StateTypeDef  State;         
  
  volatile uint32_t              ErrorCode;     
  
}I2S_HandleTypeDef;


 

 


 




  
#line 161 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s.h"


 



 






 
  


 







 
  


 






 



 




 



 
#line 221 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s.h"


 



 




 



 




 



 





 



 











 



  

 


 




 





 











   


 









 














 





 
#line 341 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s.h"
    



 
#line 352 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s.h"


 
  
 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s_ex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s_ex.h"



 



  

 


  


 

 


 



 

#line 82 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s_ex.h"
   
#line 89 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s_ex.h"








 



  

 


 



  

 


 



 

 
 
HAL_StatusTypeDef HAL_I2SEx_TransmitReceive(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData, uint16_t Size, uint32_t Timeout);
 
HAL_StatusTypeDef HAL_I2SEx_TransmitReceive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData, uint16_t Size);
 
HAL_StatusTypeDef HAL_I2SEx_TransmitReceive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData, uint16_t Size);


  



 
 
 
 


 


 

 


 
#line 157 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s_ex.h"

#line 165 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s_ex.h"







#line 181 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s_ex.h"



 

 


 
HAL_StatusTypeDef I2SEx_TransmitReceive_IT(I2S_HandleTypeDef *hi2s);
uint32_t I2S_GetInputClock(I2S_HandleTypeDef *hi2s); 


 



  



 








 
#line 358 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2s.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DeInit (I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);


 



 
 
 
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);

  
HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);

 
HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);

 
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);

 
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);


  



 

 
 
 


 



 

 


 




                           













                                    









 


 

 


 
void              I2S_DMATxCplt(DMA_HandleTypeDef *hdma);
void              I2S_DMATxHalfCplt(DMA_HandleTypeDef *hdma); 
void              I2S_DMARxCplt(DMA_HandleTypeDef *hdma);
void              I2S_DMARxHalfCplt(DMA_HandleTypeDef *hdma);
void              I2S_DMAError(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef I2S_WaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s, uint32_t Flag, uint32_t Status, uint32_t Timeout);
HAL_StatusTypeDef I2S_Transmit_IT(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef I2S_Receive_IT(I2S_HandleTypeDef *hi2s);


 



  



   








 
#line 335 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_iwdg.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_iwdg.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;  
 

  uint32_t Reload;     
 

} IWDG_InitTypeDef;



 
typedef struct
{
  IWDG_TypeDef                 *Instance;   

  IWDG_InitTypeDef             Init;        

}IWDG_HandleTypeDef;



 

 


 



 
#line 105 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_iwdg.h"


 



 



 

 


 





 







 




 

 


 



 
 
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);


 



 
 
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);


 



 

 


 



 







 

 


 





 






 






 
#line 216 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_iwdg.h"





 




 



 



 








 
#line 339 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_ltdc.h"



































  

 







#line 653 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_ltdc.h"







 
#line 343 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"



 



  

 



 
   


 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;



 

 


 
  


 



 



  
#line 104 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"


    
 


 
#line 118 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"


 




 




 
    


 




 



 




 



 







 



  
  
 


 





















 







 





 





 





 





 





 





 





 






 






 








 







 





 





 




 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



 



  

  
 


 
#line 84 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



 
#line 98 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"


 
#line 116 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



  
  
 


 

#line 144 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"









 
#line 162 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"

#line 210 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"


 

 


 
 


 
void HAL_PWREx_EnableFlashPowerDown(void);
void HAL_PWREx_DisableFlashPowerDown(void); 
HAL_StatusTypeDef HAL_PWREx_EnableBkUpReg(void);
HAL_StatusTypeDef HAL_PWREx_DisableBkUpReg(void); 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);









void HAL_PWREx_EnableMainRegulatorLowVoltage(void);
void HAL_PWREx_DisableMainRegulatorLowVoltage(void);
void HAL_PWREx_EnableLowRegulatorLowVoltage(void);
void HAL_PWREx_DisableLowRegulatorLowVoltage(void);



#line 250 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



 



 
 
 
 


 



 
 
 
 



 



 


    
 



 



 

 



   
 
 










 



 

 


 



 






#line 337 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"

#line 347 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"


 



 



  



 
  







 
#line 293 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"

 


 
  


 
 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);


 



 
 
 
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);

 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

 
void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);

 
void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);


 



 

 
 
 


 



 



 



 
 







 



 
 
 



 



 




 



 
 
 




 



 
 


 



 
#line 426 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"


 



 



  



 
  







 
#line 347 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rng.h"



































 

 







#line 361 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rng.h"








 
#line 351 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"



 



  

 


 



 
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,   
  HAL_RTC_STATE_READY             = 0x01U,   
  HAL_RTC_STATE_BUSY              = 0x02U,   
  HAL_RTC_STATE_TIMEOUT           = 0x03U,   
  HAL_RTC_STATE_ERROR             = 0x04U    
}HAL_RTCStateTypeDef;



 
typedef struct
{
  uint32_t HourFormat;      
 

  uint32_t AsynchPrediv;    
 

  uint32_t SynchPrediv;     
 

  uint32_t OutPut;          
 

  uint32_t OutPutPolarity;  
 

  uint32_t OutPutType;      
 
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t Hours;            

 

  uint8_t Minutes;          
 

  uint8_t Seconds;          
 

  uint8_t TimeFormat;       
 

  uint32_t SubSeconds;     

 

  uint32_t SecondFraction;  



 

  uint32_t DayLightSaving;  
 

  uint32_t StoreOperation;  

 
}RTC_TimeTypeDef; 



 
typedef struct
{
  uint8_t WeekDay;  
 

  uint8_t Month;    
 

  uint8_t Date;     
 

  uint8_t Year;     
 

}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef AlarmTime;      

  uint32_t AlarmMask;            
 

  uint32_t AlarmSubSecondMask;   
 

  uint32_t AlarmDateWeekDaySel;  
 

  uint8_t AlarmDateWeekDay;      

 

  uint32_t Alarm;                
 
}RTC_AlarmTypeDef;



  
typedef struct
{
  RTC_TypeDef                 *Instance;   

  RTC_InitTypeDef             Init;        

  HAL_LockTypeDef             Lock;        

  volatile HAL_RTCStateTypeDef    State;       

}RTC_HandleTypeDef;



 

 


 



  




  
  


  






  



  




  



  




  



  




  



  





 



  




 



  




 



  
 
#line 292 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


  



    
#line 306 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


  



  




  



  
#line 328 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


  



  




  



  
#line 377 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


    



  
#line 391 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



  
#line 413 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



  
  
 


 




 






 










 




 




 






 






 






 










    










 










 












 










 

  
  








 





 





 





 





 





 





 





 





 





 







 







 





 





 



 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"



 



  

  


 



 
typedef struct 
{
  uint32_t Tamper;                      
 

  uint32_t PinSelection;                
 

  uint32_t Trigger;                     
 

  uint32_t Filter;                      
 

  uint32_t SamplingFrequency;           
 

  uint32_t PrechargeDuration;           
  

  uint32_t TamperPullUp;                
            

  uint32_t TimeStampOnTamperDetection;  
 
}RTC_TamperTypeDef;


 

 


  



 
#line 123 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


  



  




 
  


  




 



  




  



  




  



  






   



  


#line 185 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 



  
#line 208 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 



  
#line 223 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 
  


  




 
  


  




 



  
#line 254 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


  



  




 



  
#line 276 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


  



  







 



  




  


 

  




  



  
  
 


 

 


 





 






 









 









 









 









 










 









 





 





 





 





 





 





 





 





 





 








 







 





 





 




 

 


 





 






 









 









 









 









 










 









 




 

 


 





 






 






 






 










 









 










 










 



 

 


 




 





 





 





 





 





 





 





 





 








 







 





 





 



 

 


 





 






 






 






 






 






 









 



 



 

 


 



 
 
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);

HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);
void HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);

void HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
uint32_t HAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
uint32_t HAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);

HAL_StatusTypeDef HAL_RTCEx_SetCoarseCalib(RTC_HandleTypeDef *hrtc, uint32_t CalibSign, uint32_t Value);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCoarseCalib(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmouthCalibMinusPulsesValue);
HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS);
HAL_StatusTypeDef HAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc);


 



 
 
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc); 
HAL_StatusTypeDef HAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 

 
 
 


 




 

 


 



  
#line 927 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"







#line 964 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"













#line 983 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 



 



  



  
  






 
#line 637 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

 


 



 
 
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void       HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void       HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);


 



 
 
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);


 



 
 
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void                HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef   HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void         HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);


 



 
 
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef* hrtc);


 



 
 
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);


 



 

 
 
 


 
 
#line 720 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"






 

 


 



 
#line 752 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

#line 783 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

#line 800 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 

 


 
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
uint8_t            RTC_ByteToBcd2(uint8_t Value);
uint8_t            RTC_Bcd2ToByte(uint8_t Value);


 



  



  
  






 
#line 355 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sai.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sai.h"



 
#line 861 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sai.h"


  



 
  






 
#line 359 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"



































  

 



#line 50 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"
 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"



































  

 



#line 50 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"
 
#line 52 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"



 



  

  


 
  


 
typedef struct
{
  uint32_t ClockEdge;            
 

  uint32_t ClockBypass;          

 

  uint32_t ClockPowerSave;       

 

  uint32_t BusWide;              
 

  uint32_t HardwareFlowControl;  
 

  uint32_t ClockDiv;             
 
  
}SDIO_InitTypeDef;
  



 
typedef struct
{
  uint32_t Argument;            


 

  uint32_t CmdIndex;            
 

  uint32_t Response;            
 

  uint32_t WaitForInterrupt;    

 

  uint32_t CPSM;                

 
}SDIO_CmdInitTypeDef;




 
typedef struct
{
  uint32_t DataTimeOut;          

  uint32_t DataLength;           
 
  uint32_t DataBlockSize;       
 
 
  uint32_t TransferDir;         

 
 
  uint32_t TransferMode;        
 
 
  uint32_t DPSM;                

 
}SDIO_DataInitTypeDef;



 
  
 


 



 







 



 







  



 







 



 









 



 







 
  


 



   
    


 



 



 









 



 









 



 







   



 











 



 



 



 
#line 313 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"

#line 329 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"


 



 







 



 







 



 







 
  


 







   



 
#line 408 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"


  



 
#line 439 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"


 



 
 


 



 
 


 
 




 
 




 



 



 



 
 




 



 



 



 




 




 

 
 
  




 
 
 



 
 




 


 


 



 




 




  





 





  





 

 































 

































 
































 






















 

































 





















 





   





   





   





   





   





   





   





   









     





   





     





   





   





   






 



   

 


 
  
 


 
HAL_StatusTypeDef SDIO_Init(SDIO_TypeDef *SDIOx, SDIO_InitTypeDef Init);


 
  
 


 
 
uint32_t          SDIO_ReadFIFO(SDIO_TypeDef *SDIOx);
HAL_StatusTypeDef SDIO_WriteFIFO(SDIO_TypeDef *SDIOx, uint32_t *pWriteData);


 
  
 


 
HAL_StatusTypeDef SDIO_PowerState_ON(SDIO_TypeDef *SDIOx);
HAL_StatusTypeDef SDIO_PowerState_OFF(SDIO_TypeDef *SDIOx);
uint32_t          SDIO_GetPowerState(SDIO_TypeDef *SDIOx);

 
HAL_StatusTypeDef SDIO_SendCommand(SDIO_TypeDef *SDIOx, SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
uint8_t           SDIO_GetCommandResponse(SDIO_TypeDef *SDIOx);
uint32_t          SDIO_GetResponse(uint32_t SDIO_RESP);

 
HAL_StatusTypeDef SDIO_DataConfig(SDIO_TypeDef *SDIOx, SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t          SDIO_GetDataCounter(SDIO_TypeDef *SDIOx);
uint32_t          SDIO_GetFIFOCount(SDIO_TypeDef *SDIOx);

 
HAL_StatusTypeDef SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);



 
  


 
  


  



 
#line 916 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_sdmmc.h"



 
#line 52 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"



 




 

  


 



 



typedef struct
{
  SDIO_TypeDef                   *Instance;         
  
  SDIO_InitTypeDef               Init;              
  
  HAL_LockTypeDef              Lock;              
  
  uint32_t                     CardType;          
  
  uint32_t                     RCA;               
  
  uint32_t                     CSD[4];            
  
  uint32_t                     CID[4];            
  
  volatile uint32_t                SdTransferCplt;    
  
  volatile uint32_t                SdTransferErr;     
  
  volatile uint32_t                DmaTransferCplt;   
  
  volatile uint32_t                SdOperation;       
  
  DMA_HandleTypeDef            *hdmarx;           
  
  DMA_HandleTypeDef            *hdmatx;           
  
}SD_HandleTypeDef;


 



  
typedef struct
{
  volatile uint8_t  CSDStruct;             
  volatile uint8_t  SysSpecVersion;        
  volatile uint8_t  Reserved1;             
  volatile uint8_t  TAAC;                  
  volatile uint8_t  NSAC;                  
  volatile uint8_t  MaxBusClkFrec;         
  volatile uint16_t CardComdClasses;       
  volatile uint8_t  RdBlockLen;            
  volatile uint8_t  PartBlockRead;         
  volatile uint8_t  WrBlockMisalign;       
  volatile uint8_t  RdBlockMisalign;       
  volatile uint8_t  DSRImpl;               
  volatile uint8_t  Reserved2;             
  volatile uint32_t DeviceSize;            
  volatile uint8_t  MaxRdCurrentVDDMin;    
  volatile uint8_t  MaxRdCurrentVDDMax;    
  volatile uint8_t  MaxWrCurrentVDDMin;    
  volatile uint8_t  MaxWrCurrentVDDMax;    
  volatile uint8_t  DeviceSizeMul;         
  volatile uint8_t  EraseGrSize;           
  volatile uint8_t  EraseGrMul;            
  volatile uint8_t  WrProtectGrSize;       
  volatile uint8_t  WrProtectGrEnable;     
  volatile uint8_t  ManDeflECC;            
  volatile uint8_t  WrSpeedFact;           
  volatile uint8_t  MaxWrBlockLen;         
  volatile uint8_t  WriteBlockPaPartial;   
  volatile uint8_t  Reserved3;             
  volatile uint8_t  ContentProtectAppli;   
  volatile uint8_t  FileFormatGrouop;      
  volatile uint8_t  CopyFlag;              
  volatile uint8_t  PermWrProtect;         
  volatile uint8_t  TempWrProtect;         
  volatile uint8_t  FileFormat;            
  volatile uint8_t  ECC;                   
  volatile uint8_t  CSD_CRC;               
  volatile uint8_t  Reserved4;             

}HAL_SD_CSDTypedef;


 



  
typedef struct
{
  volatile uint8_t  ManufacturerID;   
  volatile uint16_t OEM_AppliID;      
  volatile uint32_t ProdName1;        
  volatile uint8_t  ProdName2;        
  volatile uint8_t  ProdRev;          
  volatile uint32_t ProdSN;           
  volatile uint8_t  Reserved1;        
  volatile uint16_t ManufactDate;     
  volatile uint8_t  CID_CRC;          
  volatile uint8_t  Reserved2;        

}HAL_SD_CIDTypedef;


 



  
typedef struct
{
  volatile uint8_t  DAT_BUS_WIDTH;            
  volatile uint8_t  SECURED_MODE;             
  volatile uint16_t SD_CARD_TYPE;             
  volatile uint32_t SIZE_OF_PROTECTED_AREA;   
  volatile uint8_t  SPEED_CLASS;              
  volatile uint8_t  PERFORMANCE_MOVE;         
  volatile uint8_t  AU_SIZE;                  
  volatile uint16_t ERASE_SIZE;               
  volatile uint8_t  ERASE_TIMEOUT;            
  volatile uint8_t  ERASE_OFFSET;             

}HAL_SD_CardStatusTypedef;


 



  
typedef struct
{
  HAL_SD_CSDTypedef   SD_csd;          
  HAL_SD_CIDTypedef   SD_cid;          
  uint64_t            CardCapacity;    
  uint32_t            CardBlockSize;   
  uint16_t            RCA;             
  uint8_t             CardType;        

}HAL_SD_CardInfoTypedef;


 



  
typedef enum
{


    
  SD_CMD_CRC_FAIL                    = (1U),    
  SD_DATA_CRC_FAIL                   = (2U),    
  SD_CMD_RSP_TIMEOUT                 = (3U),    
  SD_DATA_TIMEOUT                    = (4U),    
  SD_TX_UNDERRUN                     = (5U),    
  SD_RX_OVERRUN                      = (6U),    
  SD_START_BIT_ERR                   = (7U),    
  SD_CMD_OUT_OF_RANGE                = (8U),    
  SD_ADDR_MISALIGNED                 = (9U),    
  SD_BLOCK_LEN_ERR                   = (10U),   
  SD_ERASE_SEQ_ERR                   = (11U),   
  SD_BAD_ERASE_PARAM                 = (12U),   
  SD_WRITE_PROT_VIOLATION            = (13U),   
  SD_LOCK_UNLOCK_FAILED              = (14U),   
  SD_COM_CRC_FAILED                  = (15U),   
  SD_ILLEGAL_CMD                     = (16U),   
  SD_CARD_ECC_FAILED                 = (17U),   
  SD_CC_ERROR                        = (18U),   
  SD_GENERAL_UNKNOWN_ERROR           = (19U),   
  SD_STREAM_READ_UNDERRUN            = (20U),   
  SD_STREAM_WRITE_OVERRUN            = (21U),   
  SD_CID_CSD_OVERWRITE               = (22U),   
  SD_WP_ERASE_SKIP                   = (23U),   
  SD_CARD_ECC_DISABLED               = (24U),   
  SD_ERASE_RESET                     = (25U),   
  SD_AKE_SEQ_ERROR                   = (26U),   
  SD_INVALID_VOLTRANGE               = (27U),
  SD_ADDR_OUT_OF_RANGE               = (28U),
  SD_SWITCH_ERROR                    = (29U),
  SD_SDIO_DISABLED                   = (30U),
  SD_SDIO_FUNCTION_BUSY              = (31U),
  SD_SDIO_FUNCTION_FAILED            = (32U),
  SD_SDIO_UNKNOWN_FUNCTION           = (33U),



  
  SD_INTERNAL_ERROR                  = (34U),
  SD_NOT_CONFIGURED                  = (35U),
  SD_REQUEST_PENDING                 = (36U),
  SD_REQUEST_NOT_APPLICABLE          = (37U),
  SD_INVALID_PARAMETER               = (38U),
  SD_UNSUPPORTED_FEATURE             = (39U),
  SD_UNSUPPORTED_HW                  = (40U),
  SD_ERROR                           = (41U),
  SD_OK                              = (0U) 

}HAL_SD_ErrorTypedef;


 



  
typedef enum
{
  SD_TRANSFER_OK    = 0U,   
  SD_TRANSFER_BUSY  = 1U,   
  SD_TRANSFER_ERROR = 2U    

}HAL_SD_TransferStateTypedef;


 



   
typedef enum
{
  SD_CARD_READY                  = ((uint32_t)0x00000001U),   
  SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002U),   
  SD_CARD_STANDBY                = ((uint32_t)0x00000003U),   
  SD_CARD_TRANSFER               = ((uint32_t)0x00000004U),     
  SD_CARD_SENDING                = ((uint32_t)0x00000005U),   
  SD_CARD_RECEIVING              = ((uint32_t)0x00000006U),   
  SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007U),   
  SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008U),   
  SD_CARD_ERROR                  = ((uint32_t)0x000000FFU)    

}HAL_SD_CardStateTypedef;


 



   
typedef enum
{
  SD_READ_SINGLE_BLOCK    = 0U,   
  SD_READ_MULTIPLE_BLOCK  = 1U,   
  SD_WRITE_SINGLE_BLOCK   = 2U,   
  SD_WRITE_MULTIPLE_BLOCK = 3U    

}HAL_SD_OperationTypedef;


 



 

 


 



 
#line 386 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"




 
#line 402 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"




 
#line 418 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"



 
#line 430 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"


 
  
 



 
 



  





 





  





 

 































 

































 
































 





















 

































 





















 



 
  
 


 



 
HAL_SD_ErrorTypedef HAL_SD_Init(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *SDCardInfo);
HAL_StatusTypeDef   HAL_SD_DeInit (SD_HandleTypeDef *hsd);
void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);


 



 
 
HAL_SD_ErrorTypedef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint32_t *pReadBuffer, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint32_t *pWriteBuffer, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_Erase(SD_HandleTypeDef *hsd, uint64_t startaddr, uint64_t endaddr);

 
void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd);

 
void HAL_SD_DMA_RxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_RxErrorCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxErrorCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_XferCpltCallback(SD_HandleTypeDef *hsd);
void HAL_SD_XferErrorCallback(SD_HandleTypeDef *hsd);

 
HAL_SD_ErrorTypedef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *hsd, uint32_t *pReadBuffer, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint32_t *pWriteBuffer, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_CheckWriteOperation(SD_HandleTypeDef *hsd, uint32_t Timeout);
HAL_SD_ErrorTypedef HAL_SD_CheckReadOperation(SD_HandleTypeDef *hsd, uint32_t Timeout);


 



 
HAL_SD_ErrorTypedef HAL_SD_Get_CardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *pCardInfo);
HAL_SD_ErrorTypedef HAL_SD_WideBusOperation_Config(SD_HandleTypeDef *hsd, uint32_t WideMode);
HAL_SD_ErrorTypedef HAL_SD_StopTransfer(SD_HandleTypeDef *hsd);
HAL_SD_ErrorTypedef HAL_SD_HighSpeed (SD_HandleTypeDef *hsd);


 
  
 


 
HAL_SD_ErrorTypedef HAL_SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus);
HAL_SD_ErrorTypedef HAL_SD_GetCardStatus(SD_HandleTypeDef *hsd, HAL_SD_CardStatusTypedef *pCardStatus);
HAL_SD_TransferStateTypedef HAL_SD_GetStatus(SD_HandleTypeDef *hsd);


 
  


 
    
 


 



  

 


 



  
          
 


 



  

 


 



  

 


 



 

 


 



 

 


 



 



  



  
#line 792 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_sd.h"



 
#line 363 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"



 



 

 


 



 
typedef struct
{
  uint32_t Mode;               
 

  uint32_t Direction;           
 

  uint32_t DataSize;           
 

  uint32_t CLKPolarity;        
 

  uint32_t CLKPhase;           
 

  uint32_t NSS;                

 

  uint32_t BaudRatePrescaler;  



 

  uint32_t FirstBit;           
 

  uint32_t TIMode;             
 

  uint32_t CRCCalculation;     
 

  uint32_t CRCPolynomial;      
 
}SPI_InitTypeDef;



 
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00U,     
  HAL_SPI_STATE_READY      = 0x01U,     
  HAL_SPI_STATE_BUSY       = 0x02U,     
  HAL_SPI_STATE_BUSY_TX    = 0x03U,     
  HAL_SPI_STATE_BUSY_RX    = 0x04U,     
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,     
  HAL_SPI_STATE_ERROR      = 0x06U      
}HAL_SPI_StateTypeDef;



 
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;     

  SPI_InitTypeDef            Init;          

  uint8_t                    *pTxBuffPtr;   

  uint16_t                   TxXferSize;    

  volatile uint16_t              TxXferCount;   

  uint8_t                    *pRxBuffPtr;   

  uint16_t                   RxXferSize;    

  volatile uint16_t              RxXferCount;   

  void                       (*RxISR)(struct __SPI_HandleTypeDef * hspi);  

  void                       (*TxISR)(struct __SPI_HandleTypeDef * hspi);  

  DMA_HandleTypeDef          *hdmatx;       

  DMA_HandleTypeDef          *hdmarx;       

  HAL_LockTypeDef            Lock;          

  volatile HAL_SPI_StateTypeDef  State;         

  volatile uint32_t              ErrorCode;     

}SPI_HandleTypeDef;



 

 


 



 
#line 175 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"


 



 




 



 





 



 




 



 




 



 




 



 





 



 
#line 246 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"


 



 




 



 




 



 




 



 





 



 
#line 297 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"


 



 

 


 





 











 












 















 






 






 
#line 377 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"





 
#line 390 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"





 
#line 402 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"





 






 



 

 


 



 
 
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit (SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);


 



 
 
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);


 



 
 
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);


 



 

 
 
 

 


 





 






 






 




























#line 544 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"














 

 


 



 



 



 







 
#line 367 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"



 



 

 


 
  


 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 
} TIM_Base_InitTypeDef;



 

typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 
  
  uint32_t OCFastMode;   

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;  



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
   
} TIM_OnePulse_InitTypeDef;  




 

typedef struct
{
  uint32_t  ICPolarity;   
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 

typedef struct
{
  uint32_t EncoderMode;   
 
                                  
  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 
                                  
  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;



  
typedef struct
{
  uint32_t ClockSource;     
  
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;    
 
}TIM_ClockConfigTypeDef;



  
typedef struct
{ 
  uint32_t ClearInputState;      
   
  uint32_t ClearInputSource;     
  
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;    
 
}TIM_ClearInputConfigTypeDef;



  
typedef struct {
  uint32_t  SlaveMode;         
  
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
   

}TIM_SlaveConfigTypeDef;



  
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
}HAL_TIM_StateTypeDef;



  
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
}HAL_TIM_ActiveChannel;



  
typedef struct
{
  TIM_TypeDef                 *Instance;      
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;        
  DMA_HandleTypeDef           *hdma[7];      
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;          
}TIM_HandleTypeDef;


 

 


 



 





 



 




 



 






 



 







 



 





 



 
#line 371 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"



 



 




 



 




 



 




 



 




  



 




  



 





                                 


 



 





 



 








 



 






  



 




 



 



   


 



  
#line 499 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 
  


   




 



 
#line 522 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 
#line 537 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"



 



 
#line 557 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 
#line 574 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 







 



 






 



 




 



 




 



 






 



   




 
  


 




 
  


 






   


 




 
  


 




 
  


 




   
  


   
#line 696 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


  
  


 







 



 




  
  


 
#line 733 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


   



 







 



 






 




 




  



 
#line 793 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


  



 
#line 818 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 
#line 832 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


  



 






  



    
  
 


 



 






 






 







 
#line 891 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 




 
#line 909 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"

#line 916 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"











































 













 








 






 








 









 












 
#line 1028 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"




 



















 
















 





    







 













 


















 







 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"



 



  

  


 
  


 

typedef struct
{

  uint32_t IC1Polarity;            
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
 

  uint32_t Commutation_Delay;  
 
} TIM_HallSensor_InitTypeDef;



  
typedef struct {
  uint32_t  MasterOutputTrigger;   
 

  uint32_t  MasterSlaveMode;       
 
}TIM_MasterConfigTypeDef;



  
typedef struct
{
  uint32_t OffStateRunMode;            
 
  uint32_t OffStateIDLEMode;          
 
  uint32_t LockLevel;                     
                              
  uint32_t DeadTime;                     
 
  uint32_t BreakState;                   
 
  uint32_t BreakPolarity;                 
 
  uint32_t AutomaticOutput;               
            
}TIM_BreakDeadTimeConfigTypeDef;


 
  
 


 
  


 
#line 135 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"






 

#line 154 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"



  
 
 


 



 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef* htim, TIM_HallSensor_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef* htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef* htim);

  
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef* htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef* htim);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef* htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);


 



 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef* htim, TIM_MasterConfigTypeDef * sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef* htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef* htim, uint32_t Remap);


 



  
 
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef* htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim);
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);


 



 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef* htim);


  



  

 
 
 
 


 
#line 308 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"

#line 315 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"




   
  
 


 
  


 



  



 
    






 
#line 1128 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"

 


 



 

 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 



 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);



 



 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
  
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);



 



 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef* sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef * sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef * sClockSourceConfig);    
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);



 



 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);



 
  


 
  
 


 



 












                              
#line 1361 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"





















































#line 1424 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"















































#line 1479 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"










#line 1497 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"























#line 1540 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"

#line 1559 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"




  



   

 




 
  


 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState);  


  
      


  



  
  






 
#line 371 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"



 



  

  


 



  
typedef struct
{
  uint32_t BaudRate;                  



 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 
 
  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 

 
  
  uint32_t OverSampling;              
  
}UART_InitTypeDef;







































  
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,    
 
  HAL_UART_STATE_READY             = 0x20U,    
 
  HAL_UART_STATE_BUSY              = 0x24U,    
 
  HAL_UART_STATE_BUSY_TX           = 0x21U,    
 
  HAL_UART_STATE_BUSY_RX           = 0x22U,    
 
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_UART_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_UART_STATE_ERROR             = 0xE0U     
 
}HAL_UART_StateTypeDef;



   
typedef struct
{
  USART_TypeDef                 *Instance;         
  
  UART_InitTypeDef              Init;              
  
  uint8_t                       *pTxBuffPtr;       
  
  uint16_t                      TxXferSize;        
  
  uint16_t                      TxXferCount;       
  
  uint8_t                       *pRxBuffPtr;       
  
  uint16_t                      RxXferSize;        
  
  uint16_t                      RxXferCount;         
  
  DMA_HandleTypeDef             *hdmatx;           
    
  DMA_HandleTypeDef             *hdmarx;           
  
  HAL_LockTypeDef               Lock;              

  volatile HAL_UART_StateTypeDef    gState;           

 
  
  volatile HAL_UART_StateTypeDef    RxState;          
 
  
  volatile uint32_t                 ErrorCode;         

}UART_HandleTypeDef;


 

 


 




  
#line 213 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"


 



 




 



 




  



  





  



  






 



  





 
    
 

  




 



 




 



   




 
                                         


 




 





 
#line 317 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"


 










  













 



 
  
 


 






 







 



















 


     





















 







 
#line 429 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"
                                              





 







 







 







 

                                                 















 



















 


















 















 


















 


















 


















 









      





       





     





     



 
    
 


 
  


     
 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit (UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);


 



 
 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);


 



 
 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);


 



 
 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);


  



  
 
 
 


 


  





 

 


 
#line 738 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"





 








 






 

 


 



 



  



 







 
#line 375 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_usart.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_usart.h"



 



  

  


 
   


  
typedef struct
{
  uint32_t BaudRate;                  


 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                   




 
 
  uint32_t Mode;                      
 

  uint32_t CLKPolarity;               
 

  uint32_t CLKPhase;                  
 

  uint32_t CLKLastBit;                

 
}USART_InitTypeDef;



  
typedef enum
{
  HAL_USART_STATE_RESET             = 0x00U,     
  HAL_USART_STATE_READY             = 0x01U,     
  HAL_USART_STATE_BUSY              = 0x02U,        
  HAL_USART_STATE_BUSY_TX           = 0x12U,      
  HAL_USART_STATE_BUSY_RX           = 0x22U,     
  HAL_USART_STATE_BUSY_TX_RX        = 0x32U,     
  HAL_USART_STATE_TIMEOUT           = 0x03U,     
  HAL_USART_STATE_ERROR             = 0x04U            
}HAL_USART_StateTypeDef;



   
typedef struct
{
  USART_TypeDef                 *Instance;         
  
  USART_InitTypeDef             Init;              
  
  uint8_t                       *pTxBuffPtr;       
  
  uint16_t                      TxXferSize;        
  
  volatile uint16_t                 TxXferCount;       
  
  uint8_t                       *pRxBuffPtr;       
  
  uint16_t                      RxXferSize;        
  
  volatile uint16_t                 RxXferCount;         
  
  DMA_HandleTypeDef             *hdmatx;           
    
  DMA_HandleTypeDef             *hdmarx;           
  
  HAL_LockTypeDef                Lock;             
  
  volatile HAL_USART_StateTypeDef    State;            
  
  volatile uint32_t                  ErrorCode;        

}USART_HandleTypeDef;


 

 


 




  
#line 165 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_usart.h"


 



 




 



 






  



  





  



  





 
    


  




  



 




  



 




 



 




 



 




 





 
#line 267 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_usart.h"


 










   












 



 

 


 





 
















 




















 






 
#line 361 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_usart.h"
                                               




 






 






 






 















 
#line 410 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_usart.h"
    












 






      





       






 






  




 
 


 



 
 
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart);
void HAL_USART_MspInit(USART_HandleTypeDef *husart);
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart);


 



 
 
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_TransmitReceive(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,  uint16_t Size);
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_DMAPause(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAResume(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAStop(USART_HandleTypeDef *husart);

void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart);


  



 
 
HAL_USART_StateTypeDef HAL_USART_GetState(USART_HandleTypeDef *husart);
uint32_t               HAL_USART_GetError(USART_HandleTypeDef *husart);


  



 
 
 
 


 


  








 

 


 
#line 556 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_usart.h"







 

 


 



 



  



 
  






 
#line 379 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_irda.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_irda.h"



 



  

  


 


  
typedef struct
{
  uint32_t BaudRate;                  


 

  uint32_t WordLength;                
 

  uint32_t Parity;                   




 
 
  uint32_t Mode;                      
 

  uint8_t  Prescaler;                  

  uint32_t IrDAMode;                  
 
}IRDA_InitTypeDef;







































  
typedef enum
{
  HAL_IRDA_STATE_RESET             = 0x00U,    
 
  HAL_IRDA_STATE_READY             = 0x20U,    
 
  HAL_IRDA_STATE_BUSY              = 0x24U,    
 
  HAL_IRDA_STATE_BUSY_TX           = 0x21U,    
 
  HAL_IRDA_STATE_BUSY_RX           = 0x22U,    
 
  HAL_IRDA_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_IRDA_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_IRDA_STATE_ERROR             = 0xE0U     
 
}HAL_IRDA_StateTypeDef;



   
typedef struct
{
  USART_TypeDef               *Instance;         

  IRDA_InitTypeDef            Init;              

  uint8_t                     *pTxBuffPtr;       

  uint16_t                    TxXferSize;        

  uint16_t                    TxXferCount;       

  uint8_t                     *pRxBuffPtr;       

  uint16_t                    RxXferSize;        

  uint16_t                    RxXferCount;       

  DMA_HandleTypeDef           *hdmatx;           

  DMA_HandleTypeDef           *hdmarx;           

  HAL_LockTypeDef             Lock;              

  volatile HAL_IRDA_StateTypeDef  gState;           

 

  volatile HAL_IRDA_StateTypeDef  RxState;          
 

  volatile uint32_t               ErrorCode;         

}IRDA_HandleTypeDef;


 

 


 



  
#line 205 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_irda.h"


 



 




 



  





  



  





 



 




 





 
#line 260 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_irda.h"


 
  








 












 



 
  
 


 






 









 

















 





















 







 
#line 366 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_irda.h"
                                              





 







 







 







 















 
#line 419 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_irda.h"
    













 






      





       







 







 

    


 

 


 
  


 
 
HAL_StatusTypeDef HAL_IRDA_Init(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DeInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspDeInit(IRDA_HandleTypeDef *hirda);


 



 
 
HAL_StatusTypeDef HAL_IRDA_Transmit(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Receive(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Transmit_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Transmit_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_DMAPause(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAResume(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAStop(IRDA_HandleTypeDef *hirda);

void HAL_IRDA_IRQHandler(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda);


 



 
 
HAL_IRDA_StateTypeDef HAL_IRDA_GetState(IRDA_HandleTypeDef *hirda);
uint32_t HAL_IRDA_GetError(IRDA_HandleTypeDef *hirda);


  



 

 
 
 


 



  








 

 


 
#line 557 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_irda.h"





 






 

 


 



 



  



  
  






 
#line 383 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"



































 

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"



 



 

  


 



 
typedef struct
{
  uint32_t BaudRate;                  


 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                   




 

  uint32_t Mode;                      
 

  uint32_t CLKPolarity;               
 

  uint32_t CLKPhase;                  
 

  uint32_t CLKLastBit;                

 

  uint32_t Prescaler;                 


 

  uint32_t GuardTime;                  

  uint32_t NACKState;                 
 
}SMARTCARD_InitTypeDef;







































 
typedef enum
{
  HAL_SMARTCARD_STATE_RESET             = 0x00U,    
 
  HAL_SMARTCARD_STATE_READY             = 0x20U,    
 
  HAL_SMARTCARD_STATE_BUSY              = 0x24U,    
 
  HAL_SMARTCARD_STATE_BUSY_TX           = 0x21U,    
 
  HAL_SMARTCARD_STATE_BUSY_RX           = 0x22U,    
 
  HAL_SMARTCARD_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_SMARTCARD_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_SMARTCARD_STATE_ERROR             = 0xE0U     
 
}HAL_SMARTCARD_StateTypeDef;



 
typedef struct
{
  USART_TypeDef                    *Instance;         

  SMARTCARD_InitTypeDef            Init;              

  uint8_t                          *pTxBuffPtr;       

  uint16_t                         TxXferSize;        

  uint16_t                         TxXferCount;       

  uint8_t                          *pRxBuffPtr;       

  uint16_t                         RxXferSize;        

  uint16_t                         RxXferCount;       

  DMA_HandleTypeDef                *hdmatx;           

  DMA_HandleTypeDef                *hdmarx;           

  HAL_LockTypeDef                  Lock;              

  volatile HAL_SMARTCARD_StateTypeDef  gState;           

 
  
  volatile HAL_SMARTCARD_StateTypeDef  RxState;          
 

  volatile uint32_t  ErrorCode;                           

}SMARTCARD_HandleTypeDef;



 

 


 



  
#line 225 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"


 



 



 



 




 



 




 



 





 



 




  



 




 



 




 



 




 



 




 



 
#line 344 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"


 





 
#line 361 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"


 








 
#line 379 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"


 



 
  
 


 




 







 

    













 
















 







 
#line 452 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"






 







 







 







 












 
















 





      





       






 






 








 





 

 


 
  


 
 
HAL_StatusTypeDef HAL_SMARTCARD_Init(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_ReInit(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_DeInit(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_MspInit(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_MspDeInit(SMARTCARD_HandleTypeDef *hsc);


 



 
 
HAL_StatusTypeDef HAL_SMARTCARD_Transmit(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Receive(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_DMA(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_DMA(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);

void HAL_SMARTCARD_IRQHandler(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsc);


 



 
 
HAL_SMARTCARD_StateTypeDef HAL_SMARTCARD_GetState(SMARTCARD_HandleTypeDef *hsc);
uint32_t HAL_SMARTCARD_GetError(SMARTCARD_HandleTypeDef *hsc);



  



 
 
 
 


 



  







 








 

 


 
#line 651 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_smartcard.h"


 

 


 



 



  



 






 
#line 387 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_wwdg.h"



































  

 







 
#line 48 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_wwdg.h"



 



  

 


 



 
typedef struct
{
  uint32_t Prescaler;     
 

  uint32_t Window;        
 

  uint32_t Counter;       
 

  uint32_t EWIMode ;      
 

}WWDG_InitTypeDef;



 
typedef struct
{
  WWDG_TypeDef                 *Instance;   

  WWDG_InitTypeDef             Init;        

}WWDG_HandleTypeDef;


 

 



 



 



 




 



 



  






 



 




 



 

 



 













 


 



 





 










 









 








 









 









 








 




 

 


 



 
 
HAL_StatusTypeDef     HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg);


 



 
 
HAL_StatusTypeDef     HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg);


 



 



 



 







 
#line 391 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"



































  

 



#line 50 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"
 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_usb.h"



































  

 



#line 50 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_usb.h"
 
#line 52 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_usb.h"



 



  

  



   
typedef enum 
{
   USB_OTG_DEVICE_MODE  = 0U,
   USB_OTG_HOST_MODE    = 1U,
   USB_OTG_DRD_MODE     = 2U
   
}USB_OTG_ModeTypeDef;



  
typedef enum {
  URB_IDLE = 0U,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
    
}USB_OTG_URBStateTypeDef;



  
typedef enum {
  HC_IDLE = 0U,
  HC_XFRC,
  HC_HALTED,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,  
  HC_BBLERR,   
  HC_DATATGLERR
    
}USB_OTG_HCStateTypeDef;



 
typedef struct
{
  uint32_t dev_endpoints;        

 

  uint32_t Host_channels;        

 

  uint32_t speed;                
 

  uint32_t dma_enable;            

  uint32_t ep0_mps;              
 

  uint32_t phy_itface;           
 

  uint32_t Sof_enable;            

  uint32_t low_power_enable;      

  uint32_t lpm_enable;            

  uint32_t battery_charging_enable;  

  uint32_t vbus_sensing_enable;    

  uint32_t use_dedicated_ep1;     

  uint32_t use_external_vbus;     

}USB_OTG_CfgTypeDef;



 
typedef struct
{
  uint8_t   num;            
 

  uint8_t   is_in;          
 

  uint8_t   is_stall;       
 

  uint8_t   type;           
 

  uint8_t   data_pid_start; 
 

  uint8_t   even_odd_frame; 
 

  uint16_t  tx_fifo_num;    
 

  uint32_t  maxpacket;      
 

  uint8_t   *xfer_buff;      

  uint32_t  dma_addr;        

  uint32_t  xfer_len;        

  uint32_t  xfer_count;      

}USB_OTG_EPTypeDef;



 
typedef struct
{
  uint8_t   dev_addr ;     
 

  uint8_t   ch_num;        
 

  uint8_t   ep_num;        
 

  uint8_t   ep_is_in;      
 

  uint8_t   speed;         
 

  uint8_t   do_ping;        

  uint8_t   process_ping;   

  uint8_t   ep_type;       
 

  uint16_t  max_packet;    
 

  uint8_t   data_pid;      
 

  uint8_t   *xfer_buff;     

  uint32_t  xfer_len;       

  uint32_t  xfer_count;     

  uint8_t   toggle_in;     
 

  uint8_t   toggle_out;    
 

  uint32_t  dma_addr;       

  uint32_t  ErrCnt;         

  USB_OTG_URBStateTypeDef  urb_state;  
 

  USB_OTG_HCStateTypeDef   state;     
 

}USB_OTG_HCTypeDef;
  
 



 



 





 



   






 
  


    




 
  


 





 



 






 
  


   






 



 






 



 





 



 







 



 







 



   





 
    


     





   
   




       









    












 
 


    



 
HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx , USB_OTG_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx , uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo (USB_OTG_GlobalTypeDef *USBx, uint32_t num );
HAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_ActivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len, uint8_t dma);
void *            USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len);
HAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress (USB_OTG_GlobalTypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateSetup (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx, uint8_t dma, uint8_t *psetup);
uint8_t           USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetMode(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadInterrupts (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt (USB_OTG_GlobalTypeDef *USBx , uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt (USB_OTG_GlobalTypeDef *USBx , uint8_t epnum);
void              USB_ClearInterrupts (USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt);

HAL_StatusTypeDef USB_HostInit (USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_InitFSLSPClkSel(USB_OTG_GlobalTypeDef *USBx , uint8_t freq);
HAL_StatusTypeDef USB_ResetPort(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DriveVbus (USB_OTG_GlobalTypeDef *USBx, uint8_t state);
uint32_t          USB_GetHostSpeed (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetCurrentFrame (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx,  
                                  uint8_t ch_num,
                                  uint8_t epnum,
                                  uint8_t dev_address,
                                  uint8_t speed,
                                  uint8_t ep_type,
                                  uint16_t mps);
HAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_HCTypeDef *hc, uint8_t dma);
uint32_t          USB_HC_ReadInterrupt (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Halt(USB_OTG_GlobalTypeDef *USBx , uint8_t hc_num);
HAL_StatusTypeDef USB_DoPing(USB_OTG_GlobalTypeDef *USBx , uint8_t ch_num);
HAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx);



  



 
#line 473 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_ll_usb.h"




 
#line 52 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"
   


 



  

  


 
   


 
typedef enum 
{
  HAL_PCD_STATE_RESET   = 0x00U,
  HAL_PCD_STATE_READY   = 0x01U,
  HAL_PCD_STATE_ERROR   = 0x02U,
  HAL_PCD_STATE_BUSY    = 0x03U,
  HAL_PCD_STATE_TIMEOUT = 0x04U
} PCD_StateTypeDef;

#line 88 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"

typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;
typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;
typedef USB_OTG_EPTypeDef      PCD_EPTypeDef ;



  
typedef struct
{
  PCD_TypeDef             *Instance;     
  PCD_InitTypeDef         Init;          
  PCD_EPTypeDef           IN_ep[15];     
  PCD_EPTypeDef           OUT_ep[15];    
  HAL_LockTypeDef         Lock;          
  volatile PCD_StateTypeDef   State;         
  uint32_t                Setup[12];     
#line 115 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"
  void                    *pData;        
} PCD_HandleTypeDef;



 

 
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd_ex.h"



































  

 



#line 50 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd_ex.h"
 
#line 52 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd_ex.h"
   


 



 
 
#line 69 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd_ex.h"

#line 81 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd_ex.h"

 
 
 


 


 
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size);
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size);
#line 107 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd_ex.h"



  



  



  



 
#line 129 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd_ex.h"




 
#line 124 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"

 


 



 





 
  


 




 
  


 
#line 158 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"


 



 

 



 




































































  

 


 

 


 
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);


 

 
 


 
 
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);


 

 


 
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
uint16_t          HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);


 

 


 
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd);


  



 
 
 


 



  



  



  
#line 339 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pcd.h"




 
#line 395 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_hcd.h"



































  

 



#line 50 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_hcd.h"
 
#line 52 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_hcd.h"
   


 



 

  


 



 
typedef enum 
{
  HAL_HCD_STATE_RESET    = 0x00U,
  HAL_HCD_STATE_READY    = 0x01U,
  HAL_HCD_STATE_ERROR    = 0x02U,
  HAL_HCD_STATE_BUSY     = 0x03U,
  HAL_HCD_STATE_TIMEOUT  = 0x04U
} HCD_StateTypeDef;

typedef USB_OTG_GlobalTypeDef   HCD_TypeDef;
typedef USB_OTG_CfgTypeDef      HCD_InitTypeDef;
typedef USB_OTG_HCTypeDef       HCD_HCTypeDef ;   
typedef USB_OTG_URBStateTypeDef HCD_URBStateTypeDef ;
typedef USB_OTG_HCStateTypeDef  HCD_HCStateTypeDef ;


 



  
typedef struct
{
  HCD_TypeDef               *Instance;    
  HCD_InitTypeDef           Init;        
  HCD_HCTypeDef             hc[15];      
  HAL_LockTypeDef           Lock;        
  volatile HCD_StateTypeDef     State;       
  void                      *pData;           
} HCD_HandleTypeDef;


 



 

 


 



 





 



 




 



  

 



 


   



  







 

 


 

 


 
HAL_StatusTypeDef   HAL_HCD_Init(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef   HAL_HCD_DeInit(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef   HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd,  
                                    uint8_t ch_num,
                                    uint8_t epnum,
                                    uint8_t dev_address,
                                    uint8_t speed,
                                    uint8_t ep_type,
                                    uint16_t mps);

HAL_StatusTypeDef   HAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num);

void                HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd);
void                HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd);


 

 


 
HAL_StatusTypeDef   HAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd,
                                             uint8_t pipe, 
                                             uint8_t direction,
                                             uint8_t ep_type,  
                                             uint8_t token, 
                                             uint8_t* pbuff, 
                                             uint16_t length,
                                             uint8_t do_ping);

 
void                HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);
void                HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd);
void                HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd);
void                HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd);
void                HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, 
                                                        uint8_t chnum, 
                                                        HCD_URBStateTypeDef urb_state);


 

 


 
HAL_StatusTypeDef   HAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef   HAL_HCD_Start(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef   HAL_HCD_Stop(HCD_HandleTypeDef *hhcd);


 

 


 
HCD_StateTypeDef    HAL_HCD_GetState(HCD_HandleTypeDef *hhcd);
HCD_URBStateTypeDef HAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t            HAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum);
HCD_HCStateTypeDef  HAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t            HAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd);
uint32_t            HAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd);


 



 

 


 



 



 



  
#line 259 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_hcd.h"



 
#line 399 ".\\Inc\\stm32f4xx_hal_conf.h"

   
#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dsi.h"



































 

 







#line 1235 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dsi.h"
  






 
#line 403 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_qspi.h"



































 

 







#line 777 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_qspi.h"







 
#line 407 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cec.h"



































 

 







#line 740 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cec.h"
  






 
#line 411 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_fmpi2c.h"



































 

 







#line 715 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_fmpi2c.h"




 
#line 415 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spdifrx.h"



































  

 







#line 547 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spdifrx.h"








 
#line 419 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dfsdm.h"



































 

 







#line 688 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dfsdm.h"



 
#line 423 ".\\Inc\\stm32f4xx_hal_conf.h"


#line 1 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_lptim.h"



































  

 







#line 760 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_lptim.h"



 
#line 427 ".\\Inc\\stm32f4xx_hal_conf.h"


 
#line 445 ".\\Inc\\stm32f4xx_hal_conf.h"







 

 
#line 49 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"



 



  

 
 
 


 


 
#line 89 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

#line 112 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"


 



 





 




#line 136 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

#line 151 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

#line 181 ".\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"


 

 


 


 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);


 



 
 
void HAL_IncTick(void);
void HAL_Delay(volatile uint32_t Delay);
uint32_t HAL_GetTick(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
void HAL_EnableCompensationCell(void);
void HAL_DisableCompensationCell(void);







 



 
 
 


 


 
 


 


 
 
 


 



  
  






 
#line 44 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"



 




 



 
 


 


  
 
 
 
 


 



















 

















 
void HAL_RCC_DeInit(void)
{
   
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= (0x00000001U | 0x00000080U)); 
  
   
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) = (0x0));
  
   
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~(0x00010000U | 0x00080000U | 0x01000000U| 0x04000000U)); 
  
   
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) = (0x0));
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) |= (0x00000010U | 0x00001000U | 0x00002000U | 0x04000000U)); 
  
   
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR) = (0x0));
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR) |= (0x00001000U | 0x00002000U | 0x20000000U));
  
   
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~(0x00040000U));
  
   
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) = (0x0));
  
   
  SystemCoreClock = ((uint32_t)16000000U);
}




#line 180 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"

#line 792 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"

#line 1135 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"

#line 1443 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"

#line 1594 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"

#line 1867 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"














 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;
    
   
  ((void)0);
  
   
  if(((PeriphClkInit->PeriphClockSelection) & ((uint32_t)0x00000001U)) == (((uint32_t)0x00000001U)))
  {
     
    ((void)0);
    ((void)0);



     
    (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32) + (0x1A * 4)) = DISABLE);
     
    tickstart = HAL_GetTick();
     
    while((((((((((uint8_t)0x3BU)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x3BU)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x3BU)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & ((uint32_t)1U << ((((uint8_t)0x3BU)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U)  != RESET)
    {
      if((HAL_GetTick() - tickstart ) > ((uint32_t)2))
      {
         
        return HAL_TIMEOUT;
      } 
    }

#line 1919 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"
     
     
     
    (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR = (((PeriphClkInit->PLLI2S . PLLI2SN) << (__clz(__rbit(0x00007FC0U)))) | ((PeriphClkInit->PLLI2S . PLLI2SR) << (__clz(__rbit(0x70000000U))))));

    
     
    (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32) + (0x1A * 4)) = ENABLE);
     
    tickstart = HAL_GetTick();
     
    while((((((((((uint8_t)0x3BU)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x3BU)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x3BU)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & ((uint32_t)1U << ((((uint8_t)0x3BU)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U)  == RESET)
    {
      if((HAL_GetTick() - tickstart ) > ((uint32_t)2))
      {
         
        return HAL_TIMEOUT;
      }
    }
  }
  
   
  if(((PeriphClkInit->PeriphClockSelection) & ((uint32_t)0x00000002U)) == (((uint32_t)0x00000002U)))
  {
     
    ((void)0);
    
     
    do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) |= (0x10000000U)); tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) & (0x10000000U)); ((void)(tmpreg)); } while(0);
    
     
    ((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR |= 0x00000100U;
    
     
    tickstart = HAL_GetTick();
    
    while((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR & 0x00000100U) == RESET)
    {
      if((HAL_GetTick() - tickstart ) > ((uint32_t)2U))
      {
        return HAL_TIMEOUT;
      }
    }
      
    tmpreg1 = (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR & 0x00000300U);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & 0x00000300U)))
    {
       
      tmpreg1 = (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR & ~(0x00000300U));
       
      (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x70U) * 32U) + (0x10U * 4U)) = ENABLE);
      (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x70U) * 32U) + (0x10U * 4U)) = DISABLE);
       
      ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR = tmpreg1;

       
      if((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) & (0x00000001U)) != RESET))
      {
         
        tickstart = HAL_GetTick();
        
           
        while((((((((((uint8_t)0x41U)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x41U)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x41U)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & ((uint32_t)1U << ((((uint8_t)0x41U)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    do { (((PeriphClkInit->RTCClockSelection) & 0x00000300U) == 0x00000300U) ? (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~(0x001F0000U))) | (((PeriphClkInit->RTCClockSelection) & 0xFFFFCFFU))))) : ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) &= ~(0x001F0000U)); ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR |= ((PeriphClkInit->RTCClockSelection) & 0x00000FFFU); } while (0);
  }

   
  if(((PeriphClkInit->PeriphClockSelection) & ((uint32_t)0x00000008U)) == (((uint32_t)0x00000008U)))
  {
    (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x8C) * 32) + (0x18 * 4)) = (PeriphClkInit->TIMPresSelection));
  }

  return HAL_OK;
}







 
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;
  
   
  PeriphClkInit->PeriphClockSelection = ((uint32_t)0x00000001U) | ((uint32_t)0x00000002U);
  
   
  PeriphClkInit->PLLI2S.PLLI2SN = (uint32_t)((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR & 0x00007FC0U) >> (__clz(__rbit(0x00007FC0U))));
  PeriphClkInit->PLLI2S.PLLI2SR = (uint32_t)((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR & 0x70000000U) >> (__clz(__rbit(0x70000000U))));



   
  tempreg = (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & 0x001F0000U);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR & 0x00000300U));


   
  if ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->DCKCFGR & 0x01000000U) == RESET)
  {
    PeriphClkInit->TIMPresSelection = ((uint8_t)0x00U);
  }
  else
  {
    PeriphClkInit->TIMPresSelection = ((uint8_t)0x01U);
  }

}


#line 2451 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"

#line 2480 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"

#line 2585 "Drivers\\STM32F4xx_HAL_Driver\\Src\\stm32f4xx_hal_rcc_ex.c"



 



 




 



 

 
