#line 1 "LSM9DS1\\LSM9DS1.c"
#line 1 "LSM9DS1\\LSM9DS1.h"



void init_LSM9DS1_I2C ();



#line 2 "LSM9DS1\\LSM9DS1.c"
#line 1 "LSM9DS1\\i2c.h"





































 
 






 
#line 1 "..\\Inc\\main.h"

































 

 







 
#line 1 "..\\Inc\\cube_hal.h"

































 

 



 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"


















  

 







 
#line 1 "..\\Inc\\stm32f4xx_hal_conf.h"

































 

 







 
 

 


 


 


 
 
 

 
 

 
 
 
 
 
 


 
 
 


 

 
 



 
 
 
 
 
 


 




 











 








 






 







 






 








 








 





 

 


 
#line 175 "..\\Inc\\stm32f4xx_hal_conf.h"

 



 
 

 




 



 


 

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

















 

 







 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"


















 

 







 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"











































 



 



 
    






   


 
  


 






 
#line 111 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"
   


 
#line 123 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 
#line 135 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 



 

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







































 



 



 
    









 



 








 
  


 




 
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



 

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"
 




 

























 











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






 
#line 45 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

















 




 



 

 













#line 120 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"



 
#line 135 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

#line 209 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
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









 









 








 
#line 455 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\cmsis_armcc.h"











 


#line 54 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmInstr.h"

 
#line 84 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmInstr.h"

   

#line 211 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmFunc.h"

 
#line 84 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmFunc.h"

 

#line 212 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmSimd.h"

 
#line 88 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cmSimd.h"

 






#line 213 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"
















 
#line 256 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

 






 
#line 272 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

 




 













 



 






 



 
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

 




































 






 







































 







 






 







 


 







 

 
#line 1541 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"

#line 1550 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Include\\core_cm4.h"











 










 


 



 





 









 
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

 










#line 157 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\system_stm32f4xx.h"

































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 158 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
#line 159 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



    



 

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
} USB_OTG_GlobalTypeDef;



 
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
} USB_OTG_DeviceTypeDef;



 
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
} USB_OTG_INEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
} USB_OTG_OUTEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;              
  volatile uint32_t HFNUM;             
  uint32_t Reserved40C;            
  volatile uint32_t HPTXSTS;           
  volatile uint32_t HAINT;             
  volatile uint32_t HAINTMSK;          
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;            
  volatile uint32_t HCSPLT;            
  volatile uint32_t HCINT;             
  volatile uint32_t HCINTMSK;          
  volatile uint32_t HCTSIZ;            
  volatile uint32_t HCDMA;             
  uint32_t Reserved[2];            
} USB_OTG_HostChannelTypeDef;



 



 
#line 661 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 





 
#line 689 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 





 
#line 706 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 735 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 

 


#line 754 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"






 



   
#line 786 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 825 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 
#line 867 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 921 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
  
 
#line 971 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1027 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1089 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 




 




 




 




 
#line 1160 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1210 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1260 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1299 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 




 




 
#line 1327 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1347 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 1386 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1394 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 
 
 
 
 
 





 





 





 
 
 
 
 
 
#line 1500 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 1526 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1545 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1607 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1669 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1731 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

  
#line 1793 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 





 
 
 
 
 
 
#line 1885 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1913 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 1984 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2009 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2080 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2151 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2222 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2293 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 
#line 2311 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2333 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2359 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2392 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2400 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 2441 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
                                             
 
#line 2458 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 
#line 2545 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2627 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2677 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2695 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2777 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2827 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2909 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 2959 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3009 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3027 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3077 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 3094 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3192 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3274 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 3326 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 3383 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3425 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3483 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3525 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 
 
 
 
 
 
#line 3575 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3586 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3602 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



#line 3637 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





 
#line 3649 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 3698 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3724 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3735 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 3748 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 




 
#line 3766 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 3779 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
 
 
 
 
 
#line 3803 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3810 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3840 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 3866 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
 
 
 
 
 
#line 3882 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3891 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3903 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3922 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 


#line 3933 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 3944 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3957 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 3971 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 3979 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 
 










 










 
#line 4012 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4022 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4030 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 4044 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 4060 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 










#line 4078 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4085 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 4111 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4133 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4152 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





 
#line 4185 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 



 


 
#line 4230 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4265 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 4297 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 


 






 
#line 4344 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4379 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4414 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 




 

 
#line 4460 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4495 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4506 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 4519 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4551 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 



 
#line 4568 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4582 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 4589 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






 
 
 
 
 
 
#line 4644 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4688 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4758 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 4811 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4819 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 4832 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4902 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 4972 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 
#line 4990 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5033 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5063 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 5091 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5139 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 5154 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5166 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 



 
 
 
 
 
 






 
#line 5296 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 5309 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 










#line 5347 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 




 




 




 




 




 




 
#line 5401 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 5409 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 5422 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 5501 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5542 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5616 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 
 
 
 
 


 
#line 5644 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 5651 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 5682 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5705 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5734 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 




 




 






























#line 5792 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5803 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 





 




 
#line 5833 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 
#line 5842 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5852 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5862 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5872 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5886 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5896 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5906 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5916 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5926 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5940 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5950 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5960 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5970 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 5980 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 5994 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 6004 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 6014 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 6024 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"



 
#line 6034 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6042 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 
#line 6064 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

















 
#line 6091 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6098 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6123 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6131 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6138 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





#line 6150 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 6163 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6210 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6248 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6274 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






#line 6288 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6295 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"











#line 6312 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6319 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





 







#line 6339 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 6353 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






#line 6367 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6374 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"











#line 6391 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6398 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





 







#line 6418 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 6432 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6479 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 




 




 




 




 




 




 
#line 6532 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 6557 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6567 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 6576 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 






#line 6599 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 
 
 
 
 
 
#line 6637 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 6650 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6697 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6720 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"











 
#line 6768 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6781 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





 
 
 
 
 
 
#line 6802 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 6810 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





 
#line 6826 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 6834 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"






 







 





 
 
 
 
 
 
#line 6866 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 6880 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 6918 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 


 
#line 6934 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
 
 
 
 
#line 6971 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 6982 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 6993 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7004 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"





















 
#line 7035 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7055 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7069 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7091 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 7104 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




#line 7121 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7143 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 7207 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7224 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


#line 7240 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7266 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7282 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7294 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 7337 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 7416 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7496 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7504 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 7523 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7531 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 
#line 7555 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




#line 7577 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7588 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7596 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7612 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7628 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 7641 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7661 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7669 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 
#line 7703 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7732 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7741 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7749 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 7790 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7798 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7812 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7821 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7847 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




#line 7866 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"













#line 7898 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 7911 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7922 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 7934 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 7969 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 8010 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 8045 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 8057 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 
#line 8072 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 




 




 
#line 8095 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 

#line 8136 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 8171 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 

#line 8179 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







 
#line 8196 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
 
#line 8209 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"







#line 8223 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 8231 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

#line 8239 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


  



 



 

 



 



 
#line 8276 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 8284 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 




 


 




 


 



 



 






 
#line 8325 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 
#line 8336 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 8344 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






 






 


 






 






 






 






 






 






 
#line 8411 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 



 






 




 
#line 8472 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






 






 
#line 8496 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"


 



 






 
#line 8516 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 






 
#line 8531 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 
#line 8539 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"

 


 
#line 8550 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"
 





 


 




 




 


 



 


 




 




 


 


 


 


 


 







 



#line 8623 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f401xe.h"













 



 



 









 
#line 164 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"
#line 193 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 



  
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
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;



 




 



















 

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"


















  

 
#line 285 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

 
#line 251 "..\\..\\..\\..\\..\\..\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"









 



 
  



 
#line 31 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


















 

 







 
 
 



 








 



 
#line 89 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 97 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 





 



 
#line 135 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 202 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 






 



 

#line 238 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 260 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





#line 300 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 359 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 454 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 471 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 489 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 
#line 508 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 


















#line 551 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





#line 562 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 569 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"










 



 
#line 593 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 602 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 625 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 



 










 



 




























 



 


#line 758 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 

 
#line 780 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 












 



 






























 




 















 




 
#line 867 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 









#line 897 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 



#line 935 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 945 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 964 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"












 



 




 



 

























 




 








 



 




 



 
#line 1053 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 1070 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1082 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1113 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 











 

#line 1161 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

 



 



 



 
#line 1189 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 













 



 
#line 1224 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 1238 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 

 



 







#line 1263 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1274 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 

 



 
#line 1297 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1305 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






#line 1321 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 

 



 





 



 



 



 
#line 1361 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 






 




 



 

 



 





 



 
#line 1422 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"









 




 
#line 1450 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1471 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1482 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1491 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1504 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1513 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 







 



 
#line 1549 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1564 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


#line 1597 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 1764 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



#line 1774 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 

#line 1788 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 







 



 

#line 1811 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 1839 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 










 



 














 




 




 




 







 




 
#line 1917 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




 
#line 1961 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1975 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 







#line 2248 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2262 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2505 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2653 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 



#line 2678 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2699 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2816 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2833 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2848 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






#line 2877 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

















#line 2903 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





#line 2930 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2937 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2946 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2979 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2997 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"












#line 3015 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3036 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3044 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 



 
#line 3067 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3095 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3110 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




#line 3146 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 




#line 3176 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3183 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3195 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 

#line 3209 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 
#line 3230 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 



 













 




 











 



 












#line 3303 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3312 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3321 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 








#line 3354 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 



 

#line 3371 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




 



 
#line 3405 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 



 
#line 3432 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 







 

#line 32 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 33 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"

 



   
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

 




























 


#line 103 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"







#line 118 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"


 
#line 140 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"




  









 


#line 173 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"



  



 


#line 190 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_def.h"







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

 
 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       
 

  uint32_t PLLN;       

 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 
#line 75 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
}RCC_PLLInitTypeDef;

#line 176 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 202 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 293 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 378 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"





 
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



  

 


 



 
 
#line 454 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 464 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 481 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 
    
 
#line 495 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 507 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 519 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 


 






 




 





 
#line 548 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 



 
#line 562 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 



 
#line 575 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 

#line 600 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      
#line 629 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 722 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 777 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 856 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 890 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"








 




 




#line 922 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"








 






 




#line 954 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"



 
     
 


 
 
#line 2007 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 2875 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 







 
#line 2914 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"







 







 











  
  






 



                                        



 







 




   
  






 
#line 3019 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 







 













  
  






 
#line 3073 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"






 
  






 









 



   











 




 







 




 
#line 3138 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 3145 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"


 




 











 




  




 








 













 








 





 








 













 








 









 

 

 
#line 3526 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 3902 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 4706 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 5735 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 5781 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"




























 
#line 5816 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 
                             
 








 



#line 5899 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"














 





#line 5944 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 5967 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 6057 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 
#line 6076 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
                                 
#line 6095 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6107 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
 

 











 








 


                                 
#line 6166 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6331 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      
#line 6380 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6614 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6669 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      

















      






 

#line 6720 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6733 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"



 

 


 



 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

#line 6758 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void);







  



 
 
 
 


 




 
   
#line 6792 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"






 






 





 
#line 6818 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

 





 


      



      
#line 6843 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6853 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"




 



 

 


 


 
#line 6880 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"
      



























      



      


#line 6934 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6942 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 6962 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 7014 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 7035 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"

#line 7099 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc_ex.h"






      













 



 



  



   






 
#line 34 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"



 



 

 


 



 
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



 

 


 



 







 



 





 



 





 



 






 



 




 



 





 



 






 



 




 



 






 





 






 





 






 



 
#line 236 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



 







 



 
#line 289 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



 




 



 






 



 







 



 
#line 335 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 









 
 





 


 
#line 366 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



 

 


 







 
#line 428 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 435 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 452 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 459 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 519 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 527 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 545 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 553 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 620 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 629 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 







 
#line 648 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 657 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 




 
#line 672 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 680 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 




 
#line 696 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 705 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 




 
#line 722 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 732 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 








 
#line 750 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 757 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 








 
#line 776 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 784 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 








 
#line 804 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"

#line 813 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



 















 









 




 



 








 




 



 





















 
#line 912 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



 


















 
#line 955 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"


 



 



 
























 













 







 






 




 



 







 










 










 



 



 









 










 







 



 



 















 




















 




 




 











 












 













 













 




 



















 





 



 

 
 

 



 
 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
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



 



 

 
 
 


 




 

 
 



 


 



 
 



 



 
 




 


 


 


 












 



 

 


 



 






















#line 1413 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rcc.h"































 



 



 



 







 
#line 199 "..\\Inc\\stm32f4xx_hal_conf.h"


#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"



 



  

 


 



  
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


 

 



  



 
#line 103 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"




 










  







    



 





 




 






 

 


   





 
  


 

 


 






 







 







 







 







 



 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"



 



  

 
 


 
  


 

 
#line 166 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 281 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 387 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 483 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 

 



  








  





  






  






  






  








  





  






  




  






  




  




  


 

 
#line 682 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 

 
#line 816 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 
#line 908 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 
#line 982 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 
#line 1102 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1225 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 


  



 

 


 


 

 


 


 

 
 
 


 


 

 


 


 
#line 1277 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

#line 1291 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"







#line 1305 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

#line 1333 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"



 



   
 
#line 1367 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1394 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1418 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1441 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"

 

 
#line 1459 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"


 
 




 
#line 1486 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1518 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 
#line 1547 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio_ex.h"
 

 



 

 



 



  



 

 


 



 



  



  
  






 
#line 215 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"

 


 



 
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 



 
 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



  



  
 
 
 


 



 

 


 
#line 282 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_gpio.h"


 

 


 



 



  



 







 
#line 203 "..\\Inc\\stm32f4xx_hal_conf.h"


#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"



 



  

 




 
   


 
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
  HAL_DMA_FULL_TRANSFER           = 0x00U,   
  HAL_DMA_HALF_TRANSFER           = 0x01U    
}HAL_DMA_LevelCompleteTypeDef;



 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID         = 0x00U,   
  HAL_DMA_XFER_HALFCPLT_CB_ID     = 0x01U,   
  HAL_DMA_XFER_M1CPLT_CB_ID       = 0x02U,   
  HAL_DMA_XFER_M1HALFCPLT_CB_ID   = 0x03U,   
  HAL_DMA_XFER_ERROR_CB_ID        = 0x04U,   
  HAL_DMA_XFER_ABORT_CB_ID        = 0x05U,   
  HAL_DMA_XFER_ALL_CB_ID          = 0x06U    
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

  volatile uint32_t              ErrorCode;                                                         
  
  uint32_t                   StreamBaseAddress;                                                 

  uint32_t                   StreamIndex;                                                       
 
}DMA_HandleTypeDef;



 

 




 




  
#line 194 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"


 




  
#line 220 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"


 




  





 
        



  




  




  




 




  





  




 





 




  





 




 






  




 




  




 






  




  






  




  






 




 







 




  
#line 383 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"


 



 
 
 




 













 






 






 


 





 
#line 448 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





       
#line 468 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





 
#line 488 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





 
#line 508 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"





 
#line 528 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"













 

















 
















 














 














 




















 







 



 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma_ex.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma_ex.h"



 



  

 



 
   


  
typedef enum
{
  MEMORY0      = 0x00U,     
  MEMORY1      = 0x01U      
}HAL_DMA_MemoryTypeDef;



 

 



 




 

 
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);



 


 
         
 



 


 



 



 







 
#line 641 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"

 




 




 
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


  


  
 



 


  

 



 
#line 730 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_dma.h"

















































  

 



 


 



  



 







 
#line 207 "..\\Inc\\stm32f4xx_hal_conf.h"






#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"



 



  
 


 





 
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


 




 

 



 



 
#line 100 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 



 





 




 







 



 




 



 




 



 




 



 




 



 




 



 





 



 
#line 213 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 
   


 
#line 226 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 



 
#line 241 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"


 




 


 

 


 
  


 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);



 



 

 
 
 
 


 



































#line 347 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"

#line 356 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"

#line 385 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_cortex.h"






 

 



  



 
  





 

 
#line 215 "..\\Inc\\stm32f4xx_hal_conf.h"


#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"



 



  

 


 













 
typedef struct
{
  uint32_t ClockPrescaler;               

 
  uint32_t Resolution;                   
 
  uint32_t DataAlign;                    

 
  uint32_t ScanConvMode;                 





 
  uint32_t EOCSelection;                 





 
  FunctionalState ContinuousConvMode;    

 
  uint32_t NbrOfConversion;              

 
  FunctionalState DiscontinuousConvMode; 


 
  uint32_t NbrOfDiscConversion;          

 
  uint32_t ExternalTrigConv;             


 
  uint32_t ExternalTrigConvEdge;         

 
  FunctionalState DMAContinuousRequests; 



 
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

       
  FunctionalState ITMode;     

 
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
#line 218 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"
}ADC_HandleTypeDef;

#line 241 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"



 

 


 



 
#line 262 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


 




  






  



  
#line 297 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  



  






  



  






  



 
 
 
#line 345 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  



  




  



  
#line 380 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"





  



  
#line 398 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  

  

  





  



  




 



  
#line 431 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  
    


  






  
    


  
#line 455 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"


  



  





 



  

 


 




 
#line 493 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"





 






 







 







 






 







 







 




 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"



 



  

 


 
   











 
typedef struct 
{
  uint32_t InjectedChannel;                      

 
  uint32_t InjectedRank;                         

 
  uint32_t InjectedSamplingTime;                 







 
  uint32_t InjectedOffset;                       


 
  uint32_t InjectedNbrOfConversion;              



 
  FunctionalState InjectedDiscontinuousConvMode; 





 
  FunctionalState AutoInjectedConv;              






 
  uint32_t ExternalTrigInjecConv;                






 
  uint32_t ExternalTrigInjecConvEdge;            



 
}ADC_InjectionConfTypeDef; 



  
typedef struct
{
  uint32_t Mode;              
 
  uint32_t DMAAccessMode;     
 
  uint32_t TwoSamplingDelay;  
 
}ADC_MultiModeTypeDef;



 

 


 



  
#line 150 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"


  



  






  



  






  



  
#line 196 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"


  



  






 



 
#line 221 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"








  




  

 


 
#line 254 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"


  

 


 



 

 
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



  



 
 
 
 


 



 

 


 
#line 312 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"







#line 359 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc_ex.h"







 







 







 

 


 



 



  



 








 
#line 553 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"

 


 



 
 
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


 



 
 
 
 


 
 
 
 

 
 
 



 

 



 

 





 









 








 









 






 



    
#line 770 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"

#line 779 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_adc.h"





 







 







 







 







 







 






 






 






 






 






 






 




 

 


 



 



  



 








 
#line 219 "..\\Inc\\stm32f4xx_hal_conf.h"














#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_crc.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_crc.h"



 



 

 


 



 
typedef enum
{
  HAL_CRC_STATE_RESET     = 0x00U,   
  HAL_CRC_STATE_READY     = 0x01U,   
  HAL_CRC_STATE_BUSY      = 0x02U,   
  HAL_CRC_STATE_TIMEOUT   = 0x03U,   
  HAL_CRC_STATE_ERROR     = 0x04U    
} HAL_CRC_StateTypeDef;




 
typedef struct
{
  CRC_TypeDef                 *Instance;    

  HAL_LockTypeDef             Lock;         

  volatile HAL_CRC_StateTypeDef   State;        

} CRC_HandleTypeDef;


 

 


 



 

 


 




 






 








 







 



 


 


 



 

 


 

 


 
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);


 

 


 
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);


 

 


 
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);


 



 



 



 







 
#line 235 "..\\Inc\\stm32f4xx_hal_conf.h"














#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"



 



  

 


 
 


 
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



 

 


   



  
#line 97 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"


 
  


  






 




  
#line 126 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"


 
  



  




   



 







  



  







  



  
  
 


 





  






  





  





  





  





  





  





  






 








 










   









   
















 















 



 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"



 



  

 


 



 
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
 



#line 130 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
}FLASH_AdvOBProgramInitTypeDef;




 

 



 



  




 
  


  






 
  


  




 
  


  






 
  


 






  
  


  




  
  


  




  




  




     



   






 








  




 






  






#line 266 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


 



 
   
#line 293 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  




     
#line 311 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 



  
  



 
#line 327 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

#line 336 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


  
    


 





#line 356 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


  



 
    
#line 391 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

    
#line 412 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
       

  
#line 430 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  
#line 441 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  
#line 451 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 464 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 



  



 
   
#line 502 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

  
#line 524 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
     
      
  
#line 543 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 555 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 
 
 
#line 566 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 580 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 


 
  


 
    
#line 617 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 
      
 
#line 639 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
       

 
#line 651 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 662 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 

 
#line 677 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 



 
  


 







 



 
#line 708 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


 



  
  
 

 


 



 
 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);






HAL_StatusTypeDef HAL_FLASHEx_AdvOBProgram (FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
void              HAL_FLASHEx_AdvOBGetConfig(FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
HAL_StatusTypeDef HAL_FLASHEx_OB_SelectPCROP(void);
HAL_StatusTypeDef HAL_FLASHEx_OB_DeSelectPCROP(void);










 



 
 
 
 


 
  




 




  





  




  




 






  






 

 


 



 



























#line 849 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"







#line 863 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
  
#line 883 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

#line 898 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"







#line 913 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"
 
#line 928 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

#line 939 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"

#line 949 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"













#line 968 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"





  
























   


























#line 1034 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ex.h"


 



 

 


 
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
void FLASH_FlushCaches(void);


  



  



 







 
#line 298 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ramfunc.h"

















  

 



#line 75 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash_ramfunc.h"




 
#line 299 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_flash.h"

 


 


 
 
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


 



  
 
 


 



 
 


 



  



  



  



  



  




 

 


 



 






 



 

 


 



 



  



 







 
#line 251 "..\\Inc\\stm32f4xx_hal_conf.h"














#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"



 



 

 


 




 
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
 

} I2C_InitTypeDef;



 



























 
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

} HAL_I2C_StateTypeDef;



 


















 
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,    
  HAL_I2C_MODE_MASTER             = 0x10U,    
  HAL_I2C_MODE_SLAVE              = 0x20U,    
  HAL_I2C_MODE_MEM                = 0x40U     

} HAL_I2C_ModeTypeDef;



 




 
#line 176 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"


 




 
typedef struct __I2C_HandleTypeDef
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


#line 239 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"
} I2C_HandleTypeDef;

#line 269 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"


 



 
 



 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 
#line 354 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"



 




 






 





 



 

#line 397 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"


 



 

 



 




 
#line 424 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"









 











 
























 













 






 
#line 499 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"




 
#line 511 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"




 





 




 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c_ex.h"

















 

 








 
#line 31 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c_ex.h"



 



 

 
 


 



 




 



 

 
 


 



 
 
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);


 



 
 
 
 


 



 

 


 





 



 



 










 
#line 530 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);

 
#line 553 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"


 



 
 
 
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

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);


 



 
 
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



 



 
 
 
 


 





 

 


 

#line 664 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"













 
#line 700 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_i2c.h"








 



 

 


 



 



 



 








 
#line 267 "..\\Inc\\stm32f4xx_hal_conf.h"


















#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"



 



  

 



 
   


 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;



 

 


 
  


 



 



  
#line 86 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"


    
 


 
#line 100 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"


 




 




 
    


 




 



 




 



 







 



  
  
 


 





















 







 





 





 





 





 





 





 





 






 






 








 







 





 





 




 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



 



  

  
 


 
#line 66 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



 
#line 80 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"


 
#line 99 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



  
  
 


 

#line 127 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"









 
#line 145 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"

#line 193 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"


 

 


 
 


 
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



#line 228 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"



 



 
 
 
 


 



 
 
 
 



 



 


    
 



 



 

 



   
 
 





 



 

 


 



 






#line 310 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"

#line 321 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr_ex.h"


 



 



  



 
  







 
#line 275 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"

 


 
  


 
 
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


 



 

 
 
 


 



 



 



 
 







 



 
 
 



 



 




 



 
 
 




 



 
 


 



 
#line 408 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_pwr.h"


 



 



  



 
  







 
#line 287 "..\\Inc\\stm32f4xx_hal_conf.h"










#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"



 



 

 


 



 
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

#line 194 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

}RTC_HandleTypeDef;

#line 218 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"



 

 


 



 




 



 






 



 




 



 




 



 




 



 





 



 




 



 




 



 
 
#line 319 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 
#line 333 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 




 



 
#line 355 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 




 



 
#line 404 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 
#line 418 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 
#line 440 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 

 


 




 
#line 466 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"





 










 









 






 






 






 










 










 










 












 










 











 





 





 





 





 





 





 





 





 





 







 







 





 





 



 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"

















  

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"



 



  

  


 



 
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


 

 


  



 
#line 105 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


  



  




 
  


 







 



 








  



  







  



  






   



  


#line 177 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 



  
#line 200 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 



  
#line 215 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 
  


  




 
  


  




 



  
#line 246 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


  



  




 



  
#line 268 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


  



  







 



  




  


 

  




  



  
  
 


 

 


 





 






 









 









 









 









 










 









 





 





 





 





 





 





 





 





 





 








 







 





 





 




 

 


 





 






 









 









 









 









 










 









 




 

 


 





 






 

                                                                      





 






 


                                                                      








 









 










 










 



 

 


 




 





 





 





 





 





 





 





 





 








 







 





 





 



 

 


 





 






 






 






 






 






 









 



 



 

 


 



 
 
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


 



 

 
 
 


 




 

 


 



  
#line 920 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"







#line 933 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"

#line 970 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"













#line 989 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc_ex.h"


 



 



  



  
  






 
#line 672 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

 


 



 
 
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


 



 

 
 
 


 
 
#line 761 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"






 

 


 



 
#line 793 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

#line 824 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"

#line 841 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_rtc.h"


 



 

 


 
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
uint8_t            RTC_ByteToBcd2(uint8_t Value);
uint8_t            RTC_Bcd2ToByte(uint8_t Value);


 



 



 







 
#line 299 "..\\Inc\\stm32f4xx_hal_conf.h"














#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"



 



 

 


 



 
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
 
} SPI_InitTypeDef;



 
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00U,     
  HAL_SPI_STATE_READY      = 0x01U,     
  HAL_SPI_STATE_BUSY       = 0x02U,     
  HAL_SPI_STATE_BUSY_TX    = 0x03U,     
  HAL_SPI_STATE_BUSY_RX    = 0x04U,     
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,     
  HAL_SPI_STATE_ERROR      = 0x06U,     
  HAL_SPI_STATE_ABORT      = 0x07U      
} HAL_SPI_StateTypeDef;



 
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

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);    

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);    

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_SPI_StateTypeDef  State;           

  volatile uint32_t              ErrorCode;       

#line 150 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"
} SPI_HandleTypeDef;

#line 177 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"


 

 


 



 
#line 200 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"


 



 




 



 





 



 




 



 




 



 




 



 





 



 
#line 271 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"


 



 




 



 




 



 




 



 





 



 
#line 323 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"


 



 

 


 





 
#line 350 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"










 











 











 















 






 






 
#line 422 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"





 
#line 435 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"





 
#line 447 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"





 






 




 

 


 





 






 






 















 










 






 







 







 





 







 







 







 







 








 
#line 595 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_spi.h"





 







 







 







 





 




 

 


 



 
 
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

 






 



 
 
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);
 
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);


 



 
 
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);


 



 



 



 







 
#line 315 "..\\Inc\\stm32f4xx_hal_conf.h"






#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 

  uint32_t AutoReloadPreload;  
 
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
 
} TIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t ClearInputState;      
 
  uint32_t ClearInputSource;     
 
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;     
 
} TIM_ClearInputConfigTypeDef;



 
typedef struct
{
  uint32_t  MasterOutputTrigger;   
 
  uint32_t  MasterSlaveMode;       
 
} TIM_MasterConfigTypeDef;



 
typedef struct
{
  uint32_t  SlaveMode;         
 
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
 

} TIM_SlaveConfigTypeDef;





 
typedef struct
{
  uint32_t OffStateRunMode;      
 
  uint32_t OffStateIDLEMode;     
 
  uint32_t LockLevel;            
 
  uint32_t DeadTime;             
 
  uint32_t BreakState;           
 
  uint32_t BreakPolarity;        
 
  uint32_t BreakFilter;          
 
  uint32_t AutomaticOutput;      
 
} TIM_BreakDeadTimeConfigTypeDef;



 
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
} HAL_TIM_StateTypeDef;



 
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
} HAL_TIM_ActiveChannel;



 



typedef struct

{
  TIM_TypeDef                 *Instance;      
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;        
  DMA_HandleTypeDef           *hdma[7];      
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;          

#line 350 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"
} TIM_HandleTypeDef;

#line 394 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"



 
 

 


 



 




 



 
#line 437 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 
#line 452 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 





 



 




 



 






 



 







 



 





 



 




 



 





 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 







 



 






 



 




 



 





 



 
#line 644 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 




 



 
#line 667 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 
#line 686 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 







 



 
#line 715 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 







 



 






 



 




 



 






 



 




 



 




 


 






 



 




 



 




 



 





 



 
#line 829 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 




 



 







 



 
#line 865 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 
#line 881 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 







 



 






 



 




 



 
#line 938 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 
#line 952 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


 



 






 



 
 

 


 




 
#line 1002 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"





 






 






 
#line 1032 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"






 
#line 1049 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"






 















 















 














 














 



















 



















 
















 















 








 







 







 






 








 










 












 
#line 1262 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"








 



















 




















 

















 
















 
















 
















 













 













 

















 








 
 

 


 

 




 
 

 


 



#line 1476 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"


































































#line 1552 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"















































#line 1607 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"













#line 1626 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"

#line 1635 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"























#line 1676 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"

































 
 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"



 



 

 


 



 

typedef struct
{
  uint32_t IC1Polarity;         
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
 

  uint32_t Commutation_Delay;   
 
} TIM_HallSensor_InitTypeDef;


 
 

 


 



 
#line 84 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"












#line 108 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"


 



 
 

 


 



 
 

 


 
#line 193 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim_ex.h"



 
 

 


 




 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);


 




 
 
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);


 




 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);


 
 



 



 








 
#line 1714 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_tim.h"

 


 




 
 
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


 




 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef *sConfig);
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


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef *sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef *sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef *sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                              uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

 







 




 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);







 
 



 



 







 
#line 323 "..\\Inc\\stm32f4xx_hal_conf.h"






#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"

















 

 







 
#line 30 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  



 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 
 

  uint32_t OverSampling;              
 
} UART_InitTypeDef;







































 
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
 
} HAL_UART_StateTypeDef;



 
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef                 *Instance;         

  UART_InitTypeDef              Init;              

  uint8_t                       *pTxBuffPtr;       

  uint16_t                      TxXferSize;        

  volatile uint16_t                 TxXferCount;       

  uint8_t                       *pRxBuffPtr;       

  uint16_t                      RxXferSize;        

  volatile uint16_t                 RxXferCount;       

  DMA_HandleTypeDef             *hdmatx;           

  DMA_HandleTypeDef             *hdmarx;           

  HAL_LockTypeDef               Lock;              

  volatile HAL_UART_StateTypeDef    gState;           

 

  volatile HAL_UART_StateTypeDef    RxState;          
 

  volatile uint32_t                 ErrorCode;         

#line 188 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"

} UART_HandleTypeDef;

#line 218 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"



 

 


 



 
#line 240 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"


 



 




 



 




 



 





 



 






 



 





 



 




 



 




 



 




 



 




 





 
#line 344 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"


 









 













 



 

 


 






 
#line 400 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"





 



















 























 







 
#line 465 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"






 







 







 







 

















 



















 


















 
















 



















 



















 



















 









 





 





 





 



 

 


 



 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
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
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);



 



 
 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);


 



 
 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);


 



 
 
 
 


 


 







 

 


 
#line 800 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal_uart.h"





 








 






 

 


 



 



 



 







 
#line 331 "..\\Inc\\stm32f4xx_hal_conf.h"


























 
#line 373 "..\\Inc\\stm32f4xx_hal_conf.h"







 
#line 31 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"



 



  

 
 



 



 
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


 



 
   
 


 


 
#line 94 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

#line 117 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"


 



 





 




#line 141 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

#line 156 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"

#line 186 "..\\..\\..\\..\\..\\..\\Drivers\\STM32F4xx_HAL_Driver\\Inc\\stm32f4xx_hal.h"


 



 





 

 


 


 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);


 



 
 
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
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
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);







 



 
 
 


 


 
 


 


 
 
 


 



  
  






 
#line 43 "..\\Inc\\cube_hal.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\STM32F4xx-Nucleo\\stm32f4xx_nucleo.h"





































  
  
 







 
#line 50 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\STM32F4xx-Nucleo\\stm32f4xx_nucleo.h"
   
 
 

   


 



 



  



 
typedef enum 
{
  LED2 = 0
}Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
   
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

typedef enum 
{ 
  JOY_NONE  = 0,
  JOY_SEL   = 1,
  JOY_DOWN  = 2,
  JOY_LEFT  = 3,
  JOY_RIGHT = 4,
  JOY_UP    = 5
}JOYState_TypeDef;



  



  



  






 











  
  


   




 
#line 143 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\STM32F4xx-Nucleo\\stm32f4xx_nucleo.h"




 
#line 154 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\STM32F4xx-Nucleo\\stm32f4xx_nucleo.h"



  



 
 











#line 180 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\STM32F4xx-Nucleo\\stm32f4xx_nucleo.h"




    




 





 




     


 







 




    


 







 



 
  

  





   







 



 



   


  



 
uint32_t         BSP_GetVersion(void);  
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);

uint8_t          BSP_JOY_Init(void);
JOYState_TypeDef BSP_JOY_GetState(void);
void             BSP_JOY_DeInit(void);


  


  



  



 



 







 
#line 44 "..\\Inc\\cube_hal.h"
#line 45 "..\\Inc\\cube_hal.h"
#line 46 "..\\Inc\\cube_hal.h"


#line 54 "..\\Inc\\cube_hal.h"

#line 61 "..\\Inc\\cube_hal.h"

#line 68 "..\\Inc\\cube_hal.h"












#line 90 "..\\Inc\\cube_hal.h"

#line 101 "..\\Inc\\cube_hal.h"

#line 112 "..\\Inc\\cube_hal.h"

void SystemClock_Config(void);
uint32_t Get_DMA_Flag_Status(DMA_HandleTypeDef *handle_dma);
uint32_t Get_DMA_Counter(DMA_HandleTypeDef *handle_dma);
void Config_DMA_Handler(DMA_HandleTypeDef *handle_dma);



 
#line 46 "..\\Inc\\main.h"

#line 1 "..\\Inc\\nucleo_f401re_bus.h"














































 

 










#line 67 "..\\Inc\\nucleo_f401re_bus.h"

#line 75 "..\\Inc\\nucleo_f401re_bus.h"


 
#line 79 "..\\Inc\\nucleo_f401re_bus.h"
 
int32_t BSP_I2C1_Init(void);
int32_t BSP_I2C3_Init(void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C1_IsReady(void);
int32_t BSP_I2C1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_WriteReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_ReadReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);

int32_t BSP_GetTick(void);












 
#line 49 "..\\Inc\\main.h"
#line 1 "..\\Inc\\nucleo_f401re_errno.h"














































 

 







 
#line 69 "..\\Inc\\nucleo_f401re_errno.h"







 
#line 50 "..\\Inc\\main.h"

#line 66 "..\\Inc\\main.h"

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors.h"











































 

 







 
#line 1 "..\\Inc\\iks01a2_conf.h"



































 

#line 40 "..\\Inc\\iks01a2_conf.h"
#line 41 "..\\Inc\\iks01a2_conf.h"
#line 42 "..\\Inc\\iks01a2_conf.h"

#line 61 "..\\Inc\\iks01a2_conf.h"

 


























 

#line 56 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\Common\\env_sensor.h"


















 

 









 
#line 33 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\Common\\env_sensor.h"



 



 



 



 



 



 
typedef struct
{
  int32_t ( *Init              ) ( void * );
  int32_t ( *DeInit            ) ( void * );
  int32_t ( *ReadID            ) ( void *, uint8_t * ); 
  int32_t ( *GetCapabilities   ) ( void *, void * );
} ENV_SENSOR_CommonDrv_t;

typedef struct
{
  int32_t ( *Enable            ) ( void * );
  int32_t ( *Disable           ) ( void * );
  int32_t ( *GetOutputDataRate ) ( void *, float * );
  int32_t ( *SetOutputDataRate ) ( void *, float );
  int32_t ( *GetValue          ) ( void *, float * );
} ENV_SENSOR_FuncDrv_t;



 



 



 



 



 







 
#line 57 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors.h"

















#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.h"

































 

 








 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"



































 
 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"




 





 












 






   









 






#line 61 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 75 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   




 















 
#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











 





extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double  , double  );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double  );
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 230 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   
  typedef float float_t;
  typedef double double_t;
#line 251 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"



extern const int math_errhandling;
#line 261 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    inline double _sqrt(double __x) { return sqrt(__x); }


    inline float _sqrtf(float __x) { return __sqrtf(__x); }



    



 

extern __declspec(__nothrow) __attribute__((const)) double ceil(double  );
    
    
extern __declspec(__nothrow) __attribute__((const)) double fabs(double  );
    
    

extern __declspec(__nothrow) __attribute__((const)) double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
inline __declspec(__nothrow) __attribute__((const)) double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
inline __declspec(__nothrow) __attribute__((const)) float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 479 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __attribute__((const)) double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __attribute__((const)) float _fabsf(float);  
inline __declspec(__nothrow) __attribute__((const)) float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __attribute__((const)) float ceilf(float  );
extern __declspec(__nothrow) __attribute__((const)) float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );






 
extern __declspec(__nothrow) double exp2(double  );  
extern __declspec(__nothrow) float exp2f(float  );
__declspec(__nothrow) long double exp2l(long double );
extern __declspec(__nothrow) double fdim(double  , double  );
extern __declspec(__nothrow) float fdimf(float  , float  );
__declspec(__nothrow) long double fdiml(long double , long double );
#line 803 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) double fma(double  , double  , double  );
extern __declspec(__nothrow) float fmaf(float  , float  , float  );

inline __declspec(__nothrow) long double fmal(long double __x, long double __y, long double __z)     { return (long double)fma((double)__x, (double)__y, (double)__z); }


extern __declspec(__nothrow) __attribute__((const)) double fmax(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fmaxf(float  , float  );
__declspec(__nothrow) long double fmaxl(long double , long double );
extern __declspec(__nothrow) __attribute__((const)) double fmin(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fminf(float  , float  );
__declspec(__nothrow) long double fminl(long double , long double );
extern __declspec(__nothrow) double log2(double  );  
extern __declspec(__nothrow) float log2f(float  );
__declspec(__nothrow) long double log2l(long double );
extern __declspec(__nothrow) long lrint(double  );
extern __declspec(__nothrow) long lrintf(float  );

inline __declspec(__nothrow) long lrintl(long double __x)     { return lrint((double)__x); }


extern __declspec(__nothrow) long long llrint(double  );
extern __declspec(__nothrow) long long llrintf(float  );

inline __declspec(__nothrow) long long llrintl(long double __x)     { return llrint((double)__x); }


extern __declspec(__nothrow) long lround(double  );
extern __declspec(__nothrow) long lroundf(float  );

inline __declspec(__nothrow) long lroundl(long double __x)     { return lround((double)__x); }


extern __declspec(__nothrow) long long llround(double  );
extern __declspec(__nothrow) long long llroundf(float  );

inline __declspec(__nothrow) long long llroundl(long double __x)     { return llround((double)__x); }


extern __declspec(__nothrow) __attribute__((const)) double nan(const char * );
extern __declspec(__nothrow) __attribute__((const)) float nanf(const char * );

inline __declspec(__nothrow) __attribute__((const)) long double nanl(const char *__t)     { return (long double)nan(__t); }
#line 856 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) __attribute__((const)) double nearbyint(double  );
extern __declspec(__nothrow) __attribute__((const)) float nearbyintf(float  );
__declspec(__nothrow) long double nearbyintl(long double );
extern  double remquo(double  , double  , int * );
extern  float remquof(float  , float  , int * );

inline long double remquol(long double __x, long double __y, int *__q)     { return (long double)remquo((double)__x, (double)__y, __q); }


extern __declspec(__nothrow) __attribute__((const)) double round(double  );
extern __declspec(__nothrow) __attribute__((const)) float roundf(float  );
__declspec(__nothrow) long double roundl(long double );
extern __declspec(__nothrow) double tgamma(double  );  
extern __declspec(__nothrow) float tgammaf(float  );
__declspec(__nothrow) long double tgammal(long double );
extern __declspec(__nothrow) __attribute__((const)) double trunc(double  );
extern __declspec(__nothrow) __attribute__((const)) float truncf(float  );
__declspec(__nothrow) long double truncl(long double );






#line 896 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 1087 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











#line 1317 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"





 
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"




 




 














 

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} axis3bit32_t;

typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;




 

typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;









 







 

typedef int32_t (*lps22hb_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lps22hb_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  lps22hb_write_ptr  write_reg;
  lps22hb_read_ptr   read_reg;
   
  void *handle;
} lps22hb_ctx_t;




 





 

   



 





 


typedef struct {
  uint8_t pe               : 2;  
  uint8_t lir              : 1;
  uint8_t diff_en          : 1;
  uint8_t reset_az         : 1;
  uint8_t autozero         : 1;
  uint8_t reset_arp        : 1;
  uint8_t autorifp         : 1;
} lps22hb_interrupt_cfg_t;





typedef struct {
  uint8_t sim              : 1;
  uint8_t bdu              : 1;
  uint8_t lpfp             : 2;  
  uint8_t odr              : 3;
  uint8_t not_used_01      : 1;
} lps22hb_ctrl_reg1_t;


typedef struct {
  uint8_t one_shot         : 1;
  uint8_t not_used_01      : 1;
  uint8_t swreset          : 1;
  uint8_t i2c_dis          : 1;
  uint8_t if_add_inc       : 1;
  uint8_t stop_on_fth      : 1;
  uint8_t fifo_en          : 1;
  uint8_t boot             : 1;
} lps22hb_ctrl_reg2_t;


typedef struct {
  uint8_t int_s            : 2;
  uint8_t drdy             : 1;
  uint8_t f_ovr            : 1;
  uint8_t f_fth            : 1;
  uint8_t f_fss5           : 1;
  uint8_t pp_od            : 1;
  uint8_t int_h_l          : 1;
} lps22hb_ctrl_reg3_t;



typedef struct {
  uint8_t wtm              : 5;
  uint8_t f_mode           : 3;
} lps22hb_fifo_ctrl_t;








typedef struct {
  uint8_t lc_en            : 1;
  uint8_t not_used_01      : 7;
} lps22hb_res_conf_t;


typedef struct {
  uint8_t ph               : 1;
  uint8_t pl               : 1;
  uint8_t ia               : 1;
  uint8_t not_used_01      : 4;
  uint8_t boot_status      : 1;
} lps22hb_int_source_t;


typedef struct {
  uint8_t fss              : 6;
  uint8_t ovr              : 1;
  uint8_t fth_fifo         : 1;
} lps22hb_fifo_status_t;


typedef struct {
  uint8_t p_da             : 1;
  uint8_t t_da             : 1;
  uint8_t not_used_02      : 2;
  uint8_t p_or             : 1;
  uint8_t t_or             : 1;
  uint8_t not_used_01      : 2;
} lps22hb_status_t;

#line 259 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"












 

typedef union{
  lps22hb_interrupt_cfg_t      interrupt_cfg;
  lps22hb_ctrl_reg1_t          ctrl_reg1;
  lps22hb_ctrl_reg2_t          ctrl_reg2;
  lps22hb_ctrl_reg3_t          ctrl_reg3;
  lps22hb_fifo_ctrl_t          fifo_ctrl;
  lps22hb_res_conf_t           res_conf;
  lps22hb_int_source_t         int_source;
  lps22hb_fifo_status_t        fifo_status;
  lps22hb_status_t             status;
  bitwise_t                    bitwise;
  uint8_t                      byte;
} lps22hb_reg_t;




 

int32_t lps22hb_read_reg(lps22hb_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lps22hb_write_reg(lps22hb_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t lps22hb_from_lsb_to_hpa(int32_t lsb);
extern float_t lps22hb_from_lsb_to_degc(int16_t lsb);

int32_t lps22hb_autozero_rst_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_autozero_rst_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_autozero_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_autozero_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_snap_rst_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_pressure_snap_rst_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_snap_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_pressure_snap_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_block_data_update_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_block_data_update_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_LPF_ODR_DIV_2  = 0,
  LPS22HB_LPF_ODR_DIV_9  = 2,
  LPS22HB_LPF_ODR_DIV_20 = 3,
} lps22hb_lpfp_t;
int32_t lps22hb_low_pass_filter_mode_set(lps22hb_ctx_t *ctx,
                                         lps22hb_lpfp_t val);
int32_t lps22hb_low_pass_filter_mode_get(lps22hb_ctx_t *ctx,
                                         lps22hb_lpfp_t *val);

typedef enum {
  LPS22HB_POWER_DOWN  = 0,
  LPS22HB_ODR_1_Hz    = 1,
  LPS22HB_ODR_10_Hz   = 2,
  LPS22HB_ODR_25_Hz   = 3,
  LPS22HB_ODR_50_Hz   = 4,
  LPS22HB_ODR_75_Hz   = 5,
} lps22hb_odr_t;
int32_t lps22hb_data_rate_set(lps22hb_ctx_t *ctx, lps22hb_odr_t val);
int32_t lps22hb_data_rate_get(lps22hb_ctx_t *ctx, lps22hb_odr_t *val);

int32_t lps22hb_one_shoot_trigger_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_one_shoot_trigger_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_ref_set(lps22hb_ctx_t *ctx, uint8_t *buff);
int32_t lps22hb_pressure_ref_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_pressure_offset_set(lps22hb_ctx_t *ctx, uint8_t *buff);
int32_t lps22hb_pressure_offset_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_press_data_ready_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_temp_data_ready_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_press_data_ovr_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_temp_data_ovr_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_raw_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_temperature_raw_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_low_pass_rst_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_device_id_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_reset_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_reset_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_boot_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_boot_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_low_power_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_low_power_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_boot_status_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef struct{
  lps22hb_fifo_status_t  fifo_status;
  lps22hb_status_t       status;
} lps22hb_dev_stat_t;
int32_t lps22hb_dev_status_get(lps22hb_ctx_t *ctx, lps22hb_dev_stat_t *val);

typedef enum {
  LPS22HB_NO_THRESHOLD = 0,
  LPS22HB_POSITIVE     = 1,
  LPS22HB_NEGATIVE     = 2,
  LPS22HB_BOTH         = 3,
} lps22hb_pe_t;
int32_t lps22hb_sign_of_int_threshold_set(lps22hb_ctx_t *ctx,
                                          lps22hb_pe_t val);
int32_t lps22hb_sign_of_int_threshold_get(lps22hb_ctx_t *ctx,
                                          lps22hb_pe_t *val);

typedef enum {
  LPS22HB_INT_PULSED  = 0,
  LPS22HB_INT_LATCHED = 1,
} lps22hb_lir_t;
int32_t lps22hb_int_notification_mode_set(lps22hb_ctx_t *ctx,
                                          lps22hb_lir_t val);
int32_t lps22hb_int_notification_mode_get(lps22hb_ctx_t *ctx,
                                          lps22hb_lir_t *val);

int32_t lps22hb_int_generation_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_int_generation_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_int_threshold_set(lps22hb_ctx_t *ctx, uint8_t *buff);
int32_t lps22hb_int_threshold_get(lps22hb_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LPS22HB_DRDY_OR_FIFO_FLAGS = 0,
  LPS22HB_HIGH_PRES_INT      = 1,
  LPS22HB_LOW_PRES_INT       = 2,
  LPS22HB_EVERY_PRES_INT     = 3,
} lps22hb_int_s_t;
int32_t lps22hb_int_pin_mode_set(lps22hb_ctx_t *ctx, lps22hb_int_s_t val);
int32_t lps22hb_int_pin_mode_get(lps22hb_ctx_t *ctx, lps22hb_int_s_t *val);

int32_t lps22hb_drdy_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_drdy_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_ovr_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_ovr_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_threshold_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_threshold_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_full_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_full_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_PUSH_PULL  = 0,
  LPS22HB_OPEN_DRAIN = 1,
} lps22hb_pp_od_t;
int32_t lps22hb_pin_mode_set(lps22hb_ctx_t *ctx, lps22hb_pp_od_t val);
int32_t lps22hb_pin_mode_get(lps22hb_ctx_t *ctx, lps22hb_pp_od_t *val);

typedef enum {
  LPS22HB_ACTIVE_HIGH = 0,
  LPS22HB_ACTIVE_LOW = 1,
} lps22hb_int_h_l_t;
int32_t lps22hb_int_polarity_set(lps22hb_ctx_t *ctx, lps22hb_int_h_l_t val);
int32_t lps22hb_int_polarity_get(lps22hb_ctx_t *ctx, lps22hb_int_h_l_t *val);

int32_t lps22hb_int_source_get(lps22hb_ctx_t *ctx, lps22hb_int_source_t *val);

int32_t lps22hb_int_on_press_high_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_int_on_press_low_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_interrupt_event_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_stop_on_fifo_threshold_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_stop_on_fifo_threshold_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_watermark_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_watermark_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_BYPASS_MODE           = 0,
  LPS22HB_FIFO_MODE             = 1,
  LPS22HB_STREAM_MODE           = 2,
  LPS22HB_STREAM_TO_FIFO_MODE   = 3,
  LPS22HB_BYPASS_TO_STREAM_MODE = 4,
  LPS22HB_DYNAMIC_STREAM_MODE   = 6,
  LPS22HB_BYPASS_TO_FIFO_MODE   = 7,
} lps22hb_f_mode_t;
int32_t lps22hb_fifo_mode_set(lps22hb_ctx_t *ctx, lps22hb_f_mode_t val);
int32_t lps22hb_fifo_mode_get(lps22hb_ctx_t *ctx, lps22hb_f_mode_t *val);

int32_t lps22hb_fifo_data_level_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_ovr_flag_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_fth_flag_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_SPI_4_WIRE = 0,
  LPS22HB_SPI_3_WIRE = 1,
} lps22hb_sim_t;
int32_t lps22hb_spi_mode_set(lps22hb_ctx_t *ctx, lps22hb_sim_t val);
int32_t lps22hb_spi_mode_get(lps22hb_ctx_t *ctx, lps22hb_sim_t *val);

typedef enum {
  LPS22HB_I2C_ENABLE = 0,
  LPS22HB_I2C_DISABLE = 1,
} lps22hb_i2c_dis_t;
int32_t lps22hb_i2c_interface_set(lps22hb_ctx_t *ctx, lps22hb_i2c_dis_t val);
int32_t lps22hb_i2c_interface_get(lps22hb_ctx_t *ctx, lps22hb_i2c_dis_t *val);

int32_t lps22hb_auto_add_inc_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_auto_add_inc_get(lps22hb_ctx_t *ctx, uint8_t *val);




 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.h"



 



 



 



 

typedef int32_t (*LPS22HB_Init_Func)(void);
typedef int32_t (*LPS22HB_DeInit_Func)(void);
typedef int32_t (*LPS22HB_GetTick_Func)(void);
typedef int32_t (*LPS22HB_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LPS22HB_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LPS22HB_Init_Func          Init;
  LPS22HB_DeInit_Func        DeInit;
  uint32_t                   BusType;  
  uint8_t                    Address;
  LPS22HB_WriteReg_Func      WriteReg;
  LPS22HB_ReadReg_Func       ReadReg;
  LPS22HB_GetTick_Func       GetTick;
} LPS22HB_IO_t;

typedef struct
{
  LPS22HB_IO_t        IO;
  lps22hb_ctx_t       Ctx;
  uint8_t             is_initialized;
  uint8_t             press_is_enabled;
  uint8_t             temp_is_enabled;
  lps22hb_odr_t       last_odr;
} LPS22HB_Object_t;

typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} LPS22HB_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LPS22HB_Object_t *);
  int32_t (*DeInit)(LPS22HB_Object_t *);
  int32_t (*ReadID)(LPS22HB_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LPS22HB_Object_t *, LPS22HB_Capabilities_t *);
} LPS22HB_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LPS22HB_Object_t *);
  int32_t (*Disable)(LPS22HB_Object_t *);
  int32_t (*GetOutputDataRate)(LPS22HB_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS22HB_Object_t *, float);
  int32_t (*GetTemperature)(LPS22HB_Object_t *, float *);
} LPS22HB_TEMP_Drv_t;

typedef struct
{
  int32_t (*Enable)(LPS22HB_Object_t *);
  int32_t (*Disable)(LPS22HB_Object_t *);
  int32_t (*GetOutputDataRate)(LPS22HB_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS22HB_Object_t *, float);
  int32_t (*GetPressure)(LPS22HB_Object_t *, float *);
} LPS22HB_PRESS_Drv_t;

typedef enum
{
  LPS22HB_FIFO_BYPASS_MODE                    = (uint8_t)0x00,     
  LPS22HB_FIFO_FIFO_MODE                      = (uint8_t)0x20,     
  LPS22HB_FIFO_STREAM_MODE                    = (uint8_t)0x40,     
  LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,     
  LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,     
  LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0      
} LPS22HB_FifoMode;



 



 












 



 

int32_t LPS22HB_RegisterBusIO(LPS22HB_Object_t *pObj, LPS22HB_IO_t *pIO);
int32_t LPS22HB_Init(LPS22HB_Object_t *pObj);
int32_t LPS22HB_DeInit(LPS22HB_Object_t *pObj);
int32_t LPS22HB_ReadID(LPS22HB_Object_t *pObj, uint8_t *Id);
int32_t LPS22HB_GetCapabilities(LPS22HB_Object_t *pObj, LPS22HB_Capabilities_t *Capabilities);
int32_t LPS22HB_Get_Init_Status(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_PRESS_Enable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_PRESS_Disable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_PRESS_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
int32_t LPS22HB_PRESS_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr);
int32_t LPS22HB_PRESS_GetPressure(LPS22HB_Object_t *pObj, float *Value);
int32_t LPS22HB_PRESS_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_PRESS_Get_FThStatus(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_TEMP_Enable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_TEMP_Disable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_TEMP_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
int32_t LPS22HB_TEMP_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr);
int32_t LPS22HB_TEMP_GetTemperature(LPS22HB_Object_t *pObj, float *Value);
int32_t LPS22HB_TEMP_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_FIFO_Get_Data(LPS22HB_Object_t *pObj, float *Press, float *Temp);
int32_t LPS22HB_FIFO_Get_FTh_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Full_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Level(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Ovr_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Reset_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt);
int32_t LPS22HB_FIFO_Set_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt);
int32_t LPS22HB_FIFO_Set_Mode(LPS22HB_Object_t *pObj, uint8_t Mode);
int32_t LPS22HB_FIFO_Set_Watermark_Level(LPS22HB_Object_t *pObj, uint8_t Watermark);
int32_t LPS22HB_FIFO_Usage(LPS22HB_Object_t *pObj, uint8_t Status);


int32_t LPS22HB_Read_Reg(LPS22HB_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LPS22HB_Write_Reg(LPS22HB_Object_t *pObj, uint8_t reg, uint8_t Data);

int32_t LPS22HB_Get_Press(LPS22HB_Object_t *pObj, float *Data);
int32_t LPS22HB_Get_Temp(LPS22HB_Object_t *pObj, float *Data);

int32_t LPS22HB_Set_One_Shot(LPS22HB_Object_t *pObj);
int32_t LPS22HB_Get_One_Shot_Status(LPS22HB_Object_t *pObj, uint8_t *Status);



 



 
extern LPS22HB_CommonDrv_t LPS22HB_COMMON_Driver;
extern LPS22HB_PRESS_Drv_t LPS22HB_PRESS_Driver;
extern LPS22HB_TEMP_Drv_t LPS22HB_TEMP_Driver;



 









 



 



 

 
#line 76 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors.h"


#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221.h"

































 

 








 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"



































 

 







 
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"
#line 49 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"




 




 

#line 115 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"




 







 

typedef int32_t (*hts221_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*hts221_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  hts221_write_ptr  write_reg;
  hts221_read_ptr   read_reg;
   
  void *handle;
} hts221_ctx_t;




 




 

 


 





 



typedef struct {
  uint8_t avgh                 : 3;
  uint8_t avgt                 : 3;
  uint8_t not_used_01          : 2;
} hts221_av_conf_t;


typedef struct {
  uint8_t odr                  : 2;
  uint8_t bdu                  : 1;
  uint8_t not_used_01          : 4;
  uint8_t pd                   : 1;
} hts221_ctrl_reg1_t;


typedef struct {
  uint8_t one_shot             : 1;
  uint8_t heater               : 1;
  uint8_t not_used_01          : 5;
  uint8_t boot                 : 1;
} hts221_ctrl_reg2_t;


typedef struct {
  uint8_t not_used_01          : 2;
  uint8_t drdy                 : 1;
  uint8_t not_used_02          : 3;
  uint8_t pp_od                : 1;
  uint8_t drdy_h_l             : 1;
} hts221_ctrl_reg3_t;


typedef struct {
  uint8_t t_da                 : 1;
  uint8_t h_da                 : 1;
  uint8_t not_used_01          : 6;
} hts221_status_reg_t;

#line 210 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"
typedef struct {
  uint8_t t0_msb               : 2;
  uint8_t t1_msb               : 2;
  uint8_t not_used_01          : 4;
} hts221_t1_t0_msb_t;

#line 224 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"












 
typedef union{
  hts221_av_conf_t        av_conf;
  hts221_ctrl_reg1_t      ctrl_reg1;
  hts221_ctrl_reg2_t      ctrl_reg2;
  hts221_ctrl_reg3_t      ctrl_reg3;
  hts221_status_reg_t     status_reg;
  hts221_t1_t0_msb_t      t1_t0_msb;
  bitwise_t               bitwise;
  uint8_t                 byte;
} hts221_reg_t;




 

int32_t hts221_read_reg(hts221_ctx_t *ctx, uint8_t reg, uint8_t* data,
                        uint16_t len);
int32_t hts221_write_reg(hts221_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);

typedef enum {
  HTS221_H_AVG_4    = 0,
  HTS221_H_AVG_8    = 1,
  HTS221_H_AVG_16   = 2,
  HTS221_H_AVG_32   = 3,
  HTS221_H_AVG_64   = 4,
  HTS221_H_AVG_128  = 5,
  HTS221_H_AVG_256  = 6,
  HTS221_H_AVG_512  = 7,
  HTS221_H_AVG_ND   = 8,
} hts221_avgh_t;
int32_t hts221_humidity_avg_set(hts221_ctx_t *ctx, hts221_avgh_t val);
int32_t hts221_humidity_avg_get(hts221_ctx_t *ctx, hts221_avgh_t *val);

typedef enum {
  HTS221_T_AVG_2   = 0,
  HTS221_T_AVG_4   = 1,
  HTS221_T_AVG_8   = 2,
  HTS221_T_AVG_16  = 3,
  HTS221_T_AVG_32  = 4,
  HTS221_T_AVG_64  = 5,
  HTS221_T_AVG_128 = 6,
  HTS221_T_AVG_256 = 7,
  HTS221_T_AVG_ND  = 8,
} hts221_avgt_t;
int32_t hts221_temperature_avg_set(hts221_ctx_t *ctx, hts221_avgt_t val);
int32_t hts221_temperature_avg_get(hts221_ctx_t *ctx, hts221_avgt_t *val);

typedef enum {
  HTS221_ONE_SHOT  = 0,
  HTS221_ODR_1Hz   = 1,
  HTS221_ODR_7Hz   = 2,
  HTS221_ODR_12Hz5 = 3,
  HTS221_ODR_ND    = 4,
} hts221_odr_t;
int32_t hts221_data_rate_set(hts221_ctx_t *ctx, hts221_odr_t val);
int32_t hts221_data_rate_get(hts221_ctx_t *ctx, hts221_odr_t *val);

int32_t hts221_block_data_update_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_block_data_update_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_one_shoot_trigger_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_one_shoot_trigger_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_temp_data_ready_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_hum_data_ready_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_humidity_raw_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temperature_raw_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_device_id_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_power_on_set(hts221_ctx_t *ctx, uint8_t val);

int32_t hts221_power_on_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_heater_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_heater_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_boot_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_boot_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_status_get(hts221_ctx_t *ctx, hts221_status_reg_t *val);

int32_t hts221_drdy_on_int_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_drdy_on_int_get(hts221_ctx_t *ctx, uint8_t *val);

typedef enum {
  HTS221_PUSH_PULL   = 0,
  HTS221_OPEN_DRAIN  = 1,
  HTS221_PIN_MODE_ND = 2,
} hts221_pp_od_t;
int32_t hts221_pin_mode_set(hts221_ctx_t *ctx, hts221_pp_od_t val);
int32_t hts221_pin_mode_get(hts221_ctx_t *ctx, hts221_pp_od_t *val);

typedef enum {
  HTS221_ACTIVE_HIGH = 0,
  HTS221_ACTIVE_LOW  = 1,
  HTS221_ACTIVE_ND   = 2,
} hts221_drdy_h_l_t;
int32_t hts221_int_polarity_set(hts221_ctx_t *ctx, hts221_drdy_h_l_t val);
int32_t hts221_int_polarity_get(hts221_ctx_t *ctx, hts221_drdy_h_l_t *val);

int32_t hts221_hum_rh_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_hum_rh_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temp_deg_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_temp_deg_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_hum_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_hum_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temp_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_temp_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);




 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221.h"
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221.h"



 



 



 



 

typedef int32_t (*HTS221_Init_Func)(void);
typedef int32_t (*HTS221_DeInit_Func)(void);
typedef int32_t (*HTS221_GetTick_Func)(void);
typedef int32_t (*HTS221_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*HTS221_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  HTS221_Init_Func          Init;
  HTS221_DeInit_Func        DeInit;
  uint32_t                  BusType;  
  uint8_t                   Address;
  HTS221_WriteReg_Func      WriteReg;
  HTS221_ReadReg_Func       ReadReg;
  HTS221_GetTick_Func       GetTick;
} HTS221_IO_t;

typedef struct
{
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;

typedef struct
{
  HTS221_IO_t        IO;
  hts221_ctx_t       Ctx;
  uint8_t            is_initialized;
  uint8_t            hum_is_enabled;
  uint8_t            temp_is_enabled;
} HTS221_Object_t;

typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} HTS221_Capabilities_t;

typedef struct
{
  int32_t (*Init)(HTS221_Object_t *);
  int32_t (*DeInit)(HTS221_Object_t *);
  int32_t (*ReadID)(HTS221_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(HTS221_Object_t *, HTS221_Capabilities_t *);
} HTS221_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(HTS221_Object_t *);
  int32_t (*Disable)(HTS221_Object_t *);
  int32_t (*GetOutputDataRate)(HTS221_Object_t *, float *);
  int32_t (*SetOutputDataRate)(HTS221_Object_t *, float);
  int32_t (*GetHumidity)(HTS221_Object_t *, float *);
} HTS221_HUM_Drv_t;

typedef struct
{
  int32_t (*Enable)(HTS221_Object_t *);
  int32_t (*Disable)(HTS221_Object_t *);
  int32_t (*GetOutputDataRate)(HTS221_Object_t *, float *);
  int32_t (*SetOutputDataRate)(HTS221_Object_t *, float);
  int32_t (*GetTemperature)(HTS221_Object_t *, float *);
} HTS221_TEMP_Drv_t;



 



 



 





 



 

int32_t HTS221_RegisterBusIO(HTS221_Object_t *pObj, HTS221_IO_t *pIO);
int32_t HTS221_Init(HTS221_Object_t *pObj);
int32_t HTS221_DeInit(HTS221_Object_t *pObj);
int32_t HTS221_ReadID(HTS221_Object_t *pObj, uint8_t *Id);
int32_t HTS221_GetCapabilities(HTS221_Object_t *pObj, HTS221_Capabilities_t *Capabilities);
int32_t HTS221_Get_Init_Status(HTS221_Object_t *pObj, uint8_t *Status);

int32_t HTS221_HUM_Enable(HTS221_Object_t *pObj);
int32_t HTS221_HUM_Disable(HTS221_Object_t *pObj);
int32_t HTS221_HUM_GetOutputDataRate(HTS221_Object_t *pObj, float *Odr);
int32_t HTS221_HUM_SetOutputDataRate(HTS221_Object_t *pObj, float Odr);
int32_t HTS221_HUM_GetHumidity(HTS221_Object_t *pObj, float *Value);
int32_t HTS221_HUM_Get_DRDY_Status(HTS221_Object_t *pObj, uint8_t *Status);

int32_t HTS221_TEMP_Enable(HTS221_Object_t *pObj);
int32_t HTS221_TEMP_Disable(HTS221_Object_t *pObj);
int32_t HTS221_TEMP_GetOutputDataRate(HTS221_Object_t *pObj, float *Odr);
int32_t HTS221_TEMP_SetOutputDataRate(HTS221_Object_t *pObj, float Odr);
int32_t HTS221_TEMP_GetTemperature(HTS221_Object_t *pObj, float *Value);
int32_t HTS221_TEMP_Get_DRDY_Status(HTS221_Object_t *pObj, uint8_t *Status);

int32_t HTS221_Read_Reg(HTS221_Object_t *pObj, uint8_t Reg, uint8_t *Data);
int32_t HTS221_Write_Reg(HTS221_Object_t *pObj, uint8_t Reg, uint8_t Data);

int32_t HTS221_Set_One_Shot(HTS221_Object_t *pObj);
int32_t HTS221_Get_One_Shot_Status(HTS221_Object_t *pObj, uint8_t *Status);

int32_t HTS221_Enable_DRDY_Interrupt(HTS221_Object_t *pObj);



 



 

extern HTS221_CommonDrv_t HTS221_COMMON_Driver;
extern HTS221_HUM_Drv_t HTS221_HUM_Driver;
extern HTS221_TEMP_Drv_t HTS221_TEMP_Driver;



 









 



 



 

 
#line 80 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors.h"












 



 



 



 

 
typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} IKS01A2_ENV_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} IKS01A2_ENV_SENSOR_Ctx_t;



 



 




















#line 159 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors.h"













 



 

int32_t IKS01A2_ENV_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t IKS01A2_ENV_SENSOR_DeInit(uint32_t Instance);
int32_t IKS01A2_ENV_SENSOR_GetCapabilities(uint32_t Instance, IKS01A2_ENV_SENSOR_Capabilities_t *Capabilities);
int32_t IKS01A2_ENV_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t IKS01A2_ENV_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_ENV_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_ENV_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t IKS01A2_ENV_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t IKS01A2_ENV_SENSOR_GetValue(uint32_t Instance, uint32_t Function, float *Value);



 



 



 



 







 
#line 69 "..\\Inc\\main.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors_ex.h"











































 

 







 
#line 56 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_env_sensors_ex.h"



 



 



 



 

int32_t IKS01A2_ENV_SENSOR_FIFO_Get_Data(uint32_t Instance, float *Press, float *Temp);
int32_t IKS01A2_ENV_SENSOR_FIFO_Get_Fth_Status(uint32_t Instance, uint8_t *Status);
int32_t IKS01A2_ENV_SENSOR_FIFO_Get_Full_Status(uint32_t Instance, uint8_t *Status);
int32_t IKS01A2_ENV_SENSOR_FIFO_Get_Num_Samples(uint32_t Instance, uint8_t *NumSamples);
int32_t IKS01A2_ENV_SENSOR_FIFO_Get_Ovr_Status(uint32_t Instance, uint8_t *Status);
int32_t IKS01A2_ENV_SENSOR_FIFO_Reset_Interrupt(uint32_t Instance, uint8_t interrupt);
int32_t IKS01A2_ENV_SENSOR_FIFO_Set_Interrupt(uint32_t Instance, uint8_t Interrupt);
int32_t IKS01A2_ENV_SENSOR_FIFO_Set_Mode(uint32_t Instance, uint8_t Mode);
int32_t IKS01A2_ENV_SENSOR_FIFO_Set_Watermark_Level(uint32_t Instance, uint8_t Watermark);
int32_t IKS01A2_ENV_SENSOR_FIFO_Usage(uint32_t Instance, uint8_t Status);
int32_t IKS01A2_ENV_SENSOR_Get_DRDY_Status(uint32_t Instance, uint32_t Function, uint8_t *Status);
int32_t IKS01A2_ENV_SENSOR_Read_Register(uint32_t Instance, uint8_t Reg, uint8_t *Data);
int32_t IKS01A2_ENV_SENSOR_Write_Register(uint32_t Instance, uint8_t Reg, uint8_t Data);
int32_t IKS01A2_ENV_SENSOR_Set_One_Shot(uint32_t Instance);
int32_t IKS01A2_ENV_SENSOR_Get_One_Shot_Status(uint32_t Instance, uint8_t *Status);



 



 



 



 







 
#line 70 "..\\Inc\\main.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"











































 

 







 
#line 1 "..\\Inc\\iks01a2_conf.h"



































 

#line 40 "..\\Inc\\iks01a2_conf.h"
#line 41 "..\\Inc\\iks01a2_conf.h"
#line 42 "..\\Inc\\iks01a2_conf.h"

#line 61 "..\\Inc\\iks01a2_conf.h"

 
#line 88 "..\\Inc\\iks01a2_conf.h"

 

#line 56 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\Common\\motion_sensor.h"


















 

 









 
#line 33 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\Common\\motion_sensor.h"



 



 



 



 



 



 
typedef struct
{
  int32_t ( *Init              ) ( void * );
  int32_t ( *DeInit            ) ( void * );
  int32_t ( *ReadID            ) ( void *, uint8_t * ); 
  int32_t ( *GetCapabilities   ) ( void *, void * );
} MOTION_SENSOR_CommonDrv_t;

typedef struct
{
  int32_t ( *Enable            ) ( void * );
  int32_t ( *Disable           ) ( void * );
  int32_t ( *GetSensitivity    ) ( void *, float * );
  int32_t ( *GetOutputDataRate ) ( void *, float * );
  int32_t ( *SetOutputDataRate ) ( void *, float );
  int32_t ( *GetFullScale      ) ( void *, int32_t * );
  int32_t ( *SetFullScale      ) ( void *, int32_t );
  int32_t ( *GetAxes           ) ( void *, void * );
  int32_t ( *GetAxesRaw        ) ( void *, void * );
} MOTION_SENSOR_FuncDrv_t;



 



 



 



 



 







 
#line 57 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

























































#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl.h"

































 

 








 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"



































 

 







 
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
#line 49 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"




 




 

#line 115 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"




 







 

typedef int32_t (*lsm6dsl_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lsm6dsl_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  lsm6dsl_write_ptr  write_reg;
  lsm6dsl_read_ptr   read_reg;
   
  void *handle;
} lsm6dsl_ctx_t;




 




 

 



 





 


typedef struct {
  uint8_t not_used_01              : 5;
  uint8_t func_cfg_en              : 3;   
} lsm6dsl_func_cfg_access_t;


typedef struct {
  uint8_t tph                      : 4;
  uint8_t not_used_01              : 4;
} lsm6dsl_sensor_sync_time_frame_t;


typedef struct {
  uint8_t rr                       : 2;
  uint8_t not_used_01              : 6;
} lsm6dsl_sensor_sync_res_ratio_t;


typedef struct {
  uint8_t fth                      : 8;   
} lsm6dsl_fifo_ctrl1_t;


typedef struct {
  uint8_t fth                      : 3;   
  uint8_t fifo_temp_en             : 1;
  uint8_t not_used_01              : 2;
  uint8_t  timer_pedo_fifo_drdy    : 1;
  uint8_t timer_pedo_fifo_en       : 1;
} lsm6dsl_fifo_ctrl2_t;


typedef struct {
  uint8_t dec_fifo_xl              : 3;
  uint8_t dec_fifo_gyro            : 3;
  uint8_t not_used_01              : 2;
} lsm6dsl_fifo_ctrl3_t;


typedef struct {
  uint8_t dec_ds3_fifo             : 3;
  uint8_t dec_ds4_fifo             : 3;
  uint8_t only_high_data           : 1;
  uint8_t stop_on_fth              : 1;
} lsm6dsl_fifo_ctrl4_t;


typedef struct {
  uint8_t fifo_mode                : 3;
  uint8_t odr_fifo                 : 4;
  uint8_t not_used_01              : 1;
} lsm6dsl_fifo_ctrl5_t;


typedef struct {
  uint8_t int2_wrist_tilt          : 1;
  uint8_t not_used_01              : 6;
  uint8_t drdy_pulsed              : 1;
} lsm6dsl_drdy_pulse_cfg_g_t;


typedef struct {
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_full_flag           : 1;
  uint8_t int1_sign_mot            : 1;
  uint8_t int1_step_detector       : 1;
} lsm6dsl_int1_ctrl_t;


typedef struct {
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_full_flag           : 1;
  uint8_t int2_step_count_ov       : 1;
  uint8_t int2_step_delta          : 1;
} lsm6dsl_int2_ctrl_t;



typedef struct {
  uint8_t bw0_xl                   : 1;
  uint8_t lpf1_bw_sel              : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
} lsm6dsl_ctrl1_xl_t;


typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3;   
  uint8_t odr_g                    : 4;
} lsm6dsl_ctrl2_g_t;


typedef struct {
  uint8_t sw_reset                 : 1;
  uint8_t ble                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
} lsm6dsl_ctrl3_c_t;


typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t den_drdy_int1            : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t sleep                    : 1;
  uint8_t den_xl_en                : 1;
} lsm6dsl_ctrl4_c_t;


typedef struct {
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t den_lh                   : 1;
  uint8_t rounding                 : 3;
} lsm6dsl_ctrl5_c_t;


typedef struct {
  uint8_t ftype                    : 2;
  uint8_t not_used_01              : 1;
  uint8_t usr_off_w                : 1;
  uint8_t xl_hm_mode               : 1;
  uint8_t den_mode                 : 3;   
} lsm6dsl_ctrl6_c_t;


typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t rounding_status          : 1;
  uint8_t not_used_02              : 1;
  uint8_t hpm_g                    : 2;
  uint8_t hp_en_g                  : 1;
  uint8_t g_hm_mode                : 1;
} lsm6dsl_ctrl7_g_t;


typedef struct {
  uint8_t low_pass_on_6d           : 1;
  uint8_t not_used_01              : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t input_composite          : 1;
  uint8_t hp_ref_mode              : 1;
  uint8_t hpcf_xl                  : 2;
  uint8_t lpf2_xl_en               : 1;
} lsm6dsl_ctrl8_xl_t;


typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t soft_en                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t den_xl_g                 : 1;
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
} lsm6dsl_ctrl9_xl_t;


typedef struct {
  uint8_t sign_motion_en           : 1;
  uint8_t pedo_rst_step            : 1;
  uint8_t func_en                  : 1;
  uint8_t tilt_en                  : 1;
  uint8_t pedo_en                  : 1;
  uint8_t timer_en                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t wrist_tilt_en            : 1;
} lsm6dsl_ctrl10_c_t;


typedef struct {
  uint8_t master_on                : 1;
  uint8_t iron_en                  : 1;
  uint8_t pass_through_mode        : 1;
  uint8_t pull_up_en               : 1;
  uint8_t start_config             : 1;
  uint8_t not_used_01              : 1;
  uint8_t  data_valid_sel_fifo     : 1;
  uint8_t drdy_on_int1             : 1;
} lsm6dsl_master_config_t;


typedef struct {
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state_ia           : 1;
  uint8_t ff_ia                    : 1;
  uint8_t not_used_01              : 2;
} lsm6dsl_wake_up_src_t;


typedef struct {
  uint8_t z_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_01              : 1;
} lsm6dsl_tap_src_t;


typedef struct {
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
} lsm6dsl_d6d_src_t;


typedef struct {
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
} lsm6dsl_status_reg_t;

#line 418 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub1_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub2_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub3_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub4_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub5_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub6_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub7_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub8_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub9_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub10_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub11_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub12_reg_t;


typedef struct {
  uint8_t diff_fifo                : 8;   
} lsm6dsl_fifo_status1_t;


typedef struct {
  uint8_t diff_fifo                : 3;   
  uint8_t not_used_01              : 1;
  uint8_t fifo_empty               : 1;
  uint8_t fifo_full_smart          : 1;
  uint8_t over_run                 : 1;
  uint8_t waterm                   : 1;
} lsm6dsl_fifo_status2_t;


typedef struct {
  uint8_t fifo_pattern             : 8;   
} lsm6dsl_fifo_status3_t;


typedef struct {
  uint8_t fifo_pattern             : 2;   
  uint8_t not_used_01              : 6;
} lsm6dsl_fifo_status4_t;

#line 596 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub13_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub14_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub15_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub16_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub17_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub18_reg_t;


typedef struct {
  uint8_t sensorhub_end_op         : 1;
  uint8_t si_end_op                : 1;
  uint8_t hi_fail                  : 1;
  uint8_t step_overflow            : 1;
  uint8_t step_detected            : 1;
  uint8_t tilt_ia                  : 1;
  uint8_t sign_motion_ia           : 1;
  uint8_t  step_count_delta_ia     : 1;
} lsm6dsl_func_src1_t;


typedef struct {
  uint8_t wrist_tilt_ia            : 1;
  uint8_t not_used_01              : 2;
  uint8_t slave0_nack              : 1;
  uint8_t slave1_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave3_nack              : 1;
  uint8_t not_used_02              : 1;
} lsm6dsl_func_src2_t;


typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t wrist_tilt_ia_zneg       : 1;
  uint8_t wrist_tilt_ia_zpos       : 1;
  uint8_t wrist_tilt_ia_yneg       : 1;
  uint8_t wrist_tilt_ia_ypos       : 1;
  uint8_t wrist_tilt_ia_xneg       : 1;
  uint8_t wrist_tilt_ia_xpos       : 1;
} lsm6dsl_wrist_tilt_ia_t;


typedef struct {
  uint8_t lir                      : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t slope_fds                : 1;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
} lsm6dsl_tap_cfg_t;


typedef struct {
  uint8_t tap_ths                  : 5;
  uint8_t sixd_ths                 : 2;
  uint8_t d4d_en                   : 1;
} lsm6dsl_tap_ths_6d_t;


typedef struct {
  uint8_t shock                    : 2;
  uint8_t quiet                    : 2;
  uint8_t dur                      : 4;
} lsm6dsl_int_dur2_t;


typedef struct {
  uint8_t wk_ths                   : 6;
  uint8_t not_used_01              : 1;
  uint8_t single_double_tap        : 1;
} lsm6dsl_wake_up_ths_t;


typedef struct {
  uint8_t sleep_dur                : 4;
  uint8_t timer_hr                 : 1;
  uint8_t wake_dur                 : 2;
  uint8_t ff_dur                   : 1;
} lsm6dsl_wake_up_dur_t;


typedef struct {
  uint8_t ff_ths                   : 3;
  uint8_t ff_dur                   : 5;
} lsm6dsl_free_fall_t;


typedef struct {
  uint8_t int1_timer               : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_inact_state         : 1;
} lsm6dsl_md1_cfg_t;


typedef struct {
  uint8_t int2_iron                : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_inact_state         : 1;
} lsm6dsl_md2_cfg_t;


typedef struct {
  uint8_t master_cmd_code          : 8;
} lsm6dsl_master_cmd_code_t;


typedef struct {
  uint8_t error_code               : 8;
} lsm6dsl_sens_sync_spi_error_code_t;

#line 793 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
typedef struct {
  uint8_t rw_0                     : 1;
  uint8_t slave0_add               : 7;
} lsm6dsl_slv0_add_t;


typedef struct {
  uint8_t slave0_reg               : 8;
} lsm6dsl_slv0_subadd_t;


typedef struct {
  uint8_t slave0_numop             : 3;
  uint8_t src_mode                 : 1;
  uint8_t aux_sens_on              : 2;
  uint8_t slave0_rate              : 2;
} lsm6dsl_slave0_config_t;


typedef struct {
  uint8_t r_1                      : 1;
  uint8_t slave1_add               : 7;
} lsm6dsl_slv1_add_t;


typedef struct {
  uint8_t slave1_reg               : 8;
} lsm6dsl_slv1_subadd_t;


typedef struct {
  uint8_t slave1_numop             : 3;
  uint8_t not_used_01              : 2;
  uint8_t write_once               : 1;
  uint8_t slave1_rate              : 2;
} lsm6dsl_slave1_config_t;


typedef struct {
  uint8_t r_2                      : 1;
  uint8_t slave2_add               : 7;
} lsm6dsl_slv2_add_t;


typedef struct {
  uint8_t slave2_reg               : 8;
} lsm6dsl_slv2_subadd_t;


typedef struct {
  uint8_t slave2_numop             : 3;
  uint8_t not_used_01              : 3;
  uint8_t slave2_rate              : 2;
} lsm6dsl_slave2_config_t;


typedef struct {
  uint8_t r_3                      : 1;
  uint8_t slave3_add               : 7;
} lsm6dsl_slv3_add_t;


typedef struct {
  uint8_t slave3_reg               : 8;
} lsm6dsl_slv3_subadd_t;


typedef struct {
  uint8_t slave3_numop             : 3;
  uint8_t not_used_01              : 3;
  uint8_t slave3_rate              : 2;
} lsm6dsl_slave3_config_t;


typedef struct {
  uint8_t slave_dataw              : 8;
} lsm6dsl_datawrite_src_mode_sub_slv0_t;


typedef struct {
  uint8_t ths_min                  : 5;
  uint8_t not_used_01              : 2;
  uint8_t pedo_fs                  : 1;
} lsm6dsl_config_pedo_ths_min_t;



typedef struct {
  uint8_t deb_step      : 3;
  uint8_t deb_time      : 5;
} lsm6dsl_pedo_deb_reg_t;

#line 904 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t  wrist_tilt_mask_zneg    : 1;
  uint8_t  wrist_tilt_mask_zpos    : 1;
  uint8_t  wrist_tilt_mask_yneg    : 1;
  uint8_t  wrist_tilt_mask_ypos    : 1;
  uint8_t  wrist_tilt_mask_xneg    : 1;
  uint8_t  wrist_tilt_mask_xpos    : 1;
} lsm6dsl_a_wrist_tilt_mask_t;












 
typedef union{
  lsm6dsl_func_cfg_access_t                  func_cfg_access;
  lsm6dsl_sensor_sync_time_frame_t           sensor_sync_time_frame;
  lsm6dsl_sensor_sync_res_ratio_t            sensor_sync_res_ratio;
  lsm6dsl_fifo_ctrl1_t                       fifo_ctrl1;
  lsm6dsl_fifo_ctrl2_t                       fifo_ctrl2;
  lsm6dsl_fifo_ctrl3_t                       fifo_ctrl3;
  lsm6dsl_fifo_ctrl4_t                       fifo_ctrl4;
  lsm6dsl_fifo_ctrl5_t                       fifo_ctrl5;
  lsm6dsl_drdy_pulse_cfg_g_t                 drdy_pulse_cfg_g;
  lsm6dsl_int1_ctrl_t                        int1_ctrl;
  lsm6dsl_int2_ctrl_t                        int2_ctrl;
  lsm6dsl_ctrl1_xl_t                         ctrl1_xl;
  lsm6dsl_ctrl2_g_t                          ctrl2_g;
  lsm6dsl_ctrl3_c_t                          ctrl3_c;
  lsm6dsl_ctrl4_c_t                          ctrl4_c;
  lsm6dsl_ctrl5_c_t                          ctrl5_c;
  lsm6dsl_ctrl6_c_t                          ctrl6_c;
  lsm6dsl_ctrl7_g_t                          ctrl7_g;
  lsm6dsl_ctrl8_xl_t                         ctrl8_xl;
  lsm6dsl_ctrl9_xl_t                         ctrl9_xl;
  lsm6dsl_ctrl10_c_t                         ctrl10_c;
  lsm6dsl_master_config_t                    master_config;
  lsm6dsl_wake_up_src_t                      wake_up_src;
  lsm6dsl_tap_src_t                          tap_src;
  lsm6dsl_d6d_src_t                          d6d_src;
  lsm6dsl_status_reg_t                       status_reg;
  lsm6dsl_sensorhub1_reg_t                   sensorhub1_reg;
  lsm6dsl_sensorhub2_reg_t                   sensorhub2_reg;
  lsm6dsl_sensorhub3_reg_t                   sensorhub3_reg;
  lsm6dsl_sensorhub4_reg_t                   sensorhub4_reg;
  lsm6dsl_sensorhub5_reg_t                   sensorhub5_reg;
  lsm6dsl_sensorhub6_reg_t                   sensorhub6_reg;
  lsm6dsl_sensorhub7_reg_t                   sensorhub7_reg;
  lsm6dsl_sensorhub8_reg_t                   sensorhub8_reg;
  lsm6dsl_sensorhub9_reg_t                   sensorhub9_reg;
  lsm6dsl_sensorhub10_reg_t                  sensorhub10_reg;
  lsm6dsl_sensorhub11_reg_t                  sensorhub11_reg;
  lsm6dsl_sensorhub12_reg_t                  sensorhub12_reg;
  lsm6dsl_fifo_status1_t                     fifo_status1;
  lsm6dsl_fifo_status2_t                     fifo_status2;
  lsm6dsl_fifo_status3_t                     fifo_status3;
  lsm6dsl_fifo_status4_t                     fifo_status4;
  lsm6dsl_sensorhub13_reg_t                  sensorhub13_reg;
  lsm6dsl_sensorhub14_reg_t                  sensorhub14_reg;
  lsm6dsl_sensorhub15_reg_t                  sensorhub15_reg;
  lsm6dsl_sensorhub16_reg_t                  sensorhub16_reg;
  lsm6dsl_sensorhub17_reg_t                  sensorhub17_reg;
  lsm6dsl_sensorhub18_reg_t                  sensorhub18_reg;
  lsm6dsl_func_src1_t                        func_src1;
  lsm6dsl_func_src2_t                        func_src2;
  lsm6dsl_wrist_tilt_ia_t                    wrist_tilt_ia;
  lsm6dsl_tap_cfg_t                          tap_cfg;
  lsm6dsl_tap_ths_6d_t                       tap_ths_6d;
  lsm6dsl_int_dur2_t                         int_dur2;
  lsm6dsl_wake_up_ths_t                      wake_up_ths;
  lsm6dsl_wake_up_dur_t                      wake_up_dur;
  lsm6dsl_free_fall_t                        free_fall;
  lsm6dsl_md1_cfg_t                          md1_cfg;
  lsm6dsl_md2_cfg_t                          md2_cfg;
  lsm6dsl_master_cmd_code_t                  master_cmd_code;
  lsm6dsl_sens_sync_spi_error_code_t         sens_sync_spi_error_code;
  lsm6dsl_slv0_add_t                         slv0_add;
  lsm6dsl_slv0_subadd_t                      slv0_subadd;
  lsm6dsl_slave0_config_t                    slave0_config;
  lsm6dsl_slv1_add_t                         slv1_add;
  lsm6dsl_slv1_subadd_t                      slv1_subadd;
  lsm6dsl_slave1_config_t                    slave1_config;
  lsm6dsl_slv2_add_t                         slv2_add;
  lsm6dsl_slv2_subadd_t                      slv2_subadd;
  lsm6dsl_slave2_config_t                    slave2_config;
  lsm6dsl_slv3_add_t                         slv3_add;
  lsm6dsl_slv3_subadd_t                      slv3_subadd;
  lsm6dsl_slave3_config_t                    slave3_config;
  lsm6dsl_datawrite_src_mode_sub_slv0_t      datawrite_src_mode_sub_slv0;
  lsm6dsl_config_pedo_ths_min_t              config_pedo_ths_min;
  lsm6dsl_pedo_deb_reg_t                     pedo_deb_reg;
  lsm6dsl_a_wrist_tilt_mask_t                a_wrist_tilt_mask;
  bitwise_t                                  bitwise;
  uint8_t                                    byte;
} lsm6dsl_reg_t;




 

int32_t lsm6dsl_read_reg(lsm6dsl_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lsm6dsl_write_reg(lsm6dsl_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t lsm6dsl_from_fs2g_to_mg(int16_t lsb);
extern float_t lsm6dsl_from_fs4g_to_mg(int16_t lsb);
extern float_t lsm6dsl_from_fs8g_to_mg(int16_t lsb);
extern float_t lsm6dsl_from_fs16g_to_mg(int16_t lsb);

extern float_t lsm6dsl_from_fs125dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs250dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs500dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs1000dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs2000dps_to_mdps(int16_t lsb);

extern float_t lsm6dsl_from_lsb_to_celsius(int16_t lsb);

typedef enum {
  LSM6DSL_2g       = 0,
  LSM6DSL_16g      = 1,
  LSM6DSL_4g       = 2,
  LSM6DSL_8g       = 3,
  LSM6DSL_XL_FS_ND = 4,   
} lsm6dsl_fs_xl_t;
int32_t lsm6dsl_xl_full_scale_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_xl_t val);
int32_t lsm6dsl_xl_full_scale_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_xl_t *val);

typedef enum {
  LSM6DSL_XL_ODR_OFF      =  0,
  LSM6DSL_XL_ODR_12Hz5    =  1,
  LSM6DSL_XL_ODR_26Hz     =  2,
  LSM6DSL_XL_ODR_52Hz     =  3,
  LSM6DSL_XL_ODR_104Hz    =  4,
  LSM6DSL_XL_ODR_208Hz    =  5,
  LSM6DSL_XL_ODR_416Hz    =  6,
  LSM6DSL_XL_ODR_833Hz    =  7,
  LSM6DSL_XL_ODR_1k66Hz   =  8,
  LSM6DSL_XL_ODR_3k33Hz   =  9,
  LSM6DSL_XL_ODR_6k66Hz   = 10,
  LSM6DSL_XL_ODR_1Hz6     = 11,
  LSM6DSL_XL_ODR_ND       = 12,   
} lsm6dsl_odr_xl_t;
int32_t lsm6dsl_xl_data_rate_set(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_xl_t val);
int32_t lsm6dsl_xl_data_rate_get(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_xl_t *val);

typedef enum {
  LSM6DSL_250dps     = 0,
  LSM6DSL_125dps     = 1,
  LSM6DSL_500dps     = 2,
  LSM6DSL_1000dps    = 4,
  LSM6DSL_2000dps    = 6,
  LSM6DSL_GY_FS_ND   = 7,     
} lsm6dsl_fs_g_t;
int32_t lsm6dsl_gy_full_scale_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_g_t val);
int32_t lsm6dsl_gy_full_scale_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_g_t *val);

typedef enum {
  LSM6DSL_GY_ODR_OFF    =  0,
  LSM6DSL_GY_ODR_12Hz5  =  1,
  LSM6DSL_GY_ODR_26Hz   =  2,
  LSM6DSL_GY_ODR_52Hz   =  3,
  LSM6DSL_GY_ODR_104Hz  =  4,
  LSM6DSL_GY_ODR_208Hz  =  5,
  LSM6DSL_GY_ODR_416Hz  =  6,
  LSM6DSL_GY_ODR_833Hz  =  7,
  LSM6DSL_GY_ODR_1k66Hz =  8,
  LSM6DSL_GY_ODR_3k33Hz =  9,
  LSM6DSL_GY_ODR_6k66Hz = 10,
  LSM6DSL_GY_ODR_ND     = 11,     
} lsm6dsl_odr_g_t;
int32_t lsm6dsl_gy_data_rate_set(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_g_t val);
int32_t lsm6dsl_gy_data_rate_get(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_g_t *val);

int32_t lsm6dsl_block_data_update_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_block_data_update_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_LSb_1mg   = 0,
  LSM6DSL_LSb_16mg  = 1,
  LSM6DSL_WEIGHT_ND = 2,
} lsm6dsl_usr_off_w_t;
int32_t lsm6dsl_xl_offset_weight_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_usr_off_w_t val);
int32_t lsm6dsl_xl_offset_weight_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_usr_off_w_t *val);

typedef enum {
  LSM6DSL_XL_HIGH_PERFORMANCE  = 0,
  LSM6DSL_XL_NORMAL            = 1,
  LSM6DSL_XL_PW_MODE_ND        = 2,     
} lsm6dsl_xl_hm_mode_t;
int32_t lsm6dsl_xl_power_mode_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_xl_hm_mode_t val);
int32_t lsm6dsl_xl_power_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_xl_hm_mode_t *val);

typedef enum {
  LSM6DSL_STAT_RND_DISABLE  = 0,
  LSM6DSL_STAT_RND_ENABLE   = 1,
  LSM6DSL_STAT_RND_ND       = 2,     
} lsm6dsl_rounding_status_t;
int32_t lsm6dsl_rounding_on_status_set(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_rounding_status_t val);
int32_t lsm6dsl_rounding_on_status_get(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_rounding_status_t *val);

typedef enum {
  LSM6DSL_GY_HIGH_PERFORMANCE  = 0,
  LSM6DSL_GY_NORMAL            = 1,
  LSM6DSL_GY_PW_MODE_ND        = 2,     
} lsm6dsl_g_hm_mode_t;
int32_t lsm6dsl_gy_power_mode_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_g_hm_mode_t val);
int32_t lsm6dsl_gy_power_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_g_hm_mode_t *val);

typedef struct {
  lsm6dsl_wake_up_src_t        wake_up_src;
  lsm6dsl_tap_src_t            tap_src;
  lsm6dsl_d6d_src_t            d6d_src;
  lsm6dsl_status_reg_t         status_reg;
  lsm6dsl_func_src1_t          func_src1;
  lsm6dsl_func_src2_t          func_src2;
  lsm6dsl_wrist_tilt_ia_t      wrist_tilt_ia;
  lsm6dsl_a_wrist_tilt_mask_t  a_wrist_tilt_mask;
} lsm6dsl_all_sources_t;
int32_t lsm6dsl_all_sources_get(lsm6dsl_ctx_t *ctx,
                                lsm6dsl_all_sources_t *val);

int32_t lsm6dsl_status_reg_get(lsm6dsl_ctx_t *ctx, lsm6dsl_status_reg_t *val);

int32_t lsm6dsl_xl_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_gy_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_temp_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_xl_usr_offset_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_xl_usr_offset_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_timestamp_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_timestamp_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_LSB_6ms4    = 0,
  LSM6DSL_LSB_25us    = 1,
  LSM6DSL_TS_RES_ND   = 2,     
} lsm6dsl_timer_hr_t;
int32_t lsm6dsl_timestamp_res_set(lsm6dsl_ctx_t *ctx, lsm6dsl_timer_hr_t val);
int32_t lsm6dsl_timestamp_res_get(lsm6dsl_ctx_t *ctx, lsm6dsl_timer_hr_t *val);

typedef enum {
  LSM6DSL_ROUND_DISABLE            = 0,
  LSM6DSL_ROUND_XL                 = 1,
  LSM6DSL_ROUND_GY                 = 2,
  LSM6DSL_ROUND_GY_XL              = 3,
  LSM6DSL_ROUND_SH1_TO_SH6         = 4,
  LSM6DSL_ROUND_XL_SH1_TO_SH6      = 5,
  LSM6DSL_ROUND_GY_XL_SH1_TO_SH12  = 6,
  LSM6DSL_ROUND_GY_XL_SH1_TO_SH6   = 7,
  LSM6DSL_ROUND_OUT_ND             = 8,     
} lsm6dsl_rounding_t;
int32_t lsm6dsl_rounding_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_rounding_t val);
int32_t lsm6dsl_rounding_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_rounding_t *val);

int32_t lsm6dsl_temperature_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_angular_rate_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_acceleration_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_mag_calibrated_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_fifo_raw_data_get(lsm6dsl_ctx_t *ctx, uint8_t *buffer,
                                  uint8_t len);

typedef enum {
  LSM6DSL_USER_BANK   = 0,
  LSM6DSL_BANK_A      = 4,
  LSM6DSL_BANK_B      = 5,
  LSM6DSL_BANK_ND     = 6,     
} lsm6dsl_func_cfg_en_t;
int32_t lsm6dsl_mem_bank_set(lsm6dsl_ctx_t *ctx, lsm6dsl_func_cfg_en_t val);
int32_t lsm6dsl_mem_bank_get(lsm6dsl_ctx_t *ctx, lsm6dsl_func_cfg_en_t *val);

typedef enum {
  LSM6DSL_DRDY_LATCHED    = 0,
  LSM6DSL_DRDY_PULSED     = 1,
  LSM6DSL_DRDY_ND         = 2,   
} lsm6dsl_drdy_pulsed_g_t;
int32_t lsm6dsl_data_ready_mode_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_drdy_pulsed_g_t val);
int32_t lsm6dsl_data_ready_mode_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_drdy_pulsed_g_t *val);

int32_t lsm6dsl_device_id_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_reset_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_reset_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_LSB_AT_LOW_ADD  = 0,
  LSM6DSL_MSB_AT_LOW_ADD  = 1,
  LSM6DSL_DATA_FMT_ND     = 2,     
} lsm6dsl_ble_t;
int32_t lsm6dsl_data_format_set(lsm6dsl_ctx_t *ctx, lsm6dsl_ble_t val);
int32_t lsm6dsl_data_format_get(lsm6dsl_ctx_t *ctx, lsm6dsl_ble_t *val);

int32_t lsm6dsl_auto_increment_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_auto_increment_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_boot_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_boot_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_XL_ST_DISABLE    = 0,
  LSM6DSL_XL_ST_POSITIVE   = 1,
  LSM6DSL_XL_ST_NEGATIVE   = 2,
  LSM6DSL_XL_ST_ND         = 3,     
} lsm6dsl_st_xl_t;
int32_t lsm6dsl_xl_self_test_set(lsm6dsl_ctx_t *ctx, lsm6dsl_st_xl_t val);
int32_t lsm6dsl_xl_self_test_get(lsm6dsl_ctx_t *ctx, lsm6dsl_st_xl_t *val);

typedef enum {
  LSM6DSL_GY_ST_DISABLE    = 0,
  LSM6DSL_GY_ST_POSITIVE   = 1,
  LSM6DSL_GY_ST_NEGATIVE   = 3,
  LSM6DSL_GY_ST_ND         = 4,     
} lsm6dsl_st_g_t;
int32_t lsm6dsl_gy_self_test_set(lsm6dsl_ctx_t *ctx, lsm6dsl_st_g_t val);
int32_t lsm6dsl_gy_self_test_get(lsm6dsl_ctx_t *ctx, lsm6dsl_st_g_t *val);

int32_t lsm6dsl_filter_settling_mask_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_filter_settling_mask_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_USE_SLOPE    = 0,
  LSM6DSL_USE_HPF      = 1,
  LSM6DSL_HP_PATH_ND   = 2,     
} lsm6dsl_slope_fds_t;
int32_t lsm6dsl_xl_hp_path_internal_set(lsm6dsl_ctx_t *ctx,
                                        lsm6dsl_slope_fds_t val);
int32_t lsm6dsl_xl_hp_path_internal_get(lsm6dsl_ctx_t *ctx,
                                        lsm6dsl_slope_fds_t *val);

typedef enum {
  LSM6DSL_XL_ANA_BW_1k5Hz = 0,
  LSM6DSL_XL_ANA_BW_400Hz = 1,
  LSM6DSL_XL_ANA_BW_ND    = 2,     
} lsm6dsl_bw0_xl_t;
int32_t lsm6dsl_xl_filter_analog_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_bw0_xl_t val);
int32_t lsm6dsl_xl_filter_analog_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_bw0_xl_t *val);

typedef enum {
  LSM6DSL_XL_LP1_ODR_DIV_2 = 0,
  LSM6DSL_XL_LP1_ODR_DIV_4 = 1,
  LSM6DSL_XL_LP1_NA        = 2,   
} lsm6dsl_lpf1_bw_sel_t;
int32_t lsm6dsl_xl_lp1_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_lpf1_bw_sel_t val);
int32_t lsm6dsl_xl_lp1_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_lpf1_bw_sel_t *val);

typedef enum {
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_50     = 0x00,
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_100    = 0x01,
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9      = 0x02,
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_400    = 0x03,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_50   = 0x10,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100  = 0x11,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_9    = 0x12,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_400  = 0x13,
  LSM6DSL_XL_LP_NA                     = 0x20,  
} lsm6dsl_input_composite_t;
int32_t lsm6dsl_xl_lp2_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_input_composite_t val);
int32_t lsm6dsl_xl_lp2_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_input_composite_t *val);

int32_t lsm6dsl_xl_reference_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_xl_reference_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_XL_HP_ODR_DIV_4      = 0x00,  
  LSM6DSL_XL_HP_ODR_DIV_100    = 0x01,
  LSM6DSL_XL_HP_ODR_DIV_9      = 0x02,
  LSM6DSL_XL_HP_ODR_DIV_400    = 0x03,
  LSM6DSL_XL_HP_NA             = 0x10,  
} lsm6dsl_hpcf_xl_t;
int32_t lsm6dsl_xl_hp_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_hpcf_xl_t val);
int32_t lsm6dsl_xl_hp_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_hpcf_xl_t *val);

typedef enum {
  LSM6DSL_LP2_ONLY                    = 0x00,

  LSM6DSL_HP_16mHz_LP2                = 0x80,
  LSM6DSL_HP_65mHz_LP2                = 0x90,
  LSM6DSL_HP_260mHz_LP2               = 0xA0,
  LSM6DSL_HP_1Hz04_LP2                = 0xB0,

  LSM6DSL_HP_DISABLE_LP1_LIGHT        = 0x0A,
  LSM6DSL_HP_DISABLE_LP1_NORMAL       = 0x09,
  LSM6DSL_HP_DISABLE_LP_STRONG        = 0x08,
  LSM6DSL_HP_DISABLE_LP1_AGGRESSIVE   = 0x0B,

  LSM6DSL_HP_16mHz_LP1_LIGHT          = 0x8A,
  LSM6DSL_HP_65mHz_LP1_NORMAL         = 0x99,
  LSM6DSL_HP_260mHz_LP1_STRONG        = 0xA8,
  LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE     = 0xBB,

  LSM6DSL_HP_GY_BAND_NA               = 0xFF,     
} lsm6dsl_lpf1_sel_g_t;
int32_t lsm6dsl_gy_band_pass_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_lpf1_sel_g_t val);
int32_t lsm6dsl_gy_band_pass_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_lpf1_sel_g_t *val);

typedef enum {
  LSM6DSL_SPI_4_WIRE  = 0,
  LSM6DSL_SPI_3_WIRE  = 1,
  LSM6DSL_SPI_MODE_ND = 2,     
} lsm6dsl_sim_t;
int32_t lsm6dsl_spi_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_sim_t val);
int32_t lsm6dsl_spi_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_sim_t *val);

typedef enum {
  LSM6DSL_I2C_ENABLE   = 0,
  LSM6DSL_I2C_DISABLE  = 1,
  LSM6DSL_I2C_MODE_ND  = 2,     
} lsm6dsl_i2c_disable_t;
int32_t lsm6dsl_i2c_interface_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_i2c_disable_t val);
int32_t lsm6dsl_i2c_interface_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_i2c_disable_t *val);

typedef struct {
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_full_flag           : 1;
  uint8_t int1_sign_mot            : 1;
  uint8_t int1_step_detector       : 1;
  uint8_t int1_timer               : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_inact_state         : 1;
  uint8_t den_drdy_int1            : 1;
  uint8_t drdy_on_int1             : 1;
} lsm6dsl_int1_route_t;
int32_t lsm6dsl_pin_int1_route_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int1_route_t val);
int32_t lsm6dsl_pin_int1_route_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int1_route_t *val);

typedef struct{
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_full_flag           : 1;
  uint8_t int2_step_count_ov       : 1;
  uint8_t int2_step_delta          : 1;
  uint8_t int2_iron                : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_inact_state         : 1;
  uint8_t int2_wrist_tilt          : 1;
} lsm6dsl_int2_route_t;
int32_t lsm6dsl_pin_int2_route_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int2_route_t val);
int32_t lsm6dsl_pin_int2_route_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int2_route_t *val);

typedef enum {
  LSM6DSL_PUSH_PULL   = 0,
  LSM6DSL_OPEN_DRAIN  = 1,
  LSM6DSL_PIN_MODE_ND = 2,     
} lsm6dsl_pp_od_t;
int32_t lsm6dsl_pin_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_pp_od_t val);
int32_t lsm6dsl_pin_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_pp_od_t *val);

typedef enum {
  LSM6DSL_ACTIVE_HIGH   = 0,
  LSM6DSL_ACTIVE_LOW    = 1,
  LSM6DSL_POLARITY_ND   = 2,     
} lsm6dsl_h_lactive_t;
int32_t lsm6dsl_pin_polarity_set(lsm6dsl_ctx_t *ctx, lsm6dsl_h_lactive_t val);
int32_t lsm6dsl_pin_polarity_get(lsm6dsl_ctx_t *ctx, lsm6dsl_h_lactive_t *val);

int32_t lsm6dsl_all_on_int1_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_all_on_int1_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_INT_PULSED   = 0,
  LSM6DSL_INT_LATCHED  = 1,
  LSM6DSL_INT_MODE     = 2,     
} lsm6dsl_lir_t;
int32_t lsm6dsl_int_notification_set(lsm6dsl_ctx_t *ctx, lsm6dsl_lir_t val);
int32_t lsm6dsl_int_notification_get(lsm6dsl_ctx_t *ctx, lsm6dsl_lir_t *val);

int32_t lsm6dsl_wkup_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_wkup_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_wkup_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_wkup_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_gy_sleep_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_gy_sleep_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_PROPERTY_DISABLE          = 0,
  LSM6DSL_XL_12Hz5_GY_NOT_AFFECTED  = 1,
  LSM6DSL_XL_12Hz5_GY_SLEEP         = 2,
  LSM6DSL_XL_12Hz5_GY_PD            = 3,
  LSM6DSL_ACT_MODE_ND               = 4,     
} lsm6dsl_inact_en_t;
int32_t lsm6dsl_act_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_inact_en_t val);
int32_t lsm6dsl_act_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_inact_en_t *val);

int32_t lsm6dsl_act_sleep_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_act_sleep_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_src_get(lsm6dsl_ctx_t *ctx, lsm6dsl_tap_src_t *val);

int32_t lsm6dsl_tap_detection_on_z_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_detection_on_z_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_detection_on_y_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_detection_on_y_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_detection_on_x_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_detection_on_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_threshold_x_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_threshold_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_shock_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_shock_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_quiet_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_quiet_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_ONLY_SINGLE          = 0,
  LSM6DSL_BOTH_SINGLE_DOUBLE   = 1,
  LSM6DSL_TAP_MODE_ND          = 2,     
} lsm6dsl_single_double_tap_t;
int32_t lsm6dsl_tap_mode_set(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_single_double_tap_t val);
int32_t lsm6dsl_tap_mode_get(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_single_double_tap_t *val);

typedef enum {
  LSM6DSL_ODR_DIV_2_FEED      = 0,
  LSM6DSL_LPF2_FEED           = 1,
  LSM6DSL_6D_FEED_ND          = 2,     
} lsm6dsl_low_pass_on_6d_t;
int32_t lsm6dsl_6d_feed_data_set(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_low_pass_on_6d_t val);
int32_t lsm6dsl_6d_feed_data_get(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_low_pass_on_6d_t *val);

typedef enum {
  LSM6DSL_DEG_80      = 0,
  LSM6DSL_DEG_70      = 1,
  LSM6DSL_DEG_60      = 2,
  LSM6DSL_DEG_50      = 3,
  LSM6DSL_6D_TH_ND    = 4,     
} lsm6dsl_sixd_ths_t;
int32_t lsm6dsl_6d_threshold_set(lsm6dsl_ctx_t *ctx, lsm6dsl_sixd_ths_t val);
int32_t lsm6dsl_6d_threshold_get(lsm6dsl_ctx_t *ctx, lsm6dsl_sixd_ths_t *val);

int32_t lsm6dsl_4d_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_4d_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_ff_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_ff_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_FF_TSH_156mg = 0,
  LSM6DSL_FF_TSH_219mg = 1,
  LSM6DSL_FF_TSH_250mg = 2,
  LSM6DSL_FF_TSH_312mg = 3,
  LSM6DSL_FF_TSH_344mg = 4,
  LSM6DSL_FF_TSH_406mg = 5,
  LSM6DSL_FF_TSH_469mg = 6,
  LSM6DSL_FF_TSH_500mg = 7,
  LSM6DSL_FF_TSH_ND    = 8,     
} lsm6dsl_ff_ths_t;
int32_t lsm6dsl_ff_threshold_set(lsm6dsl_ctx_t *ctx, lsm6dsl_ff_ths_t val);
int32_t lsm6dsl_ff_threshold_get(lsm6dsl_ctx_t *ctx, lsm6dsl_ff_ths_t *val);

int32_t lsm6dsl_fifo_watermark_set(lsm6dsl_ctx_t *ctx, uint16_t val);
int32_t lsm6dsl_fifo_watermark_get(lsm6dsl_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsl_fifo_data_level_get(lsm6dsl_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsl_fifo_wtm_flag_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_fifo_pattern_get(lsm6dsl_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsl_fifo_temp_batch_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_fifo_temp_batch_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_TRG_XL_GY_DRDY     = 0,
  LSM6DSL_TRG_STEP_DETECT    = 1,
  LSM6DSL_TRG_SH_DRDY        = 2,
  LSM6DSL_TRG_SH_ND          = 3,     
} lsm6dsl_trigger_fifo_t;
int32_t lsm6dsl_fifo_write_trigger_set(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_trigger_fifo_t val);
int32_t lsm6dsl_fifo_write_trigger_get(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_trigger_fifo_t *val);

int32_t lsm6dsl_fifo_pedo_and_timestamp_batch_set(lsm6dsl_ctx_t *ctx,
                                                  uint8_t val);
int32_t lsm6dsl_fifo_pedo_and_timestamp_batch_get(lsm6dsl_ctx_t *ctx,
                                                  uint8_t *val);

typedef enum {
  LSM6DSL_FIFO_XL_DISABLE  = 0,
  LSM6DSL_FIFO_XL_NO_DEC   = 1,
  LSM6DSL_FIFO_XL_DEC_2    = 2,
  LSM6DSL_FIFO_XL_DEC_3    = 3,
  LSM6DSL_FIFO_XL_DEC_4    = 4,
  LSM6DSL_FIFO_XL_DEC_8    = 5,
  LSM6DSL_FIFO_XL_DEC_16   = 6,
  LSM6DSL_FIFO_XL_DEC_32   = 7,
  LSM6DSL_FIFO_XL_DEC_ND   = 8,     
} lsm6dsl_dec_fifo_xl_t;
int32_t lsm6dsl_fifo_xl_batch_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_xl_t val);
int32_t lsm6dsl_fifo_xl_batch_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_xl_t *val);

typedef enum {
  LSM6DSL_FIFO_GY_DISABLE = 0,
  LSM6DSL_FIFO_GY_NO_DEC  = 1,
  LSM6DSL_FIFO_GY_DEC_2   = 2,
  LSM6DSL_FIFO_GY_DEC_3   = 3,
  LSM6DSL_FIFO_GY_DEC_4   = 4,
  LSM6DSL_FIFO_GY_DEC_8   = 5,
  LSM6DSL_FIFO_GY_DEC_16  = 6,
  LSM6DSL_FIFO_GY_DEC_32  = 7,
  LSM6DSL_FIFO_GY_DEC_ND  = 8,     
} lsm6dsl_dec_fifo_gyro_t;
int32_t lsm6dsl_fifo_gy_batch_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_gyro_t val);
int32_t lsm6dsl_fifo_gy_batch_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_gyro_t *val);

typedef enum {
  LSM6DSL_FIFO_DS3_DISABLE   = 0,
  LSM6DSL_FIFO_DS3_NO_DEC    = 1,
  LSM6DSL_FIFO_DS3_DEC_2     = 2,
  LSM6DSL_FIFO_DS3_DEC_3     = 3,
  LSM6DSL_FIFO_DS3_DEC_4     = 4,
  LSM6DSL_FIFO_DS3_DEC_8     = 5,
  LSM6DSL_FIFO_DS3_DEC_16    = 6,
  LSM6DSL_FIFO_DS3_DEC_32    = 7,
  LSM6DSL_FIFO_DS3_DEC_ND    = 8,     
} lsm6dsl_dec_ds3_fifo_t;
int32_t lsm6dsl_fifo_dataset_3_batch_set(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds3_fifo_t val);
int32_t lsm6dsl_fifo_dataset_3_batch_get(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds3_fifo_t *val);

typedef enum {
  LSM6DSL_FIFO_DS4_DISABLE  = 0,
  LSM6DSL_FIFO_DS4_NO_DEC   = 1,
  LSM6DSL_FIFO_DS4_DEC_2    = 2,
  LSM6DSL_FIFO_DS4_DEC_3    = 3,
  LSM6DSL_FIFO_DS4_DEC_4    = 4,
  LSM6DSL_FIFO_DS4_DEC_8    = 5,
  LSM6DSL_FIFO_DS4_DEC_16   = 6,
  LSM6DSL_FIFO_DS4_DEC_32   = 7,
  LSM6DSL_FIFO_DS4_DEC_ND   = 8,     
} lsm6dsl_dec_ds4_fifo_t;
int32_t lsm6dsl_fifo_dataset_4_batch_set(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds4_fifo_t val);
int32_t lsm6dsl_fifo_dataset_4_batch_get(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds4_fifo_t *val);

int32_t lsm6dsl_fifo_xl_gy_8bit_format_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_fifo_xl_gy_8bit_format_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_fifo_stop_on_wtm_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_fifo_stop_on_wtm_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_BYPASS_MODE           = 0,
  LSM6DSL_FIFO_MODE             = 1,
  LSM6DSL_STREAM_TO_FIFO_MODE   = 3,
  LSM6DSL_BYPASS_TO_STREAM_MODE = 4,
  LSM6DSL_STREAM_MODE           = 6,
  LSM6DSL_FIFO_MODE_ND          = 8,     
} lsm6dsl_fifo_mode_t;
int32_t lsm6dsl_fifo_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fifo_mode_t val);
int32_t lsm6dsl_fifo_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fifo_mode_t *val);

typedef enum {
  LSM6DSL_FIFO_DISABLE   =  0,
  LSM6DSL_FIFO_12Hz5     =  1,
  LSM6DSL_FIFO_26Hz      =  2,
  LSM6DSL_FIFO_52Hz      =  3,
  LSM6DSL_FIFO_104Hz     =  4,
  LSM6DSL_FIFO_208Hz     =  5,
  LSM6DSL_FIFO_416Hz     =  6,
  LSM6DSL_FIFO_833Hz     =  7,
  LSM6DSL_FIFO_1k66Hz    =  8,
  LSM6DSL_FIFO_3k33Hz    =  9,
  LSM6DSL_FIFO_6k66Hz    = 10,
  LSM6DSL_FIFO_RATE_ND   = 11,     
} lsm6dsl_odr_fifo_t;
int32_t lsm6dsl_fifo_data_rate_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_odr_fifo_t val);
int32_t lsm6dsl_fifo_data_rate_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_odr_fifo_t *val);

typedef enum {
  LSM6DSL_DEN_ACT_LOW    = 0,
  LSM6DSL_DEN_ACT_HIGH   = 1,
  LSM6DSL_DEN_POL_ND     = 2,     
} lsm6dsl_den_lh_t;
int32_t lsm6dsl_den_polarity_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_lh_t val);
int32_t lsm6dsl_den_polarity_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_lh_t *val);

typedef enum {
  LSM6DSL_DEN_DISABLE    = 0,
  LSM6DSL_LEVEL_FIFO     = 6,
  LSM6DSL_LEVEL_LETCHED  = 3,
  LSM6DSL_LEVEL_TRIGGER  = 2,
  LSM6DSL_EDGE_TRIGGER   = 4,
  LSM6DSL_DEN_MODE_ND    = 5,     
} lsm6dsl_den_mode_t;
int32_t lsm6dsl_den_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_mode_t val);
int32_t lsm6dsl_den_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_mode_t *val);

typedef enum {
  LSM6DSL_STAMP_IN_GY_DATA     = 0,
  LSM6DSL_STAMP_IN_XL_DATA     = 1,
  LSM6DSL_STAMP_IN_GY_XL_DATA  = 2,
  LSM6DSL_DEN_STAMP_ND         = 3,     
} lsm6dsl_den_xl_en_t;
int32_t lsm6dsl_den_enable_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_xl_en_t val);
int32_t lsm6dsl_den_enable_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_xl_en_t *val);

int32_t lsm6dsl_den_mark_axis_z_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_den_mark_axis_z_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_den_mark_axis_y_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_den_mark_axis_y_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_den_mark_axis_x_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_den_mark_axis_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_step_reset_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_step_reset_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_PEDO_AT_2g = 0,
  LSM6DSL_PEDO_AT_4g = 1,
  LSM6DSL_PEDO_FS_ND = 2,     
} lsm6dsl_pedo_fs_t;
int32_t lsm6dsl_pedo_full_scale_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_pedo_fs_t val);
int32_t lsm6dsl_pedo_full_scale_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_pedo_fs_t *val);

int32_t lsm6dsl_pedo_debounce_steps_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_debounce_steps_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_timeout_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_timeout_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_steps_period_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_pedo_steps_period_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_motion_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_motion_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_motion_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_motion_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_tilt_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tilt_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_wrist_tilt_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_wrist_tilt_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tilt_latency_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_tilt_latency_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_tilt_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_tilt_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_tilt_src_set(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_a_wrist_tilt_mask_t *val);
int32_t lsm6dsl_tilt_src_get(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_a_wrist_tilt_mask_t *val);

int32_t lsm6dsl_mag_soft_iron_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_mag_soft_iron_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_mag_hard_iron_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_mag_hard_iron_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_mag_soft_iron_mat_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_mag_soft_iron_mat_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_mag_offset_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_mag_offset_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_func_en_set(lsm6dsl_ctx_t *ctx, uint8_t val);

int32_t lsm6dsl_sh_sync_sens_frame_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_sync_sens_frame_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_RES_RATIO_2_11  = 0,
  LSM6DSL_RES_RATIO_2_12  = 1,
  LSM6DSL_RES_RATIO_2_13  = 2,
  LSM6DSL_RES_RATIO_2_14  = 3,
  LSM6DSL_RES_RATIO_ND    = 4,     
} lsm6dsl_rr_t;
int32_t lsm6dsl_sh_sync_sens_ratio_set(lsm6dsl_ctx_t *ctx, lsm6dsl_rr_t val);
int32_t lsm6dsl_sh_sync_sens_ratio_get(lsm6dsl_ctx_t *ctx, lsm6dsl_rr_t *val);

int32_t lsm6dsl_sh_master_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_master_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_sh_pass_through_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_pass_through_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_EXT_PULL_UP       = 0,
  LSM6DSL_INTERNAL_PULL_UP  = 1,
  LSM6DSL_SH_PIN_MODE       = 2,     
} lsm6dsl_pull_up_en_t;
int32_t lsm6dsl_sh_pin_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_pull_up_en_t val);
int32_t lsm6dsl_sh_pin_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_pull_up_en_t *val);

typedef enum {
  LSM6DSL_XL_GY_DRDY        = 0,
  LSM6DSL_EXT_ON_INT2_PIN   = 1,
  LSM6DSL_SH_SYNCRO_ND      = 2,     
} lsm6dsl_start_config_t;
int32_t lsm6dsl_sh_syncro_mode_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_start_config_t val);
int32_t lsm6dsl_sh_syncro_mode_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_start_config_t *val);

int32_t lsm6dsl_sh_drdy_on_int1_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_drdy_on_int1_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef struct {
    lsm6dsl_sensorhub1_reg_t   sh_byte_1;
    lsm6dsl_sensorhub2_reg_t   sh_byte_2;
    lsm6dsl_sensorhub3_reg_t   sh_byte_3;
    lsm6dsl_sensorhub4_reg_t   sh_byte_4;
    lsm6dsl_sensorhub5_reg_t   sh_byte_5;
    lsm6dsl_sensorhub6_reg_t   sh_byte_6;
    lsm6dsl_sensorhub7_reg_t   sh_byte_7;
    lsm6dsl_sensorhub8_reg_t   sh_byte_8;
    lsm6dsl_sensorhub9_reg_t   sh_byte_9;
    lsm6dsl_sensorhub10_reg_t  sh_byte_10;
    lsm6dsl_sensorhub11_reg_t  sh_byte_11;
    lsm6dsl_sensorhub12_reg_t  sh_byte_12;
    lsm6dsl_sensorhub13_reg_t  sh_byte_13;
    lsm6dsl_sensorhub14_reg_t  sh_byte_14;
    lsm6dsl_sensorhub15_reg_t  sh_byte_15;
    lsm6dsl_sensorhub16_reg_t  sh_byte_16;
    lsm6dsl_sensorhub17_reg_t  sh_byte_17;
    lsm6dsl_sensorhub18_reg_t  sh_byte_18;
} lsm6dsl_emb_sh_read_t;
int32_t lsm6dsl_sh_read_data_raw_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_emb_sh_read_t *val);

int32_t lsm6dsl_sh_cmd_sens_sync_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_cmd_sens_sync_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_sh_spi_sync_error_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_spi_sync_error_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_SLV_0        = 0,
  LSM6DSL_SLV_0_1      = 1,
  LSM6DSL_SLV_0_1_2    = 2,
  LSM6DSL_SLV_0_1_2_3  = 3,
  LSM6DSL_SLV_EN_ND    = 4,     
} lsm6dsl_aux_sens_on_t;
int32_t lsm6dsl_sh_num_of_dev_connected_set(lsm6dsl_ctx_t *ctx,
                                            lsm6dsl_aux_sens_on_t val);
int32_t lsm6dsl_sh_num_of_dev_connected_get(lsm6dsl_ctx_t *ctx,
                                            lsm6dsl_aux_sens_on_t *val);

typedef struct{
  uint8_t   slv0_add;
  uint8_t   slv0_subadd;
  uint8_t   slv0_data;
} lsm6dsl_sh_cfg_write_t;
int32_t lsm6dsl_sh_cfg_write(lsm6dsl_ctx_t *ctx, lsm6dsl_sh_cfg_write_t *val);

typedef struct{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} lsm6dsl_sh_cfg_read_t;
int32_t lsm6dsl_sh_slv0_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);
int32_t lsm6dsl_sh_slv1_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);
int32_t lsm6dsl_sh_slv2_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);
int32_t lsm6dsl_sh_slv3_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);

typedef enum {
  LSM6DSL_SL0_NO_DEC   = 0,
  LSM6DSL_SL0_DEC_2    = 1,
  LSM6DSL_SL0_DEC_4    = 2,
  LSM6DSL_SL0_DEC_8    = 3,
  LSM6DSL_SL0_DEC_ND   = 4,     
} lsm6dsl_slave0_rate_t;
int32_t lsm6dsl_sh_slave_0_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave0_rate_t val);
int32_t lsm6dsl_sh_slave_0_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave0_rate_t *val);

typedef enum {
  LSM6DSL_EACH_SH_CYCLE     = 0,
  LSM6DSL_ONLY_FIRST_CYCLE  = 1,
  LSM6DSL_SH_WR_MODE_ND     = 2,     
} lsm6dsl_write_once_t;
int32_t lsm6dsl_sh_write_mode_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_write_once_t val);
int32_t lsm6dsl_sh_write_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_write_once_t *val);

typedef enum {
  LSM6DSL_SL1_NO_DEC   = 0,
  LSM6DSL_SL1_DEC_2    = 1,
  LSM6DSL_SL1_DEC_4    = 2,
  LSM6DSL_SL1_DEC_8    = 3,
  LSM6DSL_SL1_DEC_ND   = 4,     
} lsm6dsl_slave1_rate_t;
int32_t lsm6dsl_sh_slave_1_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave1_rate_t val);
int32_t lsm6dsl_sh_slave_1_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave1_rate_t *val);

typedef enum {
  LSM6DSL_SL2_NO_DEC  = 0,
  LSM6DSL_SL2_DEC_2   = 1,
  LSM6DSL_SL2_DEC_4   = 2,
  LSM6DSL_SL2_DEC_8   = 3,
  LSM6DSL_SL2_DEC_ND  = 4,     
} lsm6dsl_slave2_rate_t;
int32_t lsm6dsl_sh_slave_2_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave2_rate_t val);
int32_t lsm6dsl_sh_slave_2_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave2_rate_t *val);

typedef enum {
  LSM6DSL_SL3_NO_DEC  = 0,
  LSM6DSL_SL3_DEC_2   = 1,
  LSM6DSL_SL3_DEC_4   = 2,
  LSM6DSL_SL3_DEC_8   = 3,
  LSM6DSL_SL3_DEC_ND  = 4,     
} lsm6dsl_slave3_rate_t;
int32_t lsm6dsl_sh_slave_3_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave3_rate_t val);
int32_t lsm6dsl_sh_slave_3_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave3_rate_t *val);




 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl.h"
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl.h"



 



 



 



 

typedef int32_t (*LSM6DSL_Init_Func)(void);
typedef int32_t (*LSM6DSL_DeInit_Func)(void);
typedef int32_t (*LSM6DSL_GetTick_Func)(void);
typedef int32_t (*LSM6DSL_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LSM6DSL_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef enum
{
  LSM6DSL_INT1_PIN,
  LSM6DSL_INT2_PIN,
} LSM6DSL_SensorIntPin_t;

typedef struct
{
  LSM6DSL_Init_Func          Init;
  LSM6DSL_DeInit_Func        DeInit;
  uint32_t                   BusType;  
  uint8_t                    Address;
  LSM6DSL_WriteReg_Func      WriteReg;
  LSM6DSL_ReadReg_Func       ReadReg;
  LSM6DSL_GetTick_Func       GetTick;
} LSM6DSL_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LSM6DSL_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM6DSL_Axes_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LSM6DSL_Event_Status_t;

typedef struct
{
  LSM6DSL_IO_t        IO;
  lsm6dsl_ctx_t       Ctx;
  uint8_t             is_initialized;
  uint8_t             acc_is_enabled;
  uint8_t             gyro_is_enabled;
  lsm6dsl_odr_xl_t    acc_odr;
  lsm6dsl_odr_g_t     gyro_odr;
} LSM6DSL_Object_t;

typedef struct
{
  uint8_t   Acc;
  uint8_t   Gyro;
  uint8_t   Magneto;
  uint8_t   LowPower;
  uint32_t  GyroMaxFS;
  uint32_t  AccMaxFS;
  uint32_t  MagMaxFS;
  float     GyroMaxOdr;
  float     AccMaxOdr;
  float     MagMaxOdr;
} LSM6DSL_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LSM6DSL_Object_t *);
  int32_t (*DeInit)(LSM6DSL_Object_t *);
  int32_t (*ReadID)(LSM6DSL_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM6DSL_Object_t *, LSM6DSL_Capabilities_t *);
} LSM6DSL_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM6DSL_Object_t *);
  int32_t (*Disable)(LSM6DSL_Object_t *);
  int32_t (*GetSensitivity)(LSM6DSL_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM6DSL_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM6DSL_Object_t *, float);
  int32_t (*GetFullScale)(LSM6DSL_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM6DSL_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM6DSL_Object_t *, LSM6DSL_Axes_t *);
  int32_t (*GetAxesRaw)(LSM6DSL_Object_t *, LSM6DSL_AxesRaw_t *);
} LSM6DSL_ACC_Drv_t;

typedef struct
{
  int32_t (*Enable)(LSM6DSL_Object_t *);
  int32_t (*Disable)(LSM6DSL_Object_t *);
  int32_t (*GetSensitivity)(LSM6DSL_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM6DSL_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM6DSL_Object_t *, float);
  int32_t (*GetFullScale)(LSM6DSL_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM6DSL_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM6DSL_Object_t *, LSM6DSL_Axes_t *);
  int32_t (*GetAxesRaw)(LSM6DSL_Object_t *, LSM6DSL_AxesRaw_t *);
} LSM6DSL_GYRO_Drv_t;



 



 





















 



 

int32_t LSM6DSL_RegisterBusIO(LSM6DSL_Object_t *pObj, LSM6DSL_IO_t *pIO);
int32_t LSM6DSL_Init(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_DeInit(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ReadID(LSM6DSL_Object_t *pObj, uint8_t *Id);
int32_t LSM6DSL_GetCapabilities(LSM6DSL_Object_t *pObj, LSM6DSL_Capabilities_t *Capabilities);

int32_t LSM6DSL_ACC_Enable(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Disable(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_GetSensitivity(LSM6DSL_Object_t *pObj, float *Sensitivity);
int32_t LSM6DSL_ACC_GetOutputDataRate(LSM6DSL_Object_t *pObj, float *Odr);
int32_t LSM6DSL_ACC_SetOutputDataRate(LSM6DSL_Object_t *pObj, float Odr);
int32_t LSM6DSL_ACC_GetFullScale(LSM6DSL_Object_t *pObj, int32_t *FullScale);
int32_t LSM6DSL_ACC_SetFullScale(LSM6DSL_Object_t *pObj, int32_t FullScale);
int32_t LSM6DSL_ACC_GetAxesRaw(LSM6DSL_Object_t *pObj, LSM6DSL_AxesRaw_t *Value);
int32_t LSM6DSL_ACC_GetAxes(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *Acceleration);

int32_t LSM6DSL_GYRO_Enable(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_GYRO_Disable(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_GYRO_GetSensitivity(LSM6DSL_Object_t *pObj, float *Sensitivity);
int32_t LSM6DSL_GYRO_GetOutputDataRate(LSM6DSL_Object_t *pObj, float *Odr);
int32_t LSM6DSL_GYRO_SetOutputDataRate(LSM6DSL_Object_t *pObj, float Odr);
int32_t LSM6DSL_GYRO_GetFullScale(LSM6DSL_Object_t *pObj, int32_t *FullScale);
int32_t LSM6DSL_GYRO_SetFullScale(LSM6DSL_Object_t *pObj, int32_t FullScale);
int32_t LSM6DSL_GYRO_GetAxesRaw(LSM6DSL_Object_t *pObj, LSM6DSL_AxesRaw_t *Value);
int32_t LSM6DSL_GYRO_GetAxes(LSM6DSL_Object_t *pObj, LSM6DSL_Axes_t *AngularRate);

int32_t LSM6DSL_Read_Reg(LSM6DSL_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LSM6DSL_Write_Reg(LSM6DSL_Object_t *pObj, uint8_t reg, uint8_t Data);
int32_t LSM6DSL_Set_Interrupt_Latch(LSM6DSL_Object_t *pObj, uint8_t Status);

int32_t LSM6DSL_ACC_Enable_Free_Fall_Detection(LSM6DSL_Object_t *pObj, LSM6DSL_SensorIntPin_t IntPin);
int32_t LSM6DSL_ACC_Disable_Free_Fall_Detection(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Set_Free_Fall_Threshold(LSM6DSL_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSL_ACC_Set_Free_Fall_Duration(LSM6DSL_Object_t *pObj, uint8_t Duration);

int32_t LSM6DSL_ACC_Enable_Pedometer(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Disable_Pedometer(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Get_Step_Count(LSM6DSL_Object_t *pObj, uint16_t *StepCount);
int32_t LSM6DSL_ACC_Enable_Step_Counter_Reset(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Disable_Step_Counter_Reset(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Set_Pedometer_Threshold(LSM6DSL_Object_t *pObj, uint8_t Threshold);

int32_t LSM6DSL_ACC_Enable_Tilt_Detection(LSM6DSL_Object_t *pObj, LSM6DSL_SensorIntPin_t IntPin);
int32_t LSM6DSL_ACC_Disable_Tilt_Detection(LSM6DSL_Object_t *pObj);

int32_t LSM6DSL_ACC_Enable_Wake_Up_Detection(LSM6DSL_Object_t *pObj, LSM6DSL_SensorIntPin_t IntPin);
int32_t LSM6DSL_ACC_Disable_Wake_Up_Detection(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Set_Wake_Up_Threshold(LSM6DSL_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSL_ACC_Set_Wake_Up_Duration(LSM6DSL_Object_t *pObj, uint8_t Duration);

int32_t LSM6DSL_ACC_Enable_Inactivity_Detection(LSM6DSL_Object_t *pObj, LSM6DSL_SensorIntPin_t IntPin);
int32_t LSM6DSL_ACC_Disable_Inactivity_Detection(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Set_Sleep_Duration(LSM6DSL_Object_t *pObj, uint8_t Duration);

int32_t LSM6DSL_ACC_Enable_Single_Tap_Detection(LSM6DSL_Object_t *pObj, LSM6DSL_SensorIntPin_t IntPin);
int32_t LSM6DSL_ACC_Disable_Single_Tap_Detection(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Enable_Double_Tap_Detection(LSM6DSL_Object_t *pObj, LSM6DSL_SensorIntPin_t IntPin);
int32_t LSM6DSL_ACC_Disable_Double_Tap_Detection(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Set_Tap_Threshold(LSM6DSL_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSL_ACC_Set_Tap_Shock_Time(LSM6DSL_Object_t *pObj, uint8_t Time);
int32_t LSM6DSL_ACC_Set_Tap_Quiet_Time(LSM6DSL_Object_t *pObj, uint8_t Time);
int32_t LSM6DSL_ACC_Set_Tap_Duration_Time(LSM6DSL_Object_t *pObj, uint8_t Time);

int32_t LSM6DSL_ACC_Enable_6D_Orientation(LSM6DSL_Object_t *pObj, LSM6DSL_SensorIntPin_t IntPin);
int32_t LSM6DSL_ACC_Disable_6D_Orientation(LSM6DSL_Object_t *pObj);
int32_t LSM6DSL_ACC_Set_6D_Orientation_Threshold(LSM6DSL_Object_t *pObj, uint8_t Threshold);
int32_t LSM6DSL_ACC_Get_6D_Orientation_XL(LSM6DSL_Object_t *pObj, uint8_t *XLow);
int32_t LSM6DSL_ACC_Get_6D_Orientation_XH(LSM6DSL_Object_t *pObj, uint8_t *XHigh);
int32_t LSM6DSL_ACC_Get_6D_Orientation_YL(LSM6DSL_Object_t *pObj, uint8_t *YLow);
int32_t LSM6DSL_ACC_Get_6D_Orientation_YH(LSM6DSL_Object_t *pObj, uint8_t *YHigh);
int32_t LSM6DSL_ACC_Get_6D_Orientation_ZL(LSM6DSL_Object_t *pObj, uint8_t *ZLow);
int32_t LSM6DSL_ACC_Get_6D_Orientation_ZH(LSM6DSL_Object_t *pObj, uint8_t *ZHigh);

int32_t LSM6DSL_ACC_Get_Event_Status(LSM6DSL_Object_t *pObj, LSM6DSL_Event_Status_t *Status);
int32_t LSM6DSL_ACC_Set_SelfTest(LSM6DSL_Object_t *pObj, uint8_t Val);
int32_t LSM6DSL_ACC_Get_DRDY_Status(LSM6DSL_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSL_ACC_Get_Init_Status(LSM6DSL_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSL_ACC_Set_INT1_DRDY(LSM6DSL_Object_t *pObj, uint8_t Val);

int32_t LSM6DSL_GYRO_Set_SelfTest(LSM6DSL_Object_t *pObj, uint8_t Val);
int32_t LSM6DSL_GYRO_Get_DRDY_Status(LSM6DSL_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSL_GYRO_Get_Init_Status(LSM6DSL_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSL_GYRO_Set_INT1_DRDY(LSM6DSL_Object_t *pObj, uint8_t Val);

int32_t LSM6DSL_Set_DRDY_Mode(LSM6DSL_Object_t *pObj, uint8_t Val);

int32_t LSM6DSL_FIFO_Get_Num_Samples(LSM6DSL_Object_t *pObj, uint16_t *NumSamples);
int32_t LSM6DSL_FIFO_Get_Full_Status(LSM6DSL_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSL_FIFO_Set_ODR_Value(LSM6DSL_Object_t *pObj, float Odr);
int32_t LSM6DSL_FIFO_Set_INT1_FIFO_Full(LSM6DSL_Object_t *pObj, uint8_t Status);
int32_t LSM6DSL_FIFO_Set_Watermark_Level(LSM6DSL_Object_t *pObj, uint16_t Watermark);
int32_t LSM6DSL_FIFO_Set_Stop_On_Fth(LSM6DSL_Object_t *pObj, uint8_t Status);
int32_t LSM6DSL_FIFO_Set_Mode(LSM6DSL_Object_t *pObj, uint8_t Mode);
int32_t LSM6DSL_FIFO_Get_Pattern(LSM6DSL_Object_t *pObj, uint16_t *Pattern);
int32_t LSM6DSL_FIFO_Get_Data(LSM6DSL_Object_t *pObj, uint8_t *Data);
int32_t LSM6DSL_FIFO_Get_Empty_Status(LSM6DSL_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSL_FIFO_Get_Overrun_Status(LSM6DSL_Object_t *pObj, uint8_t *Status);
int32_t LSM6DSL_FIFO_ACC_Set_Decimation(LSM6DSL_Object_t *pObj, uint8_t Decimation);
int32_t LSM6DSL_FIFO_ACC_Get_Axis(LSM6DSL_Object_t *pObj, int32_t *Acceleration);
int32_t LSM6DSL_FIFO_GYRO_Set_Decimation(LSM6DSL_Object_t *pObj, uint8_t Decimation);
int32_t LSM6DSL_FIFO_GYRO_Get_Axis(LSM6DSL_Object_t *pObj, int32_t *AngularVelocity);



 



 

extern LSM6DSL_CommonDrv_t LSM6DSL_COMMON_Driver;
extern LSM6DSL_ACC_Drv_t LSM6DSL_ACC_Driver;
extern LSM6DSL_GYRO_Drv_t LSM6DSL_GYRO_Driver;



 









 



 



 

 
#line 116 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"


#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"

































 

 








 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"



































 
 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"




 




 

#line 114 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"




 

  





 

typedef int32_t (*lsm303agr_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lsm303agr_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  lsm303agr_write_ptr  write_reg;
  lsm303agr_read_ptr   read_reg;
   
  void *handle;
} lsm303agr_ctx_t;




 




 

   



 






 


typedef struct {
  uint8_t not_used_01            : 2;
  uint8_t tda                    : 1;
  uint8_t not_used_02            : 3;
  uint8_t tor                    : 1;
  uint8_t not_used_03            : 1;
} lsm303agr_status_reg_aux_a_t;







typedef struct {
  uint8_t not_used_01            : 6;
  uint8_t temp_en                : 2;
} lsm303agr_temp_cfg_reg_a_t;


typedef struct {
  uint8_t xen                    : 1;
  uint8_t yen                    : 1;
  uint8_t zen                    : 1;
  uint8_t lpen                   : 1;
  uint8_t odr                    : 4;
} lsm303agr_ctrl_reg1_a_t;


typedef struct {
  uint8_t hp                     : 3;  
  uint8_t fds                    : 1;
  uint8_t hpcf                   : 2;
  uint8_t hpm                    : 2;
} lsm303agr_ctrl_reg2_a_t;


typedef struct {
  uint8_t not_used_01            : 1;
  uint8_t i1_overrun             : 1;
  uint8_t i1_wtm                 : 1;
  uint8_t i1_drdy2               : 1;
  uint8_t i1_drdy1               : 1;
  uint8_t i1_aoi2                : 1;
  uint8_t i1_aoi1                : 1;
  uint8_t i1_click               : 1;
} lsm303agr_ctrl_reg3_a_t;


typedef struct {
  uint8_t spi_enable             : 1;
  uint8_t st                     : 2;
  uint8_t hr                     : 1;
  uint8_t fs                     : 2;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
} lsm303agr_ctrl_reg4_a_t;


typedef struct {
  uint8_t d4d_int2               : 1;
  uint8_t lir_int2               : 1;
  uint8_t d4d_int1               : 1;
  uint8_t lir_int1               : 1;
  uint8_t not_used_01            : 2;
  uint8_t fifo_en                : 1;
  uint8_t boot                   : 1;
} lsm303agr_ctrl_reg5_a_t;


typedef struct {
  uint8_t not_used_01            : 1;
  uint8_t h_lactive              : 1;
  uint8_t not_used_02            : 1;
  uint8_t p2_act                 : 1;
  uint8_t boot_i2                : 1;
  uint8_t i2_int2                : 1;
  uint8_t i2_int1                : 1;
  uint8_t i2_clicken             : 1;
} lsm303agr_ctrl_reg6_a_t;



typedef struct {
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t _xor                   : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
} lsm303agr_status_reg_a_t;

#line 264 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"
typedef struct {
  uint8_t fth                    : 5;
  uint8_t tr                     : 1;
  uint8_t fm                     : 2;
} lsm303agr_fifo_ctrl_reg_a_t;


typedef struct {
  uint8_t fss                    : 5;
  uint8_t empty                  : 1;
  uint8_t ovrn_fifo              : 1;
  uint8_t wtm                    : 1;
} lsm303agr_fifo_src_reg_a_t;


typedef struct {
  uint8_t xlie                   : 1;  
  uint8_t xhie                   : 1;  
  uint8_t ylie                   : 1;  
  uint8_t yhie                   : 1;  
  uint8_t zlie                   : 1;  
  uint8_t zhie                   : 1;  
  uint8_t _6d                    : 1;
  uint8_t aoi                    : 1;
} lsm303agr_int1_cfg_a_t;


typedef struct {
  uint8_t xl                     : 1;
  uint8_t xh                     : 1;
  uint8_t yl                     : 1;
  uint8_t yh                     : 1;
  uint8_t zl                     : 1;
  uint8_t zh                     : 1;
  uint8_t ia                     : 1;
  uint8_t not_used_01            : 1;
} lsm303agr_int1_src_a_t;


typedef struct {
  uint8_t ths                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int1_ths_a_t;


typedef struct {
  uint8_t d                      : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int1_duration_a_t;


typedef struct {
  uint8_t xlie                   : 1;
  uint8_t xhie                   : 1;
  uint8_t ylie                   : 1;
  uint8_t yhie                   : 1;
  uint8_t zlie                   : 1;
  uint8_t zhie                   : 1;
  uint8_t _6d                    : 1;
  uint8_t aoi                    : 1;
} lsm303agr_int2_cfg_a_t;


typedef struct {
  uint8_t xl                     : 1;
  uint8_t xh                     : 1;
  uint8_t yl                     : 1;
  uint8_t yh                     : 1;
  uint8_t zl                     : 1;
  uint8_t zh                     : 1;
  uint8_t ia                     : 1;
  uint8_t not_used_01            : 1;
} lsm303agr_int2_src_a_t;


typedef struct {
  uint8_t ths                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int2_ths_a_t;


typedef struct {
  uint8_t d                      : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int2_duration_a_t;


typedef struct {
  uint8_t xs                     : 1;
  uint8_t xd                     : 1;
  uint8_t ys                     : 1;
  uint8_t yd                     : 1;
  uint8_t zs                     : 1;
  uint8_t zd                     : 1;
  uint8_t not_used_01            : 2;
} lsm303agr_click_cfg_a_t;


typedef struct {
  uint8_t x                      : 1;
  uint8_t y                      : 1;
  uint8_t z                      : 1;
  uint8_t sign                   : 1;
  uint8_t sclick                 : 1;
  uint8_t dclick                 : 1;
  uint8_t ia                     : 1;
  uint8_t not_used_01            : 1;
} lsm303agr_click_src_a_t;


typedef struct {
  uint8_t ths                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_click_ths_a_t;


typedef struct {
  uint8_t tli                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_time_limit_a_t;


typedef struct {
  uint8_t tla                    : 8;
} lsm303agr_time_latency_a_t;


typedef struct {
  uint8_t tw                     : 8;
} lsm303agr_time_window_a_t;


typedef struct {
  uint8_t acth                   : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_act_ths_a_t;


typedef struct {
  uint8_t actd                   : 8;
} lsm303agr_act_dur_a_t;

#line 414 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"
typedef struct {
  uint8_t md                     : 2;
  uint8_t odr                    : 2;
  uint8_t lp                     : 1;
  uint8_t soft_rst               : 1;
  uint8_t reboot                 : 1;
  uint8_t comp_temp_en           : 1;
} lsm303agr_cfg_reg_a_m_t;


typedef struct {
  uint8_t lpf                    : 1;
  uint8_t set_rst                : 2;  
  uint8_t int_on_dataoff         : 1;
  uint8_t off_canc_one_shot      : 1;
  uint8_t not_used_01            : 3;
} lsm303agr_cfg_reg_b_m_t;


typedef struct {
  uint8_t int_mag                : 1;
  uint8_t self_test              : 1;
  uint8_t not_used_01            : 1;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
  uint8_t i2c_dis                : 1;
  uint8_t int_mag_pin            : 1;
  uint8_t not_used_02            : 1;
} lsm303agr_cfg_reg_c_m_t;


typedef struct {
  uint8_t ien                    : 1;
  uint8_t iel                    : 1;
  uint8_t iea                    : 1;
  uint8_t not_used_01            : 2;
  uint8_t zien                   : 1;
  uint8_t yien                   : 1;
  uint8_t xien                   : 1;
} lsm303agr_int_crtl_reg_m_t;


typedef struct {
  uint8_t _int                    : 1;
  uint8_t mroi                   : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_x               : 1;
} lsm303agr_int_source_reg_m_t;




typedef struct {
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t _xor                   : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
} lsm303agr_status_reg_m_t;

#line 487 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"












 
typedef union{
  lsm303agr_status_reg_aux_a_t       status_reg_aux_a;
  lsm303agr_temp_cfg_reg_a_t         temp_cfg_reg_a;
  lsm303agr_ctrl_reg1_a_t            ctrl_reg1_a;
  lsm303agr_ctrl_reg2_a_t            ctrl_reg2_a;
  lsm303agr_ctrl_reg3_a_t            ctrl_reg3_a;
  lsm303agr_ctrl_reg4_a_t            ctrl_reg4_a;
  lsm303agr_ctrl_reg5_a_t            ctrl_reg5_a;
  lsm303agr_ctrl_reg6_a_t            ctrl_reg6_a;
  lsm303agr_status_reg_a_t           status_reg_a;
  lsm303agr_fifo_ctrl_reg_a_t        fifo_ctrl_reg_a;
  lsm303agr_fifo_src_reg_a_t         fifo_src_reg_a;
  lsm303agr_int1_cfg_a_t             int1_cfg_a;
  lsm303agr_int1_src_a_t             int1_src_a;
  lsm303agr_int1_ths_a_t             int1_ths_a;
  lsm303agr_int1_duration_a_t        int1_duration_a;
  lsm303agr_int2_cfg_a_t             int2_cfg_a;
  lsm303agr_int2_src_a_t             int2_src_a;
  lsm303agr_int2_ths_a_t             int2_ths_a;
  lsm303agr_int2_duration_a_t        int2_duration_a;
  lsm303agr_click_cfg_a_t            click_cfg_a;
  lsm303agr_click_src_a_t            click_src_a;
  lsm303agr_click_ths_a_t            click_ths_a;
  lsm303agr_time_limit_a_t           time_limit_a;
  lsm303agr_time_latency_a_t         time_latency_a;
  lsm303agr_time_window_a_t          time_window_a;
  lsm303agr_act_ths_a_t              act_ths_a;
  lsm303agr_act_dur_a_t              act_dur_a;
  lsm303agr_cfg_reg_a_m_t            cfg_reg_a_m;
  lsm303agr_cfg_reg_b_m_t            cfg_reg_b_m;
  lsm303agr_cfg_reg_c_m_t            cfg_reg_c_m;
  lsm303agr_int_crtl_reg_m_t         int_crtl_reg_m;
  lsm303agr_int_source_reg_m_t       int_source_reg_m;
  lsm303agr_status_reg_m_t           status_reg_m;
  bitwise_t                          bitwise;
  uint8_t                            byte;
} lsm303agr_reg_t;




 

int32_t lsm303agr_read_reg(lsm303agr_ctx_t *ctx, uint8_t reg, uint8_t* data,
                           uint16_t len);
int32_t lsm303agr_write_reg(lsm303agr_ctx_t *ctx, uint8_t reg, uint8_t* data,
                           uint16_t len);

extern float_t lsm303agr_from_fs_2g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_4g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_8g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_16g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_lsb_hr_to_celsius(int16_t lsb);

extern float_t lsm303agr_from_fs_2g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_4g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_8g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_16g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_lsb_nm_to_celsius(int16_t lsb);

extern float_t lsm303agr_from_fs_2g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_4g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_8g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_16g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_lsb_lp_to_celsius(int16_t lsb);

extern float_t lsm303agr_from_lsb_to_mgauss(int16_t lsb);

int32_t lsm303agr_temp_status_reg_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_temp_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_temp_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_temperature_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LSM303AGR_TEMP_DISABLE  = 0,
  LSM303AGR_TEMP_ENABLE   = 3,
} lsm303agr_temp_en_a_t;
int32_t lsm303agr_temperature_meas_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_temp_en_a_t val);
int32_t lsm303agr_temperature_meas_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_temp_en_a_t *val);

typedef enum {
  LSM303AGR_HR_12bit   = 0,
  LSM303AGR_NM_10bit   = 1,
  LSM303AGR_LP_8bit    = 2,
} lsm303agr_op_md_a_t;
int32_t lsm303agr_xl_operating_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t val);
int32_t lsm303agr_xl_operating_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t *val);

typedef enum {
  LSM303AGR_XL_POWER_DOWN                      = 0,
  LSM303AGR_XL_ODR_1Hz                         = 1,
  LSM303AGR_XL_ODR_10Hz                        = 2,
  LSM303AGR_XL_ODR_25Hz                        = 3,
  LSM303AGR_XL_ODR_50Hz                        = 4,
  LSM303AGR_XL_ODR_100Hz                       = 5,
  LSM303AGR_XL_ODR_200Hz                       = 6,
  LSM303AGR_XL_ODR_400Hz                       = 7,
  LSM303AGR_XL_ODR_1kHz620_LP                  = 8,
  LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP    = 9,
} lsm303agr_odr_a_t;
int32_t lsm303agr_xl_data_rate_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t val);
int32_t lsm303agr_xl_data_rate_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t *val);

int32_t lsm303agr_xl_high_pass_on_outputs_set(lsm303agr_ctx_t *ctx,
                                              uint8_t val);
int32_t lsm303agr_xl_high_pass_on_outputs_get(lsm303agr_ctx_t *ctx,
                                              uint8_t *val);

typedef enum {
  LSM303AGR_AGGRESSIVE  = 0,
  LSM303AGR_STRONG      = 1,
  LSM303AGR_MEDIUM      = 2,
  LSM303AGR_LIGHT       = 3,
} lsm303agr_hpcf_a_t;
int32_t lsm303agr_xl_high_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t val);
int32_t lsm303agr_xl_high_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t *val);

typedef enum {
  LSM303AGR_NORMAL_WITH_RST  = 0,
  LSM303AGR_REFERENCE_MODE   = 1,
  LSM303AGR_NORMAL           = 2,
  LSM303AGR_AUTORST_ON_INT   = 3,
} lsm303agr_hpm_a_t;
int32_t lsm303agr_xl_high_pass_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t val);
int32_t lsm303agr_xl_high_pass_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t *val);

typedef enum {
  LSM303AGR_2g   = 0,
  LSM303AGR_4g   = 1,
  LSM303AGR_8g   = 2,
  LSM303AGR_16g  = 3,
} lsm303agr_fs_a_t;
int32_t lsm303agr_xl_full_scale_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t val);
int32_t lsm303agr_xl_full_scale_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t *val);

int32_t lsm303agr_xl_block_data_update_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_xl_block_data_update_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

int32_t lsm303agr_xl_filter_reference_set(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff);
int32_t lsm303agr_xl_filter_reference_get(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff);

int32_t lsm303agr_xl_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_acceleration_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_xl_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LSM303AGR_ST_DISABLE   = 0,
  LSM303AGR_ST_POSITIVE  = 1,
  LSM303AGR_ST_NEGATIVE  = 2,
} lsm303agr_st_a_t;
int32_t lsm303agr_xl_self_test_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t val);
int32_t lsm303agr_xl_self_test_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t *val);

typedef enum {
  LSM303AGR_XL_LSB_AT_LOW_ADD = 0,
  LSM303AGR_XL_MSB_AT_LOW_ADD = 1,
} lsm303agr_ble_a_t;
int32_t lsm303agr_xl_data_format_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t val);
int32_t lsm303agr_xl_data_format_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t *val);

int32_t lsm303agr_xl_boot_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_xl_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_status_get(lsm303agr_ctx_t *ctx,
                                lsm303agr_status_reg_a_t *val);

int32_t lsm303agr_xl_int1_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int1_cfg_a_t *val);
int32_t lsm303agr_xl_int1_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int1_cfg_a_t *val);

int32_t lsm303agr_xl_int1_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int1_src_a_t *val);

int32_t lsm303agr_xl_int1_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int1_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm303agr_xl_int1_gen_duration_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_xl_int1_gen_duration_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

int32_t lsm303agr_xl_int2_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int2_cfg_a_t *val);
int32_t lsm303agr_xl_int2_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int2_cfg_a_t *val);

int32_t lsm303agr_xl_int2_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int2_src_a_t *val);

int32_t lsm303agr_xl_int2_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int2_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm303agr_xl_int2_gen_duration_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_xl_int2_gen_duration_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

typedef enum {
  LSM303AGR_DISC_FROM_INT_GENERATOR  = 0,
  LSM303AGR_ON_INT1_GEN              = 1,
  LSM303AGR_ON_INT2_GEN              = 2,
  LSM303AGR_ON_TAP_GEN               = 4,
  LSM303AGR_ON_INT1_INT2_GEN         = 3,
  LSM303AGR_ON_INT1_TAP_GEN          = 5,
  LSM303AGR_ON_INT2_TAP_GEN          = 6,
  LSM303AGR_ON_INT1_INT2_TAP_GEN     = 7,
} lsm303agr_hp_a_t;
int32_t lsm303agr_xl_high_pass_int_conf_set(lsm303agr_ctx_t *ctx,
                                            lsm303agr_hp_a_t val);
int32_t lsm303agr_xl_high_pass_int_conf_get(lsm303agr_ctx_t *ctx,
                                            lsm303agr_hp_a_t *val);

int32_t lsm303agr_xl_pin_int1_config_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg3_a_t *val);
int32_t lsm303agr_xl_pin_int1_config_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg3_a_t *val);

int32_t lsm303agr_xl_int2_pin_detect_4d_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int2_pin_detect_4d_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

typedef enum {
  LSM303AGR_INT2_PULSED   = 0,
  LSM303AGR_INT2_LATCHED  = 1,
} lsm303agr_lir_int2_a_t;
int32_t lsm303agr_xl_int2pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int2_a_t val);
int32_t lsm303agr_xl_int2pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int2_a_t *val);

int32_t lsm303agr_xl_int1_pin_detect_4d_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int1_pin_detect_4d_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

typedef enum {
  LSM303AGR_INT1_PULSED   = 0,
  LSM303AGR_INT1_LATCHED  = 1,
} lsm303agr_lir_int1_a_t;
int32_t lsm303agr_xl_int1pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int1_a_t val);
int32_t lsm303agr_xl_int1pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int1_a_t *val);

int32_t lsm303agr_xl_pin_int2_config_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg6_a_t *val);
int32_t lsm303agr_xl_pin_int2_config_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg6_a_t *val);

int32_t lsm303agr_xl_fifo_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_xl_fifo_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_watermark_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_xl_fifo_watermark_get(lsm303agr_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM303AGR_INT1_GEN = 0,
  LSM303AGR_INT2_GEN = 1,
} lsm303agr_tr_a_t;
int32_t lsm303agr_xl_fifo_trigger_event_set(lsm303agr_ctx_t *ctx,
                                            lsm303agr_tr_a_t val);
int32_t lsm303agr_xl_fifo_trigger_event_get(lsm303agr_ctx_t *ctx,
                                            lsm303agr_tr_a_t *val);

typedef enum {
  LSM303AGR_BYPASS_MODE           = 0,
  LSM303AGR_FIFO_MODE             = 1,
  LSM303AGR_DYNAMIC_STREAM_MODE   = 2,
  LSM303AGR_STREAM_TO_FIFO_MODE   = 3,
} lsm303agr_fm_a_t;
int32_t lsm303agr_xl_fifo_mode_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t val);
int32_t lsm303agr_xl_fifo_mode_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t *val);

int32_t lsm303agr_xl_fifo_status_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_fifo_src_reg_a_t *val);

int32_t lsm303agr_xl_fifo_data_level_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_empty_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_ovr_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_fth_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_tap_conf_set(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val);
int32_t lsm303agr_tap_conf_get(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val);

int32_t lsm303agr_tap_source_get(lsm303agr_ctx_t *ctx,
                                lsm303agr_click_src_a_t *val);

int32_t lsm303agr_tap_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_tap_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_shock_dur_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_shock_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_quiet_dur_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_quiet_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_double_tap_timeout_set(lsm303agr_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm303agr_double_tap_timeout_get(lsm303agr_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm303agr_act_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_act_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_act_timeout_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_act_timeout_get(lsm303agr_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM303AGR_SPI_4_WIRE = 0,
  LSM303AGR_SPI_3_WIRE = 1,
} lsm303agr_sim_a_t;
int32_t lsm303agr_xl_spi_mode_set(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t val);
int32_t lsm303agr_xl_spi_mode_get(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t *val);

int32_t lsm303agr_mag_user_offset_set(lsm303agr_ctx_t *ctx,
                                      uint8_t *buff);
int32_t lsm303agr_mag_user_offset_get(lsm303agr_ctx_t *ctx,
                                      uint8_t *buff);

typedef enum {
  LSM303AGR_CONTINUOUS_MODE  = 0,
  LSM303AGR_SINGLE_TRIGGER   = 1,
  LSM303AGR_POWER_DOWN       = 2,
} lsm303agr_md_m_t;
int32_t lsm303agr_mag_operating_mode_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t val);
int32_t lsm303agr_mag_operating_mode_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t *val);

typedef enum {
  LSM303AGR_MG_ODR_10Hz   = 0,
  LSM303AGR_MG_ODR_20Hz   = 1,
  LSM303AGR_MG_ODR_50Hz   = 2,
  LSM303AGR_MG_ODR_100Hz  = 3,
} lsm303agr_mg_odr_m_t;
int32_t lsm303agr_mag_data_rate_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t val);
int32_t lsm303agr_mag_data_rate_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t *val);

typedef enum {
  LSM303AGR_HIGH_RESOLUTION  = 0,
  LSM303AGR_LOW_POWER        = 1,
} lsm303agr_lp_m_t;
int32_t lsm303agr_mag_power_mode_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t val);
int32_t lsm303agr_mag_power_mode_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t *val);

int32_t lsm303agr_mag_offset_temp_comp_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_mag_offset_temp_comp_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

typedef enum {
  LSM303AGR_ODR_DIV_2  = 0,
  LSM303AGR_ODR_DIV_4  = 1,
} lsm303agr_lpf_m_t;
int32_t lsm303agr_mag_low_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t val);
int32_t lsm303agr_mag_low_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t *val);

typedef enum {
  LSM303AGR_SET_SENS_ODR_DIV_63        = 0,
  LSM303AGR_SENS_OFF_CANC_EVERY_ODR    = 1,
  LSM303AGR_SET_SENS_ONLY_AT_POWER_ON  = 2,
} lsm303agr_set_rst_m_t;
int32_t lsm303agr_mag_set_rst_mode_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t val);
int32_t lsm303agr_mag_set_rst_mode_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t *val);

int32_t lsm303agr_mag_set_rst_sensor_single_set(lsm303agr_ctx_t *ctx,
                                                uint8_t val);
int32_t lsm303agr_mag_set_rst_sensor_single_get(lsm303agr_ctx_t *ctx,
                                                uint8_t *val);

int32_t lsm303agr_mag_block_data_update_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_mag_block_data_update_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm303agr_mag_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_magnetic_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_mag_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_mag_reset_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_reset_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_boot_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_self_test_set(lsm303agr_ctx_t *ctx,
                                    uint8_t val);
int32_t lsm303agr_mag_self_test_get(lsm303agr_ctx_t *ctx,
                                    uint8_t *val);

typedef enum {
  LSM303AGR_MG_LSB_AT_LOW_ADD  = 0,
  LSM303AGR_MG_MSB_AT_LOW_ADD  = 1,
} lsm303agr_ble_m_t;
int32_t lsm303agr_mag_data_format_set(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t val);
int32_t lsm303agr_mag_data_format_get(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t *val);

int32_t lsm303agr_mag_status_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_status_reg_m_t *val);

typedef enum {
  LSM303AGR_CHECK_BEFORE  = 0,
  LSM303AGR_CHECK_AFTER   = 1,
} lsm303agr_int_on_dataoff_m_t;
int32_t lsm303agr_mag_offset_int_conf_set(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t val);
int32_t lsm303agr_mag_offset_int_conf_get(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t *val);

int32_t lsm303agr_mag_drdy_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_drdy_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_int_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_int_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_int_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int_crtl_reg_m_t *val);
int32_t lsm303agr_mag_int_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int_crtl_reg_m_t *val);

int32_t lsm303agr_mag_int_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int_source_reg_m_t *val);

int32_t lsm303agr_mag_int_gen_treshold_set(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff);
int32_t lsm303agr_mag_int_gen_treshold_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff);
typedef enum {
  LSM303AGR_I2C_ENABLE   = 0,
  LSM303AGR_I2C_DISABLE  = 1,
} lsm303agr_i2c_dis_m_t;
int32_t lsm303agr_mag_i2c_interface_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t val);
int32_t lsm303agr_mag_i2c_interface_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t *val);




 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"



 



 



 



 

typedef int32_t (*LSM303AGR_Init_Func)(void);
typedef int32_t (*LSM303AGR_DeInit_Func)(void);
typedef int32_t (*LSM303AGR_GetTick_Func)(void);
typedef int32_t (*LSM303AGR_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LSM303AGR_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LSM303AGR_Init_Func          Init;
  LSM303AGR_DeInit_Func        DeInit;
  uint32_t                     BusType;  
  uint8_t                      Address;
  LSM303AGR_WriteReg_Func      WriteReg;
  LSM303AGR_ReadReg_Func       ReadReg;
  LSM303AGR_GetTick_Func       GetTick;
} LSM303AGR_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LSM303AGR_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM303AGR_Axes_t;

typedef struct
{
  LSM303AGR_IO_t        IO;
  lsm303agr_ctx_t       Ctx;
  uint8_t               is_initialized;
  uint8_t               acc_is_enabled;
  lsm303agr_odr_a_t     acc_odr;
} LSM303AGR_ACC_Object_t;

typedef struct
{
  LSM303AGR_IO_t        IO;
  lsm303agr_ctx_t       Ctx;
  uint8_t               is_initialized;
  uint8_t               mag_is_enabled;
} LSM303AGR_MAG_Object_t;

typedef struct
{
  uint8_t   Acc;
  uint8_t   Gyro;
  uint8_t   Magneto;
  uint8_t   LowPower;
  uint32_t  GyroMaxFS;
  uint32_t  AccMaxFS;
  uint32_t  MagMaxFS;
  float     GyroMaxOdr;
  float     AccMaxOdr;
  float     MagMaxOdr;
} LSM303AGR_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LSM303AGR_ACC_Object_t *);
  int32_t (*DeInit)(LSM303AGR_ACC_Object_t *);
  int32_t (*ReadID)(LSM303AGR_ACC_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM303AGR_ACC_Object_t *, LSM303AGR_Capabilities_t *);
} LSM303AGR_ACC_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM303AGR_ACC_Object_t *);
  int32_t (*Disable)(LSM303AGR_ACC_Object_t *);
  int32_t (*GetSensitivity)(LSM303AGR_ACC_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM303AGR_ACC_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM303AGR_ACC_Object_t *, float);
  int32_t (*GetFullScale)(LSM303AGR_ACC_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM303AGR_ACC_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM303AGR_ACC_Object_t *, LSM303AGR_Axes_t *);
  int32_t (*GetAxesRaw)(LSM303AGR_ACC_Object_t *, LSM303AGR_AxesRaw_t *);
} LSM303AGR_ACC_Drv_t;

typedef struct
{
  int32_t (*Init)(LSM303AGR_MAG_Object_t *);
  int32_t (*DeInit)(LSM303AGR_MAG_Object_t *);
  int32_t (*ReadID)(LSM303AGR_MAG_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM303AGR_MAG_Object_t *, LSM303AGR_Capabilities_t *);
} LSM303AGR_MAG_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM303AGR_MAG_Object_t *);
  int32_t (*Disable)(LSM303AGR_MAG_Object_t *);
  int32_t (*GetSensitivity)(LSM303AGR_MAG_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM303AGR_MAG_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM303AGR_MAG_Object_t *, float);
  int32_t (*GetFullScale)(LSM303AGR_MAG_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM303AGR_MAG_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM303AGR_MAG_Object_t *, LSM303AGR_Axes_t *);
  int32_t (*GetAxesRaw)(LSM303AGR_MAG_Object_t *, LSM303AGR_AxesRaw_t *);
} LSM303AGR_MAG_Drv_t;



 



 







#line 196 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"





 



 

int32_t LSM303AGR_ACC_RegisterBusIO(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_IO_t *pIO);
int32_t LSM303AGR_ACC_Init(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_DeInit(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_ReadID(LSM303AGR_ACC_Object_t *pObj, uint8_t *Id);
int32_t LSM303AGR_ACC_GetCapabilities(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities);

int32_t LSM303AGR_ACC_Enable(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_Disable(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_GetSensitivity(LSM303AGR_ACC_Object_t *pObj, float *sensitivity);
int32_t LSM303AGR_ACC_GetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float *odr);
int32_t LSM303AGR_ACC_SetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float odr);
int32_t LSM303AGR_ACC_GetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t *fullscale);
int32_t LSM303AGR_ACC_SetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t fullscale);
int32_t LSM303AGR_ACC_GetAxes(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Axes_t *acceleration);
int32_t LSM303AGR_ACC_GetAxesRaw(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_AxesRaw_t *value);

int32_t LSM303AGR_ACC_Read_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t LSM303AGR_ACC_Write_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t LSM303AGR_ACC_Get_DRDY_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *status);
int32_t LSM303AGR_ACC_Get_Init_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *status);

int32_t LSM303AGR_MAG_RegisterBusIO(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_IO_t *pIO);
int32_t LSM303AGR_MAG_Init(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_DeInit(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_ReadID(LSM303AGR_MAG_Object_t *pObj, uint8_t *id);
int32_t LSM303AGR_MAG_GetCapabilities(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities);

int32_t LSM303AGR_MAG_Enable(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_Disable(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_GetSensitivity(LSM303AGR_MAG_Object_t *pObj, float *sensitivity);
int32_t LSM303AGR_MAG_GetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float *odr);
int32_t LSM303AGR_MAG_SetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float odr);
int32_t LSM303AGR_MAG_GetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t *fullscale);
int32_t LSM303AGR_MAG_SetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t fullscale);
int32_t LSM303AGR_MAG_GetAxes(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Axes_t *magnetic_field);
int32_t LSM303AGR_MAG_GetAxesRaw(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_AxesRaw_t *value);

int32_t LSM303AGR_MAG_Read_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t LSM303AGR_MAG_Write_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t LSM303AGR_MAG_Get_DRDY_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *status);
int32_t LSM303AGR_MAG_Get_Init_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *status);



 



 

extern LSM303AGR_ACC_CommonDrv_t LSM303AGR_ACC_COMMON_Driver;
extern LSM303AGR_ACC_Drv_t LSM303AGR_ACC_Driver;
extern LSM303AGR_MAG_CommonDrv_t LSM303AGR_MAG_COMMON_Driver;
extern LSM303AGR_MAG_Drv_t LSM303AGR_MAG_Driver;



 









 



 



 

 
#line 120 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"
















































 



 



 



 

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} IKS01A2_MOTION_SENSOR_Axes_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} IKS01A2_MOTION_SENSOR_AxesRaw_t;

 
typedef struct
{
  uint8_t  Acc;
  uint8_t  Gyro;
  uint8_t  Magneto;
  uint8_t  LowPower;
  uint32_t GyroMaxFS;
  uint32_t AccMaxFS;
  uint32_t MagMaxFS;
  float    GyroMaxOdr;
  float    AccMaxOdr;
  float    MagMaxOdr;
} IKS01A2_MOTION_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} IKS01A2_MOTION_SENSOR_Ctx_t;



 



 




















#line 249 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 257 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 266 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 276 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 287 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 299 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 312 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 326 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 341 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 357 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 367 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"

#line 383 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors.h"







 



 

int32_t IKS01A2_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t IKS01A2_MOTION_SENSOR_DeInit(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_GetCapabilities(uint32_t Instance, IKS01A2_MOTION_SENSOR_Capabilities_t *Capabilities);
int32_t IKS01A2_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t IKS01A2_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, IKS01A2_MOTION_SENSOR_Axes_t *Axes);
int32_t IKS01A2_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, IKS01A2_MOTION_SENSOR_AxesRaw_t *Axes);
int32_t IKS01A2_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, float *Sensitivity);
int32_t IKS01A2_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t IKS01A2_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t IKS01A2_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale);
int32_t IKS01A2_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale);



 



 



 



 







 
#line 71 "..\\Inc\\main.h"
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors_ex.h"











































 

 







 
#line 56 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\IKS01A2\\iks01a2_motion_sensors_ex.h"



 



 



 



 

typedef enum
{
  IKS01A2_MOTION_SENSOR_INT1_PIN = 0,
  IKS01A2_MOTION_SENSOR_INT2_PIN
} IKS01A2_MOTION_SENSOR_IntPin_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} IKS01A2_MOTION_SENSOR_Event_Status_t;



 



 

int32_t IKS01A2_MOTION_SENSOR_Enable_HP_Filter(uint32_t Instance, uint8_t CutOff);
int32_t IKS01A2_MOTION_SENSOR_Set_INT1_DRDY(uint32_t Instance,uint8_t Status);
int32_t IKS01A2_MOTION_SENSOR_Set_INT2_DRDY(uint32_t Instance,uint8_t Status);
int32_t IKS01A2_MOTION_SENSOR_DRDY_Set_Mode(uint32_t Instance,uint8_t Status);
int32_t IKS01A2_MOTION_SENSOR_Clear_DRDY(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Enable_6D_Orientation(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin);
int32_t IKS01A2_MOTION_SENSOR_Disable_6D_Orientation(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Set_6D_Orientation_Threshold(uint32_t Instance, uint8_t Threshold);
int32_t IKS01A2_MOTION_SENSOR_Enable_Free_Fall_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin);
int32_t IKS01A2_MOTION_SENSOR_Disable_Free_Fall_Detection(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Set_Free_Fall_Threshold(uint32_t Instance, uint8_t Threshold);
int32_t IKS01A2_MOTION_SENSOR_Set_Free_Fall_Duration(uint32_t Instance, uint8_t Duration);
int32_t IKS01A2_MOTION_SENSOR_Enable_Pedometer(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Disable_Pedometer(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Set_Pedometer_Threshold(uint32_t Instance, uint8_t Threshold);
int32_t IKS01A2_MOTION_SENSOR_Reset_Step_Counter(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Enable_Single_Tap_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin);
int32_t IKS01A2_MOTION_SENSOR_Disable_Single_Tap_Detection(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Enable_Double_Tap_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin);
int32_t IKS01A2_MOTION_SENSOR_Disable_Double_Tap_Detection(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Threshold(uint32_t Instance, uint8_t Threshold);
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Shock_Time(uint32_t Instance, uint8_t Time);
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Quiet_Time(uint32_t Instance, uint8_t Time);
int32_t IKS01A2_MOTION_SENSOR_Set_Tap_Duration_Time(uint32_t Instance, uint8_t Time);
int32_t IKS01A2_MOTION_SENSOR_Enable_Tilt_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin);
int32_t IKS01A2_MOTION_SENSOR_Disable_Tilt_Detection(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Enable_Wake_Up_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin);
int32_t IKS01A2_MOTION_SENSOR_Disable_Wake_Up_Detection(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Set_Wake_Up_Threshold(uint32_t Instance, uint8_t Threshold);
int32_t IKS01A2_MOTION_SENSOR_Set_Wake_Up_Duration(uint32_t Instance, uint8_t Duration);
int32_t IKS01A2_MOTION_SENSOR_Enable_Inactivity_Detection(uint32_t Instance, IKS01A2_MOTION_SENSOR_IntPin_t IntPin);
int32_t IKS01A2_MOTION_SENSOR_Disable_Inactivity_Detection(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_Set_Sleep_Duration(uint32_t Instance, uint8_t Duration);
int32_t IKS01A2_MOTION_SENSOR_Get_Event_Status(uint32_t Instance, IKS01A2_MOTION_SENSOR_Event_Status_t *Status);
int32_t IKS01A2_MOTION_SENSOR_Get_DRDY_Status(uint32_t Instance, uint32_t Function, uint8_t *Status);
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XL(uint32_t Instance, uint8_t *xl);
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XH(uint32_t Instance, uint8_t *xh);
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YL(uint32_t Instance, uint8_t *yl);
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YH(uint32_t Instance, uint8_t *yh);
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZL(uint32_t Instance, uint8_t *zl);
int32_t IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZH(uint32_t Instance, uint8_t *zh);
int32_t IKS01A2_MOTION_SENSOR_Get_Step_Count(uint32_t Instance, uint16_t *StepCount);
int32_t IKS01A2_MOTION_SENSOR_Read_Register(uint32_t Instance, uint8_t Reg, uint8_t *Data);
int32_t IKS01A2_MOTION_SENSOR_Write_Register(uint32_t Instance, uint8_t Reg, uint8_t Data);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Num_Samples(uint32_t Instance, uint16_t *NumSamples);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Full_Status(uint32_t Instance, uint8_t *Status);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Decimation(uint32_t Instance, uint32_t Function, uint8_t Decimation);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_ODR_Value(uint32_t Instance, float Odr);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_INT1_FIFO_Full(uint32_t Instance, uint8_t Status);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_INT2_FIFO_Full(uint32_t Instance, uint8_t Status);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Watermark_Level(uint32_t Instance, uint16_t Watermark);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Stop_On_Fth(uint32_t Instance, uint8_t Status);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Set_Mode(uint32_t Instance, uint8_t Mode);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Pattern(uint32_t Instance, uint16_t *Pattern);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Axis(uint32_t Instance, uint32_t Function, int32_t *Data);
int32_t IKS01A2_MOTION_SENSOR_FIFO_Get_Data_Word(uint32_t Instance,uint32_t Function,int16_t *Data);
int32_t IKS01A2_MOTION_SENSOR_Set_SelfTest(uint32_t Instance, uint32_t Function, uint8_t Status);



 



 



 



 







 
#line 72 "..\\Inc\\main.h"


 
 



 
 


 




 



#line 106 "..\\Inc\\main.h"

 
#line 115 "..\\Inc\\main.h"

 
extern uint32_t SensorsEnabled;
extern TIM_HandleTypeDef AlgoTimHandle;

extern IKS01A2_MOTION_SENSOR_Axes_t AccValue;
extern IKS01A2_MOTION_SENSOR_Axes_t MagValue;
extern IKS01A2_MOTION_SENSOR_Axes_t MagValueComp;

extern int UseLSI;

 
void Error_Handler(void);
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw);
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);







 
#line 48 "LSM9DS1\\i2c.h"

 

 

extern I2C_HandleTypeDef hi2c2;

 

 

void MX_I2C2_Init(void);

 

 








 



 

 
#line 3 "LSM9DS1\\LSM9DS1.c"
#line 4 "LSM9DS1\\LSM9DS1.c"
#line 5 "LSM9DS1\\LSM9DS1.c"
extern I2C_HandleTypeDef hi2c3;

extern  HAL_StatusTypeDef status;

void init_LSM9DS1_I2C ()
{
	
	uint8_t data[1];
	data[0] = 0x81;
	status = HAL_I2C_Mem_Write(&hi2c3, 0xD6, 0x20, 0x00000001U , data, 0x01, 100);
	HAL_Delay(10);
	
	status = HAL_I2C_Mem_Read(&hi2c3, 0xD6, 0x0F, 0x00000001U , data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0xC3;
	HAL_I2C_Mem_Write(&hi2c3, 0xD6, 0x10, 0x00000001U , data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0xC3;
	HAL_I2C_Mem_Write(&hi2c3, 0xD6, 0x12, 0x00000001U , data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0x38;
	HAL_I2C_Mem_Write(&hi2c3, 0xD6, 0x1E,0x00000001U , data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0x38;
  HAL_I2C_Mem_Write(&hi2c3, 0xD6, 0x1F, 0x00000001U , data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0xC0;
  HAL_I2C_Mem_Write(&hi2c3, 0xD6, 0x20, 0x00000001U , data, 0x01, 100);
	HAL_Delay(10);
  HAL_Delay(1);
	
	data[0] = 0x04;
	status = HAL_I2C_Mem_Write(&hi2c3, 0x3D, 0x20, 0x00000001U, data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0x04;
	status = HAL_I2C_Mem_Write(&hi2c3, 0x3D, 0x21,0x00000001U, data, 0x01, 100);
	HAL_Delay(10);
	status = HAL_I2C_Mem_Read(&hi2c3, 0x3D, 0x0F, 0x00000001U, data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0x7E;
	status = HAL_I2C_Mem_Write(&hi2c3, 0x3D, 0x20,0x00000001U, data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0x00;
	status = HAL_I2C_Mem_Write(&hi2c3, 0x3D, 0x22,0x00000001U, data, 0x01, 100);
	HAL_Delay(10);
	data[0] = 0x0C;
	status = HAL_I2C_Mem_Write(&hi2c3, 0x3D, 0x23,0x00000001U, data, 0x01, 100);
	HAL_Delay(10);
}


