// HSE 72 MHz internal clock
 
#include "stm32f10x_conf.h"

 
#define F_CPU           72000000   // 1MHz
#define PRESCALE        64
//#define PERIOD1         250 // microseconds - 1/2 second delay
#define PERIOD2		 100 // microseconds - 1/4 second delay
#define TCLKS1          ((F_CPU/PRESCALE*PERIOD2)/1000)

#define SystemCoreClock 72000000

#define PCLK1		36000000
//#define PERIOD1		10000 // 10ms
#define PERIOD1		1000
#define TCLKS2		((PCLK1/PRESCALE*PERIOD1)/1000)
 
volatile uint8_t flag1 = 1;
volatile uint8_t flag2 = 1;

void TIM_Config(void);

void TIM1_CC_IRQHandler(void)
{
  if (TIM1->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    //if(flag1)
    //  flag1 = 0;
    //else
    //  flag1 = 1;

    //TIM1->CCR1 += TCLKS1;        // next interrupt time
    TIM1->SR &= ~TIM_SR_CC1IF;   // clear CC1 int flag (write 0)
  }
  if (TIM1->SR & TIM_SR_CC2IF)  // CC1 match?
  {
    if(flag2)
      flag2 = 0;
    else
      flag2 = 1;

    TIM1->CCR2 += TCLKS1;        // next interrupt time
    TIM1->SR &= ~TIM_SR_CC2IF;   // clear CC1 int flag (write 0)
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_CC1IF){
    detectEmergency();
    if(flag1)
      flag1 = 0;
    else
      flag1 = 1;
    TIM2->SR &= ~TIM_SR_CC1IF;
    TIM2->CCR1 += TCLKS1;
  }
}

void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_CC1IF){
    refreshSensorData();
    TIM3->SR &= ~TIM_SR_CC1IF;
  }
}

void TIM4_IRQHandler(void)
{
  if (TIM4->SR & TIM_SR_CC1IF){
    calculateOrientation();
    TIM4->SR &= ~TIM_SR_CC1IF;
  }
}

void HSE_Configuration(void){
  
  // RCC system reset
  RCC_DeInit();
  
  RCC_HSEConfig(RCC_HSE_ON);
  
  RCC_WaitForHSEStartUp();
  
  // HCLK = SYSCLK
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  
  // PCLK2 = HCLK
  RCC_PCLK2Config(RCC_HCLK_Div1);
  
  // PCLK1 = HCLK
  //RCC_PCLK1Config(RCC_HCLK_Div1);
  
  // PLL selected as system clock
  // RCC->CFGR &= ~RCC_CFGR_SW_0;
  // RCC->CFGR |= RCC_CFGR_SW_1;
  
  // while(RCC_GetSYSCLKSource() != 0x04);
  
  // PREDIV1SCR
  // PREDIV1
  // PLLSCR
  // PLLMUL
  // SW
  
  RCC->CR |= RCC_CR_PLLON;// | RCC_CR_PLL2ON | RCC_CR_PLL3ON;
  //RCC->CR &= ~RCC_CR_PLLON;
  
  // PLLXTPRE
  RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;
  
  // PLLSRC select HSE from PLLXTPRE
  RCC->CFGR |= RCC_CFGR_PLLSRC;
  
  // PLLMUL input clk x4 (0010)
  RCC->CFGR &= ~(RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_2 | RCC_CFGR_PLLMULL_3);
  RCC->CFGR |= RCC_CFGR_PLLMULL_1;
  
  // PLL selected as system clock
  RCC->CFGR &= ~RCC_CFGR_SW_0;
  RCC->CFGR |= RCC_CFGR_SW_1;
  
  RCC->CFGR &= ~(RCC_CFGR_HPRE_0 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_3);
    
      // HSE Clock Enable
  //RCC->CR = RCC_CR_HSEON;
}

void TIM1_Configuration(void)
{
  // TIM1
  TIM1->PSC = PRESCALE - 1;
  TIM1->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;// | TIM_DIER_CC2IE;		// enable 2 CC interrupt channels
  TIM1->CCMR1 = 0;                     // chan 1 is output
  TIM1->CCMR2 = 0;
  TIM1->CR1 = TIM_CR1_CEN;             // enable timer
 
  NVIC_SetPriority (TIM1_CC_IRQn, NVIC_IPR0_PRI_1);
  NVIC->ISER[0] |= (1 << TIM1_CC_IRQn); // enable TIM1 int in NVIC
}

void TIM2_Configuration(void)
{
  
  // APB1 Prescaler /2 - 100: HCLK divided by 2
  //RCC->CFGR &= ~(RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1);
  //RCC->CFGR |= RCC_CFGR_PPRE1_2;

  TIM2->PSC = PRESCALE - 1;
  TIM2->DIER |= TIM_DIER_CC1IE;// | TIM_DIER_CC2IE;		// enable 2 CC interrupt channels
  TIM2->CCMR1 = 0;                     // chan 1 is output
  TIM2->CR1 = TIM_CR1_CEN;             // enable timer

  NVIC_SetPriority (TIM2_IRQn, NVIC_IPR0_PRI_2);
  NVIC->ISER[0] |= (1 << TIM2_IRQn); // enable TIM2 int in NVIC
}

void TIM3_Configuration(void)
{

  TIM3->PSC = PRESCALE - 1;
  TIM3->DIER |= TIM_DIER_CC1IE;		// enable 2 CC interrupt channels
  TIM3->CCMR1 = 0;                     // chan 1 is output
  TIM3->CR1 = TIM_CR1_CEN;             // enable timer

  NVIC_SetPriority (TIM3_IRQn, NVIC_IPR0_PRI_3);
  NVIC->ISER[0] |= (1 << TIM3_IRQn); // enable TIM2 int in NVIC
}

void TIM4_Configuration(void)
{
  TIM4->PSC = PRESCALE - 1;
  TIM4->DIER |= TIM_DIER_CC1IE;		// enable 2 CC interrupt channels
  TIM4->CCMR1 = 0;                     // chan 1 is output
  TIM4->CR1 = TIM_CR1_CEN;             // enable timer
  
  NVIC_SetPriority (TIM4_IRQn, NVIC_IPR1_PRI_4);
  NVIC->ISER[0] |= (1 << TIM4_IRQn); // enable TIM2 int in NVIC
}

void RCC_Configuration(void){
   
  // APB1 Prescaler /2 - 100: HCLK divided by 2
  //RCC->CFGR &= ~(RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1);
  //RCC->CFGR |= RCC_CFGR_PPRE1_2;
  
  // APB2 Prescaler HCLK not divided
  //RCC->CFGR |= (RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE1_2);
  
  //RCC->CFGR = 0;                       // HSI, 8 MHz, 

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // enable PORTA 
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// enable PORTB
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // enable Timer1
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  
  //RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;  // enable Timer2

  // Enable GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
}

void Config(void)
{
  RCC_DeInit();
  
  RCC_HSEConfig(RCC_HSE_ON);
  
  ErrorStatus HSEStartUpStatus;
  
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  
  if (HSEStartUpStatus = SUCCESS)
  {
    
     /* Enable Prefetch Buffer */
    //FLASH->ACR |= FLASH_ACR_PRFTBE;
 
    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
 
    //FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    
    //FLASH_SetLatency(FLASH_Latency_2);
    
    RCC_PLLCmd(DISABLE);
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    
    RCC_PCLK2Config(RCC_HCLK_Div1);
    
    RCC_PCLK1Config(RCC_HCLK_Div2);
    
    // DIV2
    RCC->CFGR |= RCC_CFGR_PLLXTPRE;  
    // 0111: PLL input clock x 9
    
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
    //RCC->CFGR &= ~(RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_2 | RCC_CFGR_PLLMULL_3 | RCC_CFGR_PLLMULL_3);
    //RCC->CFGR |= (RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2);
    
    
    RCC_PLLCmd(ENABLE);
    
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    while(RCC_GetSYSCLKSource() != 0x08);
  }
  else
    for(;;);

}
  
int main(void)
{
  RCC_Configuration();
  
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);
  
  Config();
  //HSE_Configuration();

  //TIM_Config();
  TIM1_Configuration();
  TIM2_Configuration();
  
  //TIM_Config();
  
  // GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);

  // * Configure PB5 *
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  while (1)
  {
    if (flag1)
      // toggle green LED
      GPIOB->BRR  |= GPIO_BRR_BR5;
    else
      GPIOB->BSRR |= GPIO_BSRR_BS5;
    if (flag2)
      // toggle red LED
      GPIOB->BRR |= GPIO_BRR_BR4;
    else
      GPIOB->BSRR |= GPIO_BSRR_BS4;
    
    //logDebugInfo();
  }
}


/*
 * 
 * void TIM_Configuration(void)
{

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                         RCC_APB1Periph_TIM4, ENABLE);


  // /* Clear timers and update pending flags *
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);

  // /* Enable TIM2, TIM3 and TIM4 Update interrupts *
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

  // /* TIM2, TIM3 and TIM4 enable counters *
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}


void NVIC_Configuration(void)
{

  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Configure two bits for preemption priority *
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Enable the TIM2 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

    /* Enable the TIM3 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM4 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM5 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM6 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM7 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the TIM8 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
  NVIC_Init(&NVIC_InitStructure);
}
*

void NVIC_Configuration(void)
{

  __disable_irq();

  NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);

//  NVIC_SetPriority (TIM1_IRQn, uint32_t priority);
  NVIC_SetPriority (TIM2_IRQn, NVIC_IPR0_PRI_0);
  NVIC_SetPriority (TIM3_IRQn, NVIC_IPR0_PRI_1);
  NVIC_SetPriority (TIM4_IRQn, NVIC_IPR0_PRI_2);
  //NVIC_SetPriority (TIM5_IRQn, NVIC_IPR0_PRI_3);
  //NVIC_SetPriority (TIM6_IRQn, NVIC_IPR1_PRI_4);
  //NVIC_SetPriority (TIM7_IRQn, NVIC_IPR1_PRI_5);
//  NVIC_SetPriority (TIM8_IRQn, uint32_t priority);

  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(TIM4_IRQn);
  //NVIC_EnableIRQ(TIM5_IRQn);
  //NVIC_EnableIRQ(TIM6_IRQn);
  //NVIC_EnableIRQ(TIM7_IRQn);

  __enable_irq();

}

*/

void TIM_Config(void)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  // /* Enable TIM2, TIM3 and TIM4 clocks *
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                         RCC_APB1Periph_TIM4, ENABLE);

  // /* TIM2 configuration *
  TIM_TimeBaseStructure.TIM_Period = 0x95F;
  TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock/1200) - 1);
  //TIM_TimeBaseStructure.TIM_Prescaler = PRESCALE - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);

  // /* Output Compare Timing Mode configuration: Channel1 *
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  //TIM_OCInitStructure.TIM_Pulse = 0x0;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);

  // /* TIM3 configuration *
  TIM_TimeBaseStructure.TIM_Period = 0x95F; 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  // /* Output Compare Timing Mode configuration: Channel1 *
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  // /* TIM4 configuration *
  TIM_TimeBaseStructure.TIM_Period = 0xE0F;  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  // /* Output Compare Timing Mode configuration: Channel1 *
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  // /* Immediate load of TIM2,TIM3 and TIM4 Precaler values *
  TIM_PrescalerConfig(TIM2, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);
  TIM_PrescalerConfig(TIM3, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);
  TIM_PrescalerConfig(TIM4, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);

  // /* Clear TIM2, TIM3 and TIM4 update pending flags *
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);

  // /* Configure two bits for preemption priority *
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // /* Enable the TIM2 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  // /* Enable the TIM3 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  // /* Enable the TIM4 Interrupt *
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_Init(&NVIC_InitStructure);

  // /* Enable TIM2, TIM3 and TIM4 Update interrupts *
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

  // /* TIM2, TIM3 and TIM4 enable counters *
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}

// */
