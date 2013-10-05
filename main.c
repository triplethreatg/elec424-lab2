// HSE 72 MHz internal clock
 
#include "stm32f10x_conf.h"

void TIM1_CC_IRQHandler(void)
{
  if (TIM1->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    detectEmergency();
    //TIM1->CCR1 += TCLKS1;        // next interrupt time
    TIM1->SR &= ~TIM_SR_CC1IF;   // clear CC1 int flag (write 0)
  }
  if (TIM1->SR & TIM_SR_CC2IF)  // CC1 match?
  {
    /*
    TIM1->CCR2 += TCLKS1;        // next interrupt time
    TIM1->SR &= ~TIM_SR_CC2IF;   // clear CC1 int flag (write 0)
    */
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_CC1IF){
    refreshSensorData();

    TIM2->SR &= ~TIM_SR_CC1IF;
    //TIM2->CCR1 += TCLKS1;
  }
}

void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_CC1IF){
    calculateOrientation();

    //TIM3->CCR1 += TCLKS1;        // next interrupt time   
    TIM3->SR &= ~TIM_SR_CC1IF;
  }
}

void TIM4_IRQHandler(void)
{
  if (TIM4->SR & TIM_SR_CC1IF){
    updatePid();
    
    TIM4->SR &= ~TIM_SR_CC1IF;
  }
}

void SysTick_Configuration(void){
   
  // Clock source selection (AHB/8)
  SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE;
  
  SysTick->LOAD = 2250000;
  
  // SysTick exception request enable
  // SysTick->CTRL |= SysTick_CTRL_TICKINT;
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT;
  
  // Counter enable
  SysTick->CTRL |= SysTick_CTRL_ENABLE;
  
}

void TIM1_Configuration(void)
{
  // TIM1
  TIM1->PSC = 35;
  TIM1->CR1 |= TIM_CR1_ARPE;
  TIM1->ARR = 55535;
  TIM1->DIER |= TIM_DIER_CC1IE;// | TIM_DIER_CC2IE;		// enable 2 CC interrupt channels
  TIM1->CCMR1 = 0;                     // chan 1 is output
  TIM1->CR1 = TIM_CR1_CEN;             // enable timer
 
  NVIC_SetPriority (TIM1_CC_IRQn, NVIC_IPR0_PRI_1);
  NVIC->ISER[0] |= (1 << TIM1_CC_IRQn); // enable TIM1 int in NVIC
}

void TIM2_Configuration(void)
{
  
  // APB1 Prescaler /2 - 100: HCLK divided by 2
  //RCC->CFGR &= ~(RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1);
  //RCC->CFGR |= RCC_CFGR_PPRE1_2;

  //TIM2->PSC = PRESCALE - 1;
  //
  // Auto-reload preload enabled
  TIM2->PSC = 71;
  TIM2->CR1 |= TIM_CR1_ARPE;
  TIM2->ARR = 15535;
  //
  TIM2->DIER |= TIM_DIER_CC1IE;// | TIM_DIER_CC2IE;		// enable 2 CC interrupt channels
  TIM2->CCMR1 = 0;                     // chan 1 is output
  TIM2->CR1 = TIM_CR1_CEN;             // enable timer

  NVIC_SetPriority (TIM2_IRQn, NVIC_IPR0_PRI_2);
  NVIC->ISER[0] |= (1 << TIM2_IRQn); // enable TIM2 int in NVIC
}

void TIM3_Configuration(void)
{

  TIM3->PSC = 71;
  TIM3->CR1 |= TIM_CR1_ARPE;
  TIM3->ARR = 15535;
  
  TIM3->DIER |= TIM_DIER_CC1IE;		// enable 2 CC interrupt channels
  TIM3->CCMR1 = 0;                     // chan 1 is output
  TIM3->CR1 = TIM_CR1_CEN;             // enable timer

  NVIC_SetPriority (TIM3_IRQn, NVIC_IPR0_PRI_3);
  NVIC->ISER[0] |= (1 << TIM3_IRQn); // enable TIM2 int in NVIC
}

void TIM4_Configuration(void)
{
  
  TIM4->PSC = 1023;
  TIM4->CR1 |= TIM_CR1_ARPE;
  TIM4->ARR = 30380;
  
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
    
    RCC_PCLK2Config(RCC_HCLK_Div2);
    
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
  SysTick_Configuration();

  TIM1_Configuration();
  TIM2_Configuration();
  TIM3_Configuration();
  TIM4_Configuration();

  // * Configure PB5 *
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  while (1)
  {

    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
    GPIOB->BRR  |= GPIO_BRR_BR5;

    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
    GPIOB->BSRR |= GPIO_BSRR_BS5;      
    GPIOB->ODR ^= GPIO_ODR_ODR4;

    logDebugInfo();
  }
}
