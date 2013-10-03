// Default 8 MHz internal clock
 
#include <stm32f10x.h>
 
#define F_CPU           72000000   // 1MHz
#define PRESCALE        64
#define PERIOD1         250 // microseconds - 1/2 second delay
#define PERIOD2		 500 // microseconds - 1/4 second delay
#define TCLKS1          ((F_CPU/PRESCALE*PERIOD2)/1000)
 
volatile uint8_t flag1 = 1;
volatile uint8_t flag2 = 1;

void HSE_Configuration(void){
  // HSE Clock Enable
  RCC->CR |= RCC_CR_HSEON;
  
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
  
}
 
void TIM1_CC_IRQHandler(void)
{
  if (TIM1->SR & TIM_SR_CC1IF)  // CC1 match?
  {
    if(flag1)
      flag1 = 0;
    else
      flag1 = 1;

    TIM1->CCR1 += TCLKS1;        // next interrupt time
    TIM1->SR = ~TIM_SR_CC1IF;   // clear CC1 int flag (write 0)
  }

  
  if (TIM1->SR & TIM_SR_CC2IF)  // CC2 match?
  {
    if(flag2)
      flag2 = 0;
    else
      flag2 = 1;

    TIM1->CCR2 += TCLKS1 / 2;        // next interrupt time
    TIM1->SR = ~TIM_SR_CC2IF;   // clear CC1 int flag (write 0)
  }
  
}
 
int main(void)
{

  HSE_Configuration();

  RCC->CFGR = 0;                       // HSI, 8 MHz, 
 
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // enable PORTA 
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;	// enable PORTB

  // Enable GPIO
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST , ENABLE);
  
  // * Configure PB5 *
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // enable Timer1
 
  TIM1->PSC = PRESCALE - 1;
  TIM1->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;;// | TIM_DIER_CC2IE;		// enable 2 CC interrupt channels
  //TIM1->DIER |= ~TIM_DIER_CC2IE;
  TIM1->CCMR1 = 0;                     // chan 1 is output
  TIM1->CCMR2 = 0;
  TIM1->CR1 = TIM_CR1_CEN;             // enable timer
 
  NVIC->ISER[0] = (1 << TIM1_CC_IRQn); // enable TIM1 int in NVIC
 
  while (1)
  {
    if (flag1)
      // toggle green LED
      GPIOB->BRR = 0b001000000;
    else
      GPIOB->BSRR = 0b00100000;
    if (flag2)
      // toggle red LED
      GPIOB->BRR = 0b00010000;
    //else
      GPIOB->BSRR = 0b00010000;
  }
}
