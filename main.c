/*********************************
 * stm32 timer led example main.c
 ********************************/

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define LED_PIN    6

/* User defined function prototypes */
void GPIOA_Init(void);
void TIM2_CC1_Init(void);
void led_toggle(void);

/* Prototypes for func. from stm32f10x lib. */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);

int main(void)
{
    /* Initialize GPIOA LED PIN */
    GPIOA_Init();

    /* Initialize TIM2 Capture/Compare 1 in output compare mode */
    TIM2_CC1_Init();

    /* Toggle LED forever */
    while(1)
    {
        /* Do nothing, all happens in ISR */
    }
}

/* Initialize GPIOA LED PIN */
void GPIOA_Init(void)
{
    /* Configuration info. for PORTA LED PIN:
     * - Speed = 50MHz
     * - Push-pull output mode
     */
    GPIO_InitTypeDef gpioa_led_pin_config = { LED_PIN,
                                              GPIO_Speed_50MHz,
                                              GPIO_Mode_Out_PP };

    /* Enable PORT A clock */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    /* Configure PORTA LED PIN  */
    GPIO_Init(GPIOA, &gpioa_led_pin_config);

    /* Turn off LED to start with */
    GPIOA->BSRR = (uint32_t)1 << LED_PIN;
}

/* Toggle LED */
void led_toggle(void)
{
    /* If PORTA LED BIT clear, set it */
    if((GPIOA->ODR & (uint32_t)1<< LED_PIN) == 0)
    {
        GPIOA->BSRR = (uint32_t)1 << LED_PIN;
    }
    /* If PORTA LED BIT set, clear it */
    else
    {
        GPIOA->BRR = (uint32_t)1 << LED_PIN;
    }
}

/* Configure TIM2 Capture/Compare 1 to work in output compare mode
 * so that the red LED flashes at 1HZ (toggle every 0.5s) */
void TIM2_CC1_Init(void)
{
    /* Enable TIM2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    /* Enable TIM2 global interrupt */
    NVIC->ISER[0] |= 0x10000000;

    /* Frequency after prescalar = 72MHz/(7199+1) = 10KHz.
     * Compare register = 5000 so a compare match occurs every 0.5s.
     */
    TIM2->PSC = (uint16_t)7199;
    TIM2->CCR1 = (uint16_t)5000;

    /* Enable Capture/Compare 1 interrupt */
    TIM2->DIER |= (uint16_t)0x0002;

    /* Enable TIM2 counter (in upcount mode) */
    TIM2->CR1 |= (uint16_t)0x0001;
}

/* Timer 2 Interrupt Service Routine */
void TIM2_IRQHandler(void)
{
    /* Toggle LED if TIM2's Capture/Compare 1 interrupt occurred. */
    if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        led_toggle();
        /* Clear TIM2's Capture/Compare 1 interrupt pending bit. */
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        /* Increment compare register by 5000 so next interrupt
         * occurs in 0.5s */
        TIM2->CCR1 += (uint16_t)5000;
    }

    /* ===== Other TIM2 interrupt types can go below ======
     * .........
     */
}

/* This function comes straight from stm32f10x_gpio.c. All it does is
 * configure a GPIO pin's speed and mode etc. but seems so complicated.
 */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{
  uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
  uint32_t tmpreg = 0x00, pinmask = 0x00;
  /* Check the parameters */
  /*
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
  assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));
  */

/*---------------------------- GPIO Mode Configuration -----------------------*/
  currentmode = ((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x0F);
  if ((((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x10)) != 0x00)
  {
    /* Check the parameters */
    /* assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed)); */
    /* Output mode */
    currentmode |= (uint32_t)GPIO_InitStruct->GPIO_Speed;
  }
/*---------------------------- GPIO CRL Configuration ------------------------*/
  /* Configure the eight low port pins */
  if (((uint32_t)GPIO_InitStruct->GPIO_Pin & ((uint32_t)0x00FF)) != 0x00)
  {
    tmpreg = GPIOx->CRL;
    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = ((uint32_t)0x01) << pinpos;
      /* Get the port pins position */
      currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;
      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding low control register bits */
        pinmask = ((uint32_t)0x0F) << pos;
        tmpreg &= ~pinmask;
        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);
        /* Reset the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
        {
          GPIOx->BRR = (((uint32_t)0x01) << pinpos);
        }
        else
        {
          /* Set the corresponding ODR bit */
          if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
          {
            GPIOx->BSRR = (((uint32_t)0x01) << pinpos);
          }
        }
      }
    }
    GPIOx->CRL = tmpreg;
  }
/*---------------------------- GPIO CRH Configuration ------------------------*/
  /* Configure the eight high port pins */
  if (GPIO_InitStruct->GPIO_Pin > 0x00FF)
  {
    tmpreg = GPIOx->CRH;
    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = (((uint32_t)0x01) << (pinpos + 0x08));
      /* Get the port pins position */
      currentpin = ((GPIO_InitStruct->GPIO_Pin) & pos);
      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding high control register bits */
        pinmask = ((uint32_t)0x0F) << pos;
        tmpreg &= ~pinmask;
        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);
        /* Reset the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
        {
          GPIOx->BRR = (((uint32_t)0x01) << (pinpos + 0x08));
        }
        /* Set the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
        {
          GPIOx->BSRR = (((uint32_t)0x01) << (pinpos + 0x08));
        }
      }
    }
    GPIOx->CRH = tmpreg;
  }
}

/* This function comes straight from stm32f10x_tim.c. It checks
 * whether interrupt TIM_IT in TIMx has occurred or not.
 */
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  ITStatus bitstatus = RESET;
  uint16_t itstatus = 0x0, itenable = 0x0;
  /* Check the parameters */
  //assert_param(IS_TIM_ALL_PERIPH(TIMx));
  //assert_param(IS_TIM_GET_IT(TIM_IT));

  itstatus = TIMx->SR & TIM_IT;

  itenable = TIMx->DIER & TIM_IT;
  if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/* This function comes straight from stm32f10x_tim.c.
 * It clears the TIMx's TIM_IT interrupt pending bits.
 */
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  /* Check the parameters */
  //assert_param(IS_TIM_ALL_PERIPH(TIMx));
  //assert_param(IS_TIM_IT(TIM_IT));
  /* Clear the IT pending Bit */
  TIMx->SR = (uint16_t)~TIM_IT;
}
