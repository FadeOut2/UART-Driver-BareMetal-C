// main.c — STM32F767ZI, bare-metal USART3 on PD8/PD9 (AF7), 115200 8N1, APB1=48 MHz
#include "stm32f767xx.h"
#include <stdint.h>

#define PD8_TX   8U   // USART3_TX
#define PD9_RX   9U   // USART3_RX

static inline void delay(volatile uint32_t t){ while(t--){ __NOP(); } }

// /* ---------------- System Clock Config to get 48MHz PCLK1 ---------------- */
// static void SystemClock_Config_48MHz_PCLK1(void){
//     // This configures: SYSCLK=96MHz, AHB=96MHz, APB1=48MHz, APB2=96MHz
    
//     // 1. Enable HSE (8MHz external oscillator)
//     RCC->CR |= RCC_CR_HSEON;
//     while(!(RCC->CR & RCC_CR_HSERDY));
    
//     // 2. Configure Flash latency for 96MHz
//     FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_3WS;
//     FLASH->ACR |= FLASH_ACR_ARTEN | FLASH_ACR_PRFTEN;
    
//     // 3. Configure PLL: HSE(8MHz) / PLLM(4) * PLLN(96) / PLLP(2) = 96MHz
//     RCC->PLLCFGR = 0;
//     RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;        // HSE as PLL source
//     RCC->PLLCFGR |= (4 << RCC_PLLCFGR_PLLM_Pos);   // PLLM = 4
//     RCC->PLLCFGR |= (96 << RCC_PLLCFGR_PLLN_Pos);  // PLLN = 96  
//     RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLP_Pos);   // PLLP = 2 (00 = /2)
//     RCC->PLLCFGR |= (4 << RCC_PLLCFGR_PLLQ_Pos);   // PLLQ = 4
    
//     // 4. Enable PLL and wait
//     RCC->CR |= RCC_CR_PLLON;
//     while(!(RCC->CR & RCC_CR_PLLRDY));
    
//     // 5. Configure bus prescalers BEFORE switching to PLL
//     RCC->CFGR |= RCC_CFGR_HPRE_DIV1;    // AHB = SYSCLK/1 = 96MHz
//     RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // APB1 = AHB/2 = 48MHz 
//     RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;   // APB2 = AHB/1 = 96MHz
    
//     // 6. Switch system clock to PLL
//     RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
//     while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
//     // 7. Update SystemCoreClock variable
//     SystemCoreClockUpdate(); // Now SystemCoreClock = 96MHz
// }


/* ---------------- GPIO init ---------------- */
static void GPIOD_init_USART3(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    __DSB(); (void)RCC->AHB1ENR;

    // PD8/PD9 -> Alternate Function mode (10b)
    GPIOD->MODER   = (GPIOD->MODER   & ~(3U<<(2*PD8_TX))) | (2U<<(2*PD8_TX));
    GPIOD->MODER   = (GPIOD->MODER   & ~(3U<<(2*PD9_RX))) | (2U<<(2*PD9_RX));

    // AF7 for USART3
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(0xFU<<((PD8_TX-8)*4))) | (7U<<((PD8_TX-8)*4));
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(0xFU<<((PD9_RX-8)*4))) | (7U<<((PD9_RX-8)*4));
}

/* ---------------- USART3 init ---------------- */
static void USART3_init_115200_sysclk(void){

    // Enable USART3 peripheral clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    __DSB(); (void)RCC->APB1ENR;

    // USART3 Clock Source Selection (DCKCFGR2 bits 5:4)
    // 00: PCLK1 (APB1 clock) - Default, depends on your system clock config
    // 01: SYSCLK (System clock) - Same as CPU clock
    // 10: HSI (16MHz internal oscillator) 
    // 11: LSE (32.768kHz external oscillator)
    
    // Method 1: Use PCLK1 (you need to configure system clocks first)
   // RCC->DCKCFGR2 &= ~(3U << 4);           // Clear bits 5:4
   // RCC->DCKCFGR2 |= (0U << 4);            // USART3SEL = 00 -> PCLK1
    
     // Method 2: Use SYSCLK (system clock)
     //RCC->DCKCFGR2 &= ~(3U << 4);        // Clear bits 5:4  
    // RCC->DCKCFGR2 |= (1U << 4);         // USART3SEL = 01 -> SYSCLK
    
   //  Method 3: Use HSI (16MHz - always available)
     RCC->DCKCFGR2 &= ~(3U << 4);        // Clear bits 5:4
    RCC->DCKCFGR2 |= (2U << 4);         // USART3SEL = 10 -> HSI
    
    // Method 4: Use LSE (32.768kHz - if available)
    // RCC->DCKCFGR2 &= ~(3U << 4);        
    // RCC->DCKCFGR2 |= (3U << 4);         // USART3SEL = 11 -> LSE
    __DSB(); (void)RCC->DCKCFGR2;

    USART3->CR1 = 0; USART3->CR2 = 0; USART3->CR3 = 0;
    
    // BRR calculation depends on selected clock source:
    uint32_t usart3_clock = 16000000;
    
    // For PCLK1: Need to calculate based on your system clock setup
    // If SYSCLK=96MHz and APB1_PRESCALER=/2, then PCLK1=48MHz
    
    
    // For HSI: Always 16MHz
    // usart3_clock = 16000000;
    
    // For SYSCLK: Same as system clock
    // SystemCoreClockUpdate();
    // usart3_clock = SystemCoreClock;
    
    USART3->BRR = usart3_clock / 115200;

    USART3->CR1 |= USART_CR1_TE;     // TX only
    USART3->CR1 |= USART_CR1_UE;     // enable
    while(!(USART3->ISR & USART_ISR_TEACK)) {} // wait for TEACK
}

/* ---------------- main ---------------- */
int main(void){
    // Option A: Use 48MHz PCLK1 (recommended)
    //SystemClock_Config_48MHz_PCLK1();   // SYSCLK=96MHz, PCLK1=48MHz
    
    // Option B: Use default 16MHz HSI (simple but slower)
    // SystemCoreClockUpdate();  // Keep default HSI=16MHz
    
    GPIOD_init_USART3();
    USART3_init_115200_sysclk();

    delay(500000);  // wait for terminal to be ready

    const char* msg = "USART3 at 16MHz HSI - 115200 baud!\r\n";
    
    while(1){
        const char* p = msg;
        while(*p){
            delay(10000);  // short delay between chars
            while((USART3->ISR & USART_ISR_TXE)==0) {}  // wait for TXE
            USART3->TDR = *p++;
            delay(100);  // short delay between chars
        }
        delay(5000000);  // wait before next message
    }

    for(;;){ delay(1000000); }  // idle forever
}

void _init(){}


/*
A simple UART Driver that lets you send strings over USART3 using PD8 as TX and PD9 as RX.
It is configured for 115200 baud, 8 data bits, no parity, 1 stop bit (115200 8N1).
The APB1 peripheral clock is set to 48 MHz.
and the system core clock is at 16 mhz or the rest of the system clock configurations.
here you have the option of choosing the clock source for the USART3 peripheral.
but the HSI is faster to set up and it ALWAYS works eventhough its not that precise.
but still works fine for 115200 baud.
bare-metal, no HAL, no stdlib.
*/
