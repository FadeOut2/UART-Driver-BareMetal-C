#include "stm32f767xx.h"
#include <stdint.h>

//===MACROS FOR USART3 PERI===//
#define USART3_PCLK 48000000UL
#define USART3_BAUDRATE 115200UL
#define PD8_TX 8
#define PD9 9

//===Prototype Declrations===//

static void USART3_init(void);
static void GPIOD_init(void);

static inline void delay(volatile uint32_t t);
static inline void sendChar(char c);
static inline void sendString(char* str);

int main(void){
    GPIOD_init();
    USART3_init();
    for(;;){
        sendString("I mastered UART communication! basic's\r\n");
        delay(1000000);
    }
    while(1);
    return 0; // it should never reach here
}

static void GPIOD_init(){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    __DSB(); (void)RCC->AHB1ENR;

    GPIOD->MODER   = (GPIOD->MODER   & ~(3u << (2*PD8_TX))) | (2u << (2*PD8_TX));
    GPIOD->OTYPER &= ~(1u << PD8_TX);
    GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~(3u << (2*PD8_TX))) | (2u << (2*PD8_TX)); // high
    GPIOD->PUPDR  &= ~(3u << (2*PD8_TX));
    GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(0xFu << ((PD8_TX-8)*4))) | (7u << ((PD8_TX-8)*4)); // AF7

}

static void USART3_init(){
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
   __DSB(); (void)RCC->APB1ENR; 
   RCC->DCKCFGR2 &= ~(3U << 6); // USART3SEL = 00 ->PCLK1
    __DSB(); (void)RCC->DCKCFGR2; // per ref manual errata
    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->BRR = (0X0341);
    USART3->CR1 |= USART_CR1_TE;
    USART3->CR1 |= USART_CR1_UE;
    __DSB();
    while(!(USART3->ISR & USART_ISR_TEACK)){}
}

static inline void sendChar(char c){
    while(!(USART3->ISR & USART_ISR_TXE)); // wait until TXE is set
    USART3->TDR = (uint8_t)c & 0xFF;
}

static inline void sendString(char* str){
    while(*str){
        while(!(USART3->ISR & USART_ISR_TXE)); // wait until TXE is set
        USART3->TDR = (uint8_t)(*str++) & 0xFF;
    }
}
static inline void delay(volatile uint32_t t){
    for(; t>0; t--);
   __NOP();
}

void _init(void){}