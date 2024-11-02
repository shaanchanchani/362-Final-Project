/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 7, 2024
  * @brief   ECE 362 Lab 7 student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username!  Even though we're not using an autotest, 
// it should be a habit to fill out your username in this field now.
const char* username = "schancha";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
#include "lcd.h"

void internal_clock();
void init_spi1_slow(void);
void sdcard_io_high_speed(void);
void init_lcd_spi(void);
uint8_t spi_transfer(uint8_t data);

// Uncomment only one of the following to test each step
// #define STEP1
// #define STEP2
// #define STEP3
// #define STEP4

#define LCD_SETUP
int __io_putchar(int c) {
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5 -> TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}
void init_usart5() {
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC -> AHBENR |= RCC_AHBENR_GPIODEN;

    GPIOC->MODER &= ~GPIO_MODER_MODER12_Msk;
    GPIOC->MODER |= GPIO_MODER_MODER12_1;
    GPIOC->AFR[1] &= ~GPIO_AFRH_AFSEL12_Msk;
    GPIOC->AFR[1] |= 2 << GPIO_AFRH_AFSEL12_Pos;

    GPIOD->MODER &= ~GPIO_MODER_MODER2_Msk;
    GPIOD->MODER |= GPIO_MODER_MODER2_1;
    GPIOD->AFR[0] &= ~GPIO_AFRL_AFSEL2_Msk;
    GPIOD->AFR[0] |= 2 << GPIO_AFRL_AFSEL2_Pos;

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    USART5->CR1 &= ~USART_CR1_UE;
    USART5->CR1 &= ~USART_CR1_M;
    USART5->CR2 &= ~USART_CR2_STOP;

    USART5->CR1 &= ~USART_CR1_PCE;
    USART5->CR1 &= ~USART_CR1_OVER8;
    USART5->BRR = 48000000 / 115200;

    USART5 -> CR1 |= USART_CR1_TE;
    USART5 -> CR1 |= USART_CR1_RE;

    USART5->CR1 |= USART_CR1_UE;

    while(!((USART5 -> ISR & USART_ISR_TEACK) && (USART5 -> ISR & USART_ISR_REACK))) { }
}

#ifdef STEP1
int main(void){
    internal_clock();
    init_usart5();
    for(;;) {
        while (!(USART5->ISR & USART_ISR_RXNE)) { }
        char c = USART5->RDR;
        while(!(USART5->ISR & USART_ISR_TXE)) { }
        USART5->TDR = c;
    }
}
#endif

#ifdef STEP2
#include <stdio.h>

// TODO Resolve the echo and carriage-return problem

int __io_putchar(int c) {
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5 -> TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    while (!(USART5->ISR & USART_ISR_RXNE));
    char c = USART5->RDR;
    if(c == '\r') {
        c = '\n';
    }
    __io_putchar(c);
    return c;
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP3
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
int __io_putchar(int c) {
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5 -> TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    return line_buffer_getchar();
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP4

#include <stdio.h>
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void enable_tty_interrupt(void) {
    USART5 -> CR1 |= USART_CR1_RXNEIE;
    NVIC -> ISER[0] |= 1 << USART3_8_IRQn;
    USART5 -> CR3 |= USART_CR3_DMAR;

    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;  

    DMA2_Channel2 -> CMAR = (uint32_t) &serfifo;
    DMA2_Channel2 -> CPAR = (uint32_t) &(USART5 -> RDR);
    DMA2_Channel2 -> CNDTR = FIFOSIZE;

    DMA2_Channel2 -> CCR &= ~DMA_CCR_DIR;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_HTIE;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_TCIE;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_MSIZE;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_PSIZE;
    
    DMA2_Channel2 -> CCR |= DMA_CCR_MINC;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_PINC;
    DMA2_Channel2 -> CCR |= DMA_CCR_CIRC;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_MEM2MEM;
    DMA2_Channel2 -> CCR |= DMA_CCR_PL_0 | DMA_CCR_PL_1;
    DMA2_Channel2->CCR |= DMA_CCR_EN;
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    while(fifo_newline(&input_fifo) == 0) {
        asm volatile ("wfi"); // wait for an interrupt
    }
    return fifo_remove(&input_fifo);
}

int __io_putchar(int c) {
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5 -> TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    return interrupt_getchar();
}

// TODO Copy the content for the USART5 ISR here
// TODO Remember to look up for the proper name of the ISR function
void USART3_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0); // These turn off buffering; more efficient, but makes it hard to explain why first 1023 characters not dispalyed
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: "); // Types name but shouldn't echo the characters; USE CTRL-J to finish
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n"); // After, will type TWO instead of ONE
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

// Add these functions before main or in a separate section
void init_spi1_slow(void) {
    // Enable GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    
    // Configure PB3 (SCK), PB4 (MISO), and PB5 (MOSI) for alternate function
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
    
    // Set alternate function to AF0 (SPI1) for PB3, PB4, PB5
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5);
    
    // Enable SPI1 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    // Disable SPI1 before configuring
    SPI1->CR1 &= ~SPI_CR1_SPE;
    
    // Configure SPI1
    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    SPI1->CR1 |= SPI_CR1_MSTR;    
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 |= SPI_CR2_FRXTH;
    SPI1->CR2 = (SPI1->CR2 & ~SPI_CR2_DS) | (0x7 << 8);
    
    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void sdcard_io_high_speed(void) {
    // Disable SPI1
    SPI1->CR1 &= ~SPI_CR1_SPE;
    
    // Clear BR bits and set for 12MHz
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= SPI_CR1_BR_1;
    
    // Re-enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void init_lcd_spi(void) {
    // Enable GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    
    // Configure PB8, PB11, and PB14 as outputs
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);
    GPIOB->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0;
    
    // Initialize SPI1 at slow speed
    init_spi1_slow();
    
    // Switch to high speed operation
    sdcard_io_high_speed();
}

uint8_t spi_transfer(uint8_t data) {
    while(!(SPI1->SR & SPI_SR_TXE));
    *((uint8_t*)&SPI1->DR) = data;
    while(!(SPI1->SR & SPI_SR_RXNE));
    return *((uint8_t*)&SPI1->DR);
}

#ifdef LCD_SETUP
#define CELL_SIZE   30    // Size of each cell
#define BOARD_X     20    // Starting X position
#define BOARD_Y     40    // Starting Y position
#define ROWS        6     // Standard Connect Four has 6 rows
#define COLS        7     // Standard Connect Four has 7 columns

void draw_connect_four_board(void) {
    // Draw blue background rectangle
    // drawfillrect(BOARD_X, BOARD_Y, COLS * CELL_SIZE, ROWS * CELL_SIZE, 0x001F);  // Blue color
    
    // // Draw the grid holes (black circles represented as filled rectangles with small size)
    // for(int row = 0; row < ROWS; row++) {
    //     for(int col = 0; col < COLS; col++) {
    //         // Calculate center position for each cell
    //         int x = BOARD_X + (col * CELL_SIZE) + 5;
    //         int y = BOARD_Y + (row * CELL_SIZE) + 5;
            
    //         // Draw black circle (represented as smaller filled rectangle)
    //         drawfillrect(x, y, CELL_SIZE - 10, CELL_SIZE - 10, 0x0000);  // Black color
    //     }
    // }
    
    
    // Draw title above the board
    // char title[] = "CONNECT FOUR";
    // drawtext(BOARD_X + 30, BOARD_Y - 30, title, 0xFFFF);  // White color
}

int main(void) {
    internal_clock();
    init_usart5();
    init_lcd_spi();
    
    // Enable line buffering for stdio
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    
    printf("This is the STM32 command shell.\n");
    printf("Type 'lcd_init' before trying any draw commands.\n");
    
    char cmd[80];
    
    // Main loop
    for(;;) {
        printf("> ");
        if (fgets(cmd, 80, stdin) != NULL) {
            // Remove newline
            cmd[strcspn(cmd, "\n")] = 0;
            
            // Parse command
            if (strcmp(cmd, "lcd_init") == 0) {
                lcd_init();
            }
            else if (strncmp(cmd, "clear ", 6) == 0) {
                uint16_t color;
                if (sscanf(cmd, "clear %hx", &color) == 1) {
                    clear(color);
                }
            }
            else if (strncmp(cmd, "drawfillrect ", 12) == 0) {
                int x1, y1, x2, y2;
                uint16_t color;
                if (sscanf(cmd, "drawfillrect %d %d %d %d %hx", &x1, &y1, &x2, &y2, &color) == 5) {
                    drawfillrect(x1, y1, x2, y2, color);
                }
            }
            else if (strncmp(cmd, "drawline ", 9) == 0) {
                int x1, y1, x2, y2;
                uint16_t color;
                if (sscanf(cmd, "drawline %d %d %d %d %hx", &x1, &y1, &x2, &y2, &color) == 5) {
                    drawline(x1, y1, x2, y2, color);
                }
            }
        }
    }
}


// int main(void) {
//     internal_clock();
//     init_usart5();
//     init_lcd_spi();
    
//     // Enable line buffering for stdio
//     setbuf(stdin,0);
//     setbuf(stdout,0);
//     setbuf(stderr,0);
    
//     printf("LCD initialized. Ready for commands.\n");
    
//     // Main loop
//     for(;;) {
//         // Wait for commands through USART
//         // The actual lcd_init, clear, drawfillrect, and drawline commands
//         // will be handled by the provided library functions
//     }
// }
#endif