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
#include <stdlib.h>
#include "fifo.h"
#include "tty.h"
#include "lcd.h"
#include "commands.h"


void internal_clock();
void init_spi1_slow(void);
void sdcard_io_high_speed(void);
void init_lcd_spi(void);
uint8_t spi_transfer(uint8_t data);

char ** allocateBoard() ; 
void freeBoard(char **) ; 
char placeChip(int * ,int, char) ; 
int * allocateRow() ; 
void checkWin(int ,int,int, char,int) ; 
void changeCoordinates(int*,int*,int);
void printBoard() ; 
char simulateGame(int * ) ; 
void nano_wait(int);


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
void init_spi1_slow(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);  // Alternate function mode

    // Set alternate function to SPI1 (AF0) for PB3, PB4, PB5
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
    GPIOB->AFR[0] |= (0x0 << GPIO_AFRL_AFSEL3_Pos) | (0x0 << GPIO_AFRL_AFSEL4_Pos) | (0x0 << GPIO_AFRL_AFSEL5_Pos); // AF0

    // Configure SPI1
    // Set baud rate to the lowest (max divisor)
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= (0x7 << SPI_CR1_BR_Pos); // Baud rate divisor set to max

    // Set to Master mode
    SPI1->CR1 |= SPI_CR1_MSTR;

    // Set data frame format to 8-bit
    SPI1->CR2 &= ~SPI_CR2_DS;
    SPI1->CR2 |= (0x7 << SPI_CR2_DS_Pos); // Data size 8-bit

    // Enable software slave management and set internal slave select
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;

    // Set FIFO reception threshold to 8-bit
    SPI1->CR2 |= SPI_CR2_FRXTH;

    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void enable_sdcard(void) {
    // Enable clock for GPIOB if not already enabled
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Set PB2 as output
    GPIOB->MODER &= ~GPIO_MODER_MODER2;
    GPIOB->MODER |= GPIO_MODER_MODER2_0; // Set PB2 to output mode

    // Set PB2 low to enable the SD card
    GPIOB->ODR &= ~(1 << 2);
}

void disable_sdcard(void) {
    // Ensure PB2 is configured as output (in case it wasn't set)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock if not already enabled
    GPIOB->MODER &= ~GPIO_MODER_MODER2;
    GPIOB->MODER |= GPIO_MODER_MODER2_0; // Set PB2 to output mode

    // Set PB2 high to disable the SD card
    GPIOB->ODR |= 1 << 2;
}

void init_sdcard_io(void) {
    // Call init_spi1_slow to initialize SPI1
    init_spi1_slow();

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB2 as output
    GPIOB->MODER &= ~GPIO_MODER_MODER2;
    GPIOB->MODER |= GPIO_MODER_MODER2_0; // Set PB2 to output mode

    // Disable the SD card by setting PB2 high
    disable_sdcard();
}

void sdcard_io_high_speed(void) {
    // Disable the SPI1 channel
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // Set the SPI1 baud rate to achieve a clock rate of 12 MHz
    // Assuming the APB2 clock is 48 MHz, we need a divisor of 4 to get 12 MHz (48 MHz / 4 = 12 MHz)
    SPI1->CR1 &= ~SPI_CR1_BR;        // Clear baud rate bits
    SPI1->CR1 |= (0x1 << SPI_CR1_BR_Pos); // Set baud rate divisor to 4

    // Re-enable the SPI1 channel
    SPI1->CR1 |= SPI_CR1_SPE;
}

uint8_t spi_transfer(uint8_t data) {
    while(!(SPI1->SR & SPI_SR_TXE));     // Wait until TX buffer empty
    *(uint8_t*)&(SPI1->DR) = data;       // Send data
    while(!(SPI1->SR & SPI_SR_RXNE));    // Wait until RX buffer not empty
    while(SPI1->SR & SPI_SR_BSY);        // Wait until SPI not busy
    return *(uint8_t*)&(SPI1->DR);       // Return received data
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
uint8_t spi_transfer_detailed(uint8_t data) {
    uint32_t timeout = 100000;  // Arbitrary timeout value

    // Wait for TXE with timeout
    while(!(SPI1->SR & SPI_SR_TXE)) {
        if(--timeout == 0) {
            printf("SPI TX timeout\n");
            return 0xFF;  // Error value
        }
    }

    // Send the data
    *(uint8_t*)&(SPI1->DR) = data;
    
    timeout = 100000;  // Reset timeout

    // Wait for RXNE with timeout
    while(!(SPI1->SR & SPI_SR_RXNE)) {
        if(--timeout == 0) {
            printf("SPI RX timeout\n");
            return 0xFF;  // Error value
        }
    }

    timeout = 100000;  // Reset timeout

    // Wait for BSY flag to clear with timeout
    while(SPI1->SR & SPI_SR_BSY) {
        if(--timeout == 0) {
            printf("SPI busy timeout\n");
            return 0xFF;  // Error value
        }
    }

    // Read received data
    uint8_t received = *(uint8_t*)&(SPI1->DR);
    
    // Check for overrun error
    if(SPI1->SR & SPI_SR_OVR) {
        printf("SPI overrun error\n");
        // Clear overrun flag by reading DR and SR
        (void)SPI1->DR;
        (void)SPI1->SR;
    }

    return received;
}

// Helper function to check if SPI is correctly configured
void check_spi_config(void) {
    printf("SPI Configuration Status:\n");
    
    // Check if SPI is enabled
    printf("SPI enabled: %s\n", (SPI1->CR1 & SPI_CR1_SPE) ? "Yes" : "No");
    
    // Check master/slave mode
    printf("Master mode: %s\n", (SPI1->CR1 & SPI_CR1_MSTR) ? "Yes" : "No");
    
    // Check baud rate
    uint32_t br = (SPI1->CR1 & SPI_CR1_BR_Msk) >> SPI_CR1_BR_Pos;
    printf("Baud rate divider: %d\n", (1 << (br + 1)));
    
    // Check data size
    uint32_t ds = (SPI1->CR2 & SPI_CR2_DS_Msk) >> SPI_CR2_DS_Pos;
    printf("Data size: %d bits\n", (int)(ds + 1));
    
    // Check if software slave management is enabled
    printf("Software slave management: %s\n", 
           (SPI1->CR1 & SPI_CR1_SSM) ? "Yes" : "No");
}

void test_lcd_setup(void) {
    // 1. Initialize basic SPI and GPIO
    init_lcd_spi();  // This sets up SPI1 and GPIO pins
    
    // 2. Initialize LCD
    LCD_Setup();
    
    // 3. Test sequence
    // Clear screen to blue
    LCD_Clear(0x001F);  // Blue in RGB565 format
    
    // Draw test pattern
    LCD_DrawFillRectangle(10, 10, 50, 50, 0xF800);  // Red square
    LCD_DrawLine(0, 0, 100, 100, 0x07E0);  // Green diagonal line
    
    // Print status
    printf("LCD Test Pattern:\n");
    printf("- Blue background\n");
    printf("- Red square at (10,10)\n");
    printf("- Green diagonal line\n");
}

//============================================================================
// constants
//============================================================================
// grid characteristics
static int GRIDHEIGHT = 4;     // grid height
static int GRIDWIDTH  = 4 ;     // grid witdh
static int CELLCOUNT = 16 ;     // count of cells
// board cell states
static char EMPTYCELL = '0' ;   // no player as marked    
static char P1CELL    = '1' ;   // P1 has marked
static char P2CELL    = '2' ;   // p2 has marked
// game states
static char NONE  = '0' ;       // game is currently runnnig 
static char P1WINS = '1' ;      // P1 won
static char P2WINS = '2' ;      // P2 won
static char TIE = '3' ;         // TIE (no winner and no cells)
// placing valid or invalid chips (might need to revise)
static const char VALID = 'v' ; // if the placing is valid or not
static const char INVALID ='i' ;

//============================================================================
// game state variables
//============================================================================
char GAMESTATE =  '0' ;          // current state
int CHIPSPLACED = 0 ;            // chips placed
char ** board ;                  // current game

//============================================================================
// game mechanics
//============================================================================
int * allocateRow(){
    // rowCount is a vector of the lowest unused space in
    // the grid
    int * rowCount = (int *)malloc(sizeof(int)* GRIDWIDTH) ; 
    for (int i = 0 ; i < GRIDWIDTH ; i++){
        rowCount[i] = GRIDHEIGHT - 1 ;
    }
    return rowCount ; 
}
void freeBoard(char ** arr){
    // release all the arrays inside grid
    for (int i = 0 ; i < GRIDHEIGHT ; i++)
        free(arr[i]) ; 
    free(arr);
}
char ** allocateBoard() { 
    // creates the 2D array represnting the grid, initialize
    // the board as empty by doing the value as EMPTYCELL
    char ** arr = (char**) malloc(sizeof(char*) * GRIDHEIGHT); 
    for (int i = 0 ; i < GRIDHEIGHT ; i++){
        arr[i] = (char *)malloc(sizeof(char) * GRIDWIDTH) ; 
    }
    for (int i = 0; i < GRIDHEIGHT; i++)
        for (int j = 0; j < GRIDWIDTH; j++)
            arr[i][j] = EMPTYCELL;
    return arr ;  
}
char placeChip(int * rowCount, int COL, char CHIP){
    // places a chip and checks if there is any winner

    // if you were not supposed to place the chip (game ended)
    if (GAMESTATE == P1WINS) return P1WINS ;            // P1 won
    else if (GAMESTATE == P2WINS) return P2WINS ;       // P2 won
    else if (GAMESTATE == TIE) return TIE ;             // no one won

    // if you are trying to place a chip outside the board 
    if (rowCount[COL] == -1 ) return INVALID;           // overfill at top
    else if ((COL >  (GRIDWIDTH - 1)) || (COL < 0)){    // invalid column
        return INVALID ;
    }

    // if program made it here, it can place the chip correctly
    board[rowCount[COL]][COL] = CHIP ;                  // update the board
    rowCount[COL] -= 1 ;                                // update the lowest empty space
    // starting here, program is getting ready for DFS,
    // start by making a visited array
    // check all possible directions (D,DL,L,UL,UR,R,DR)
    for (int i = 0; i < 7 ; i++){
    checkWin(rowCount[COL] + 1, COL, 1, CHIP, i);
    }
    CHIPSPLACED += 1 ;                                  // one more chip placed
    // if there is no winner yet and the board is full, tie
    if ((CHIPSPLACED == CELLCOUNT) && (GAMESTATE == NONE)){
        GAMESTATE = TIE ;
    }                       
    return VALID ;                                      // placing is valid
}

void checkWin(int i, int j , int d, char CHIP, int dir){
    // basically a DFS, if at any point we get to level 4,
    // that means that there are 4 connected
    // (i,j) are the 2D coordinates of the point
    // d is the depth (if 4 player won)
    // CHIP is who the player is (P1 or P2)
    // dir is which direction we are checking (7 possible directions)
    if (GAMESTATE != NONE) return ;                 // if someone already won
    if (d == 4){                                    // current player won!
        GAMESTATE = CHIP ; 
        return ;
    } 
    changeCoordinates(&i,&j,dir) ;                  // update (i,j) based on direction

    // make sure the new coordinate is in bounds
    if ((i < 0) || (i > (GRIDHEIGHT - 1)) || (j < 0) || (j > (GRIDWIDTH - 1))){
        return ;
    }
    if (board[i][j] == NONE) return ;             // make sure it has not been visited or empty
    if (board[i][j] != CHIP) return ;               // make sure is owned by correct player
    checkWin(i,j,d + 1,CHIP, dir) ;         // if here, it means player has connection in the direction
}

void changeCoordinates(int * i , int * j, int dir){
    // dir can be thought of a dictionary 
    // 0 -> Down
    // 1 -> Down left
    // 2 -> Left
    // 3 -> Up left
    // 4 -> Up right (Up is always empty)
    // 5 -> Right
    // 6 -> Down right
    switch (dir)
    {
    case 0:
        *i += 1 ;
        break;
    case 1:
        *i += 1 ;
        *j -= 1 ;
        break;
    case 2 :
        *j -= 1 ;
        break ;
    case 3 :
        *i -= 1 ;
        *j -= 1 ;
        break ;
    case 4 :
        *i -= 1 ;
        *j += 1 ;
        break ;
    case 5 :
        *j += 1 ;
        break ;
    case 6 :
        *i += 1 ;
        *j -= 1 ;
    default:
        break;
    }
}

void display_board() {
    // Clear display
    // Draw the grid


    for(int i = 0; i < GRIDHEIGHT; i++) {
        for(int j = 0; j < GRIDWIDTH; j++) {
            if (board[i][j] != NONE) {
                if (board[i][j] == '1') {
                    // draw a shape for P1
                } else {
                    // draw a shape for P2
                }
            }
        }
    }
}


uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];
void print(const char str[]);
void printfloat(float f);


//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOB->MODER |= 0x00155555;
    GPIOC->MODER |= 0x00005500;
    GPIOC->OTYPER |= 0x00f0;
    GPIOC->MODER &= ~0x000000ff;
    GPIOC->PUPDR |= 0x00000055;
}



uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//============================================================================
// The Timer 7 ISR
//============================================================================
void TIM7_IRQHandler(){
    TIM7->SR &= ~TIM_SR_UIF;
    int rows = read_rows();
    update_history(col,rows);
    col = (col + 1) & 3;
    drive_column(col);
}
//============================================================================
// init_tim7()
//============================================================================
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 480 - 1;
    TIM7->ARR = 100 - 1;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] |= 1 << TIM7_IRQn;
    TIM7->CR1 |= TIM_CR1_CEN;
}
void init_spi1() {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0x80008800;
    GPIOA->AFR[0] &= ~(0xf << 20);
    GPIOA->AFR[0] &= ~(0xf << 28);
    GPIOA->AFR[1] &= ~(0xf << 28);
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= SPI_CR1_BR;
    SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_3 | SPI_CR2_DS_0;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 |= SPI_CR1_SPE;
}
void spi_cmd(unsigned int data) {
    while((SPI1->SR & SPI_SR_TXE) == 0);
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    for(int i = 0; string[i] != '\0'; i++){
        spi_data(string[i]);
    }
}
void spi1_display2(const char *string) {
    spi_cmd(0xc0);
    for(int i = 0; string[i] != '\0'; i++){
        spi_data(string[i]);
    }
}


int main(void) {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);


    enable_ports();
    init_tim7();
    init_spi1();
    spi1_init_oled();


    command_shell();
    

    // board = allocateBoard() ;   // create board
    // int * rowCount = allocateRow() ;    // create rowCount


    // int i = 0;
    // char turn = P1CELL;
    // // init_tim7();
    // int c ;
    // char CHECK;
    // spi1_display1("Turn of : ") ; 
    // spi1_display2("Player 1");
    // for(;;){
    //     c = (int)get_keypress();
    //     c = c - 49;
        
    //     turn = (i % 2) ? P2CELL : P1CELL;
        
    //     if (turn == P2CELL) spi1_display2("Player 1    ");
    //     else spi1_display2("Player 2     ");

    //     CHECK = placeChip(rowCount,c,turn) ;
    //     if (CHECK == INVALID){
    //         spi1_display1("INVALID MOVE!") ; 
    //         spi1_display2("            ");
    //         nano_wait(1000000000);
    //         nano_wait(1000000000);
    //         nano_wait(1000000000); 
    //         spi1_display1("Turn of :     ") ; 
    //         if (turn == P2CELL) spi1_display2("Player 2    ");
    //         else spi1_display2("Player 1     ");
    //         i -= 1 ; 
    //     }else if (CHECK == P1WINS){
    //         spi1_display1("PLAYER 1 WON!!") ; 
    //         spi1_display2("            ");
    //          break ;
    //     }else if (CHECK == P2WINS){
    //         spi1_display1("PLAYER 2 WON!!") ; 
    //         spi1_display2("            ");
    //         break ;
    //     }else if (CHECK == TIE){
    //         spi1_display1("TIE...       ") ; 
    //         spi1_display2("            ");
    //         break ;
    //     }
    //     i = i + 1;
    // }
}