#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 0x8) | (num))
#define PINNO(pin) (pin & 0xFF)
#define PINBANK(pin) (pin >> 0x8)

/*_____________________________________________________________________________________________

                                GPIO INTERACTIONS

-----------------------------------------------------------------------------------------------*/

// Define GPIO Peripheral Registers
struct gpio {
    volatile uint32_t MODER, OTYPER, OSSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
};
// memory location and offset for gpio banks 
#define GPIO(bank) ((struct gpio *) (0x50000000 + 0x0400 * (bank)))

// Enumerate GPIO Mode values as per the data sheet 
enum {GPI_MODE, GPO_MODE, AF_MODE, ANALOG_MODE};

// function to set GPIO modes
static inline void gpio_set_mode(uint16_t pin, uint8_t gpio_mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));           // GPIO Bank
  int n = PINNO(pin);                               // Pin Number
  gpio->MODER &= ~(3U << (n * 2));                  // Clear existing setting
  gpio->MODER |= (gpio_mode & 3) << (n * 2);        // Set new mode
}

// function to write gpio logic level to ODR thru BSRR
static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));                     // select gpio bank
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);          // shift 1 left to pin number in reset side. if value is 1 shift left another 16 to set side
}

// function to get input value of a pin through IDR
bool gpio_read(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  bool pin_val = gpio->IDR & BIT(PINNO(pin));
  return pin_val;
}

/*_____________________________________________________________________________________________

                            Control and setup RCC

------------------------------------------------------------------------------------------------*/


// Define RCC (Reset and Clock Control) registers 
struct rcc {
    volatile uint32_t CR, ICSCR, CFGR, PLLCFGR, RESERVED0[2], CIER, CIFR, CICR, IOPRSTR,
        AHBRSTR, APBRSTR1, APBRSTR2, IOPENR, AHBENR, APBENR1, APBENR2, IOPSMENR, AHBSMENR, 
        APBSMENR1, APBSMENR2, CCIPR, CCIPR2, BDCR, CSR;
};
// memory location of RCC
#define RCC ((struct rcc *) 0x40021000)

// enum gpio ports
enum {GPIO_A, GPIO_B, GPIO_C, GPIO_D, GPIO_E, GPIO_F};

// function to enable or disable IO port clock
static inline void io_port_en(uint16_t bank, bool val) {
  RCC->IOPENR &= ~(1U << bank);           // clear existing setting
  RCC->IOPENR |= (val << bank);           // apply new setting
}

/*_____________________________________________________________________________________________

                        SysTick and Timing Functions

-----------------------------------------------------------------------------------------------*/


// create a structure for the systick register
struct stk {
  volatile uint32_t CSR, RVR, CVR, CALIB;
};
// define the location of the systick register
#define STK ((struct stk *) 0xE000E010)

// function to initiate the ARM systick timer PM0223 datasheet
static inline void SysTick_init(uint32_t ticks) {
  if ((ticks-1) > 0xFFFFFF) return;               // limited to 24 bit timer
  STK->RVR = ticks-1;                             // set the value to be loaded 
  STK->CVR = 0;                                   // reset the current value to 0
  STK->CSR = BIT(0) | BIT(1) | BIT(2);            // set the enable bit to 1
  RCC->APBENR2 |= BIT(0);                         // Enable the SYSCFG clock
}

// function to handle SysTick interupt
static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

void delay(unsigned ms) {
  uint32_t wait = s_ticks + ms;                   // time to wait 
  while(s_ticks < wait) (void) 0;                 // wait until its time    
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

/*_____________________________________________________________________________________________

                                INTERRUPTS

------------------------------------------------------------------------------------------------*/


// create a structure for the NVIC registers
struct nvic {
  volatile uint32_t ISER, ICER, ISPR, ICPR, IABR, IPR[8];   // there is also a priority register but it has a different size
};
#define NVIC ((struct nvic *) 0xE000E100)

// create a structure for the Extended Interrupt and event controller (EXTI)
struct exti {
  volatile uint32_t RTSR1, FTSR1, SWIER1, RPR1, FPR1, RESERVED[19], 
      EXTICR[4], RESERVED2[4], IMR1, EMR1, RESERVED3[2];
};
#define EXTI ((struct exti *) 0x40021800)



void gpio_interrupt_init(uint16_t pin) {
  int x, m;                                         // variables for selecting EXTICR x and m
  
  RCC->APBENR2 |= BIT(0);                           // Enable the SYSCFG clock              
  
  x = PINNO(pin)/4;                                 // is index of EXTICRx register, (x-1)
  m = PINNO(pin)%4;                                 // multiplier to shift into EXTICRx
  EXTI->RTSR1 |= BIT(PINNO(pin));                   // select line to be a rising edge trigger
  EXTI->EXTICR[x] |= (PINBANK(pin) << m*8);         // select pin port "pin" as interrupt source
  EXTI->IMR1 |= BIT(PINNO(pin));                    // unmask interrupt on selected line

  NVIC->ISER |= BIT(7);                             // enable EXTI line 4-15 interrupt
}


/*_____________________________________________________________________________________________

                        Hardware Delay (Doesnt depend on systick)

-----------------------------------------------------------------------------------------------*/

// define spin function for crude delay
static inline void spin(volatile uint32_t count) {
  while(count--) (void) 0;
}


/*_____________________________________________________________________________________________


                        Interrupt Service Routines (ISR)

-----------------------------------------------------------------------------------------------*/

static volatile uint32_t button_mode;                // volatile var to store button setting / speed multiplier
// function to handle interrupt on EXTI lines 4 to 15
void EXTI4_15_Handler(void) {

  spin(10000);
  
  button_mode++;                                    // increase the speed multiplier from 1 to 3                            
  if (button_mode > 3) {
    button_mode = 1;
  }
  EXTI->RPR1 |= BIT(13);                            // clear pending interupt bit
  NVIC->ICPR |= BIT(7);                             //clear interupt flag

}

//_____________________________________________________________________________________________//