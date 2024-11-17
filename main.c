#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 0x8) | (num))
#define PINNO(pin) (pin & 0xFF)
#define PINBANK(pin) (pin >> 0x8)

// Define GPIO Peripheral Registers
struct gpio {
    volatile uint32_t MODER, OTYPER, OSSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
};
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

static inline void spin(volatile uint32_t count) {
  while(count--) (void) 0;
}



// main function
// turns on 
int main(void) {
    uint16_t usr_led = PIN('A', 5);                    // pin for user accessed LED (should be green)
    io_port_en(PINBANK(usr_led), true);                // Enable I/O port for this bank
    gpio_set_mode(usr_led, GPO_MODE);                  // set pin for user accessed LED to output mode
                             
    for (;;) {
      gpio_write(usr_led, true);
      spin(999999);
      gpio_write(usr_led, false);
      spin(999999);
    }                                
    return 0;
}



// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 32 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 32])(void) = {
    _estack, _reset};
