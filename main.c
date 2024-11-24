
#include "hal.c"


// main function
// turns on 
int main(void) {
    
    SysTick_init(16000000 / 1000);                     // intialize the SysTick to count every 1ms | 1khz
    
    button_mode = 1;
    uint16_t usr_pb = PIN('C', 13);                    // pin of user pb on nucleo board
    io_port_en(PINBANK(usr_pb), true);                 // enable this IO port (port c)
    gpio_set_mode(usr_pb, GPI_MODE);                   // set pin to input
    gpio_interrupt_init(usr_pb);                       // set this pin to interrupt on rising edge

    uint16_t usr_led = PIN('A', 5);                    // pin for user accessed LED (should be green)
    io_port_en(PINBANK(usr_led), true);                // Enable I/O port for this bank
    gpio_set_mode(usr_led, GPO_MODE);                  // set pin for user accessed LED to output mode
    
    uint32_t timer;                                    // create timer                 
    while(1) {
      
      uint32_t period = button_mode*button_mode * 100;             // set the period to rely on the button mode
      if (timer_expired(&timer, period, s_ticks)) {
        static bool on;
        gpio_write(usr_led, on);
        on = !on;
      }
      
    }                                
    return 0;
}

//_____________________________________________________________________________________________//


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
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler,
     0, 0, 0, 0, 0, 0, 0, EXTI4_15_Handler};
