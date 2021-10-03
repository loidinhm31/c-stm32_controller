#include <stdint.h>

int main(void) {
    // RCC AHB1 peripheral clock enable register (AHB1ENR)
    // 0x40023800 + 0x30 ==> 0x40023830
    uint32_t *pClockCtrlReg = (uint32_t *) 0x40023830;

    // GPIO port mode register (GPIOx_MODER)
    // 0x40020C00 + 0x00 ==> 0x40020C00
    uint32_t *pPortDModeReg = (uint32_t *) 0x40020C00;

    // GPIO port output data register (GPIOx_ODR)
    // 0x40020C00 + 0x14 ==> 0x40020C14
    uint32_t *pPortDOutputReg = (uint32_t *) 0x40020C14;

    // Enable the clock for GPIOD peripheral in AHB1ENR (set the 3rd bit position)
//    *pClockCtrlReg |= (0x08)
    *pClockCtrlReg |= (1 << 3);

    // Configure the mode of the IO pin as output
    // clear the 24th and 25 th bit position
//    *pPortDModeReg &= 0xFCFFFFFF;
    *pPortDModeReg &= ~(3 << 24);
    // make 24th bit position as 1
//    *pPortDModeReg |= 0x1000000;
    *pPortDModeReg |= (1 << 24);

    // Set 12th bit of the output data register to make I/O pin-12 as HIGH
//    *pPortDOutputReg |= 0x1000;
    *pPortDOutputReg |= (1 << 12);

    return 0;
}