#include <stdint.h>
#include "main.h"

void reg_gpio_pin_led(GPIOx_MODE_t volatile *p_port_d_mode_reg, GPIOx_ODR_t volatile *p_port_d_output_reg, int led_pin) {
    switch (led_pin) {
        case 12:
            // Configure the mode of the IO pin as output
            // Make n-th bit position as 1 for "General purpose output mode"
            p_port_d_mode_reg->pin_12 = 1;

            // Set led_pin-th bit of the output data register to make I/O pin-led_pin as HIGH
            p_port_d_output_reg->pin_12 = 1;
            break;
        case 13:
            p_port_d_mode_reg->pin_13 = 1;

            p_port_d_output_reg->pin_13 = 1;
            break;
        case 14:
            p_port_d_mode_reg->pin_14 = 1;

            p_port_d_output_reg->pin_14 = 1;
            break;
        case 15:
            p_port_d_mode_reg->pin_15 = 1;

            p_port_d_output_reg->pin_15 = 1;
            break;
    }
}

int main(void) {
    // RCC AHB1 peripheral clock enable register (AHB1ENR)
    // 0x40023800 + 0x30 ==> 0x40023830
    RCC_AHB1ENR_t volatile *const p_clock_ctrl_bus_reg = (RCC_AHB1ENR_t *) 0x40023830;

    // GPIOD port mode register (GPIOx_MODER)
    // 0x40020C00 + 0x00 ==> 0x40020C00
    GPIOx_MODE_t volatile *const p_port_d_mode_reg = (GPIOx_MODE_t *) 0x40020C00;

    // GPIOD port output data register (GPIOx_ODR)
    // 0x40020C00 + 0x14 ==> 0x40020C14
    GPIOx_ODR_t volatile *const p_port_d_output_reg = (GPIOx_ODR_t *) 0x40020C14;

    // Enable the clock for GPIOD peripheral in AHB1ENR (set the 3rd bit position)
    p_clock_ctrl_bus_reg->gpiod_en = 1;

    // GPIOA port mode register (GPIOx_MODER) to test connecting between PA0 and VDD
    // 0x40020000 + 0x00 ==> 0x40020000
    GPIOx_MODE_t volatile *const p_port_a_mode_reg = (GPIOx_MODE_t *) 0x40020000;

    GPIOx_ODR_t const volatile *const p_port_a_input_reg = (GPIOx_ODR_t *) 0x40020010; // read-only pointer, do not change it

    // Enable the clock for GPIOA peripheral in AHB1ENR (set the 0 bit position)
    p_clock_ctrl_bus_reg->gpioa_en = 1;

    // Configure PA0 as input mode
    p_port_a_mode_reg->pin_0 = 0;

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 13);

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 15);

    uint8_t pa0_status;
    while (1) {
        // Read the pin status of the pin PA0 (GPIOA INPUT DATA REGISTER)
        pa0_status = (uint8_t) p_port_a_input_reg->pin_0;

        if (pa0_status) {
            reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 12);
            reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 14);
        } else {
            p_port_d_output_reg->pin_12 = 0;
            p_port_d_output_reg->pin_14 = 0;
        }
    }
    return 0;
}