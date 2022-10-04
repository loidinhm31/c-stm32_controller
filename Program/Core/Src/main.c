#include <stdint.h>

void reg_gpio_pin_led(uint32_t volatile *p_port_d_mode_reg, uint32_t volatile *p_port_d_output_reg, int led_pin) {
    // Configure the mode of the IO pin as output
    // Clear the n-th and (n+1)-th bit position (for PD-led_pin)
    *p_port_d_mode_reg &= ~(3 << 2 * led_pin);
    // Then, make n-th bit position as 1 for "General purpose output mode"
    *p_port_d_mode_reg |= (1 << 2 * led_pin);

    // Set led_pin-th bit of the output data register to make I/O pin-led_pin as HIGH
    *p_port_d_output_reg |= (1 << led_pin);
}

int main(void) {
    // RCC AHB1 peripheral clock enable register (AHB1ENR)
    // 0x40023800 + 0x30 ==> 0x40023830
    uint32_t volatile *const p_clock_ctrl_bus_reg = (uint32_t *) 0x40023830;

    // GPIOD port mode register (GPIOx_MODER)
    // 0x40020C00 + 0x00 ==> 0x40020C00
    uint32_t volatile *const p_port_d_mode_reg = (uint32_t *) 0x40020C00;

    // GPIOD port output data register (GPIOx_ODR)
    // 0x40020C00 + 0x14 ==> 0x40020C14
    uint32_t volatile *const p_port_d_output_reg = (uint32_t *) 0x40020C14;

    // Enable the clock for GPIOD peripheral in AHB1ENR (set the 3rd bit position)
    *p_clock_ctrl_bus_reg |= (1 << 3);

    // GPIOA port mode register (GPIOx_MODER)
    // 0x40020000 + 0x00 ==> 0x40020000
    uint32_t volatile *const p_port_a_mode_reg = (uint32_t *) 0x40020000;

    uint32_t const volatile *const p_port_a_input_reg = (uint32_t *) 0x40020010; // read-only pointer, do not change it

    // Enable the clock for GPIOA peripheral in AHB1ENR (set the 0 bit position)
    *p_clock_ctrl_bus_reg |= (1 << 0);

    // Configure PA0 as input mode
    *p_port_a_mode_reg &= ~(3 << 0);

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 13);

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 15);

    uint8_t pa0_status;
    while (1) {
        // Read the pin status of the pin PA0 (GPIOA INPUT DATA REGISTER)
        pa0_status = (uint8_t) (*p_port_a_input_reg & 0x1); // zero out all other bits except bit 0 for PA0

        if (pa0_status) {
            reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 12);
            reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 14);
        } else {
            *p_port_d_output_reg &= ~(1 << 12);
            *p_port_d_output_reg &= ~(1 << 14);
        }
    }
    return 0;
}