#include <stdint.h>

void reg_gpio_pin_led(uint32_t *p_port_d_mode_reg, uint32_t *p_port_d_output_reg, int led_pin) {
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
    uint32_t *p_clock_ctrl_bus_reg = (uint32_t *) 0x40023830;

    // GPIO port mode register (GPIOx_MODER)
    // 0x40020C00 + 0x00 ==> 0x40020C00
    uint32_t *p_port_d_mode_reg = (uint32_t *) 0x40020C00;

    // GPIO port output data register (GPIOx_ODR)
    // 0x40020C00 + 0x14 ==> 0x40020C14
    uint32_t *p_port_d_output_reg = (uint32_t *) 0x40020C14;

    // Enable the clock for GPIOD peripheral in AHB1ENR (set the 3rd bit position)
    *p_clock_ctrl_bus_reg |= (1 << 3);

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 12);

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 13);

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 14);

    reg_gpio_pin_led(p_port_d_mode_reg, p_port_d_output_reg, 15);

    return 0;
}