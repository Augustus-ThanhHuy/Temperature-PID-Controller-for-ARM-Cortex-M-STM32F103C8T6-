# **Temperature PID Controller for ARM Cortex M (STM32F103C8T6))**
## Introduction
This is project about the STM32F103C8T6 microcontroller
It has all the functions of a thermostat including:
* Use PID algorithm to adjust temperature (heat emitted from incandescent light bulbs)
* Use the push button to increase or decrease the temperature (using External interrupt)
* LCD screen to display temperature
* Collect temperature using DHT22 sensor
## Knowledge used in the project
* uses HAL library and STM32CubeMX software
* Use timer1 to create a delayus function that communicates with DHT22
* Use timer2 to hash the PWM pulse to adjust the brightness of the heat-generating bulb
* use timer3 as update time in PID controller
* Uses I2C communication standard to communicate with LCD screen
* Use an external interrupt as an input to control the temperature increase or decrease for the push button
## Result
Image displayed on LCD when temperature reaches setpoint and fluctuates stably with error 0.1 - 0.2℃ (The setpoint in the project is set to 37℃)
![nhietdo](https://github.com/Augustus-ThanhHuy/Temperature-PID-Controller-for-ARM-Cortex-M-STM32F103C8T6-/assets/109091737/0f2cf591-81e7-407a-ae23-a240375e9bcb)
