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

<div style="display:flex; justify-content:center;">
  <div style="width:300px;">
    <img src="https://github.com/Augustus-ThanhHuy/Temperature-PID-Controller-for-ARM-Cortex-M-STM32F103C8T6-/assets/109091737/439a62bc-b144-4284-87a1-d8b772d855cb" alt="Temperature" style="width:60%; height:auto;">
  </div>
</div>

The screen displays the relationship between set point and temperature through STMStudio software

<div style="display: flex; justify-content: center;">
  <div style="width: 300px; height: 300px; border: 1px solid #ccc; overflow: hidden; display: flex; justify-content: center; align-items: center;">
    <img src="https://github.com/Augustus-ThanhHuy/Temperature-PID-Controller-for-ARM-Cortex-M-STM32F103C8T6-/assets/109091737/e83ac2f9-a052-4f61-b293-791afe0bcaeb" alt="PID" style="max-width: 60%; max-height: 100%;">
  </div>
</div>

Overview image when setting up the project

