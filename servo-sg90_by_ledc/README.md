# Drive servo SG90 by LEDC (LED PWM Controller) Example

This example shows how to control intensity of servo SG90 using ESP32's on-board hardware LED PWM Controller module.

## Functionality

Operations performed by example:

* Configuration of one timer to high speed that will be clocking four LEDC channels.

* Configuration of one channels of LEDC module, these channel will drive one GPIO / Servo.

* Operation of channels in a UART key event. That will drive servo as follows:

  'a' : Decrease duty ratio
  's' : Increase duty ratio
  'q' : Stop ledc. Restart when another event comes.

## Hardware Setup

Connect four servo to the following LEDC channels / individual GPIOs:

  * Channel 0 - GPIO5 - to Servo signal pin
  * Channel 1 - N/A
  * Channel 2 - N/A
  * Channel 3 - N/A

