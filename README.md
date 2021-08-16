# stm32_HID

### 1.0.1 (feature)
- added USB HID device in stm32 cube 
- Clock Configuration - MCU working at 48 MHz
- added serial wire debug
- USB device name to STM32 Keyboard

### 1.0.2 (feature)
- added keyboard codes.
- added all the other relevant code for stm32 to work as keyboard.

### 1.0.3 (feature + bugfix)
- added push button for copy and paste function
- changed device name to IEM OPEN CONTROLLER
- push button at PA1, PA2
- fixed the push button extra more than one triggers giving a small delay and condition

### 1.0.4 (feature + update)
- used the joystick module.
- used ADC1 and ADC2 - PA7 for X axis and PA6 for Y axis
- used PA5 as GPIO External Interrupt for Switch
- PA7 and PA6 will do the function of W,A,S,D (not added code)
- PA5 will work as ENTER
- Next thing is to use the Push button as CTRL, SPACE_BAR and others.