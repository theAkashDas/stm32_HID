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

### 1.0.5 (feature + update)
- added the interrupt pins for push button.
- changed the pin configurations as required.
- interrupt code is not yet finished.

### 1.0.6 (feature + update)
- added usart for serial port communication
- assigned the push buttons button no. Refer .ioc file
- interrupts in push button is working fine.
- Need to remove the debounce problem in push button. 

### 1.0.7 (update)
- Added a structure for enabling debug.
- Added system debug in the code to print the data.
- need to add stdbool library to support true,false
- code need to be organised.

### 1.0.8 (feature + update)
- Added another print statement SIMPLE DEBUG
- Added i2c2 for mpu
- Added mpu6050 library.

### 1.0.9 (feature + update)
- Added TIM 1 in the code to deal with push button debouncing.
- Added a Timer interrupt counting on push button interrupt.
- need to check this thing thoroughly.
- added adc library and usart library. Will organise the code in the end.
- added structure for mouse controls.
- Added Kalman filter in the code.
- Everything is working fine.

### 1.0.10 (feature)
- Added code for mouse x,y control using mpu data.
- Added quaternion but removed due to some problems.
- couldnt test the mouse control hid feature in windows.
- not working in linux.
- need to find solutions for quaternion

### 1.0.11 (feature)
- Mouse movement functions are working fine with the orientation of the MPU.
- Need to change the sensitivity by changing the newx and newy values.
- Next work is related to walking movement by joystick module.
- Keyboard arrow keys will be assigned to Joystick module





















