# Social Distancing Monitor
Sophie Oldroyd, Corpus Christi, svo22

## Summary
This respository contains the firmware for a social distancing montior. The aim of the project was to develop a method of helping a person monitor whether they are adhering to the social distancing guidline of 2m. In addition to the firmware, the project uses the following hardware: FRDMKL03 Development Board, Ultrasonic Distance Sensor, LED, Buzzer, Resistor, Capacitor, Temperature Sensor and Switch. The `FRDMKL03 Devleopment Board` contains the hardware and software to host the project. The `Ultrasonic Distance Sensor`sends out an ultrasound pulse and records the time taken for the reflected pulse to return. The distance can be determine from the Time of Flight. The output of the sensor is a pulse that's width is equal to the time taken for the pulse travel to the person, be reflected, and return to the sensor. The `LED` and `Buzzer` are used as alert mechanisms for when social distancing is breached. The mechanical `switch` is used to turn the output from the buzzer on and off, because it can be very invasive. 

## Hardware Layout
The hardware layout for the project can be seen in the image below. 
![alt text](https://github.com/sophie-oldroyd/Warp-firmware/blob/master/physicallayout.jpg?raw=true)

The mapping table for the OLED pins is given below. 

FRDM-KL03Z I/0 Port Name | FRDM-KL03Z I/0 Header Pin | OLED Port Name | Wire Colour
-------------------------|---------------------------|----------------|-------------
PTA8			 | J4, 2 		     | MOSI (SI)      | Brown
PTA9			 | J4, 1		     | SCK (CK)       | Brown
PTB13			 | J4, 5		     | OCS (OC)	      | White
PTA12			 | J4, 3		     | D/C (DC)	      | Purple
PTB0			 | J2, 6		     | RST (R) 	      | Yellow
GND			 | J3, 7		     | GND (G)	      | Blue
5V 			 | J3, 5		     | VCC (+) 	      | White	

The mapping table for the Ultrasonic Senor pins is given below.

FRDM-KL03Z I/0 Port Name | FRDM-KL03Z I/0 Header Pin | Sensor Port Name | Wire Colour
-------------------------|---------------------------|------------------|-----------
PTB6			 | J1, 4		     | SIG 	        | Orange
GND			 | J3, 7		     | GND 	        | Blue
5V 			 | J3, 5		     | 5V  	        | Purple

The mapping table for the LED is given below.

FRDM-KL03Z I/0 Port Name | FRDM-KL03Z I/0 Header Pin | LED Port Name 		      | Wire Colour
-------------------------|---------------------------|--------------------------------|------------
GND			 | J3, 7		     | G	   		      | Blue
PTA5 			 | J2, 3		     | + (via a resistor)	      | Grey

The mapping table for the buzzer is given below.

FRDM-KL03Z I/0 Port Name | FRDM-KL03Z I/0 Header Pin | Buzzer Port Name 	      | Wire Colour
-------------------------|---------------------------|--------------------------------|------------
GND			 | J3, 7		     | G	   		      | Black
PTA5 (via LED and switch)| J2, 3		     | +                      	      | Grey

The mapping table for the switch is given below. The input to the switch is PTA5 and the output controls the input to the buzzer (red).

FRDM-KL03Z I/0 Port Name | FRDM-KL03Z I/0 Header Pin | Switch Port Name 	      | Wire Colour
-------------------------|---------------------------|--------------------------------|------------
PTA5 (via LED)		 | J2, 3		     | + (via a resistor)	      | Black

The mapping table for the temperature sensor pins is given below.

FRDM-KL03Z I/0 Port Name | FRDM-KL03Z I/0 Header Pin | Sensor Port Name | Wire Colour
-------------------------|---------------------------|------------------|-----------
PTB4			 | J4, 9		     | SDA	        | Green
PTB3			 | J2, 10		     | SCL 	        | Red
GND			 | J3, 7		     | VSS	        | White
5V (via capacitor)	 | J3, 5		     | VDD 	        | Green

## Repository Layout
The firmware for the project is an editted version of the `Warp-firmware` from the `Physical Computation Laboratory` at the `University of Cambridge` run by `Phillip Stanley-Marbell`. The original firmware can be accessed at the following link `https://github.com/physical-computation/Warp-hardware`.
#### `Source files`
##### `CMakeLists.txt`
This is the CMake configuration file. Edit this to change the default size of the stack and heap.

##### `SEGGER_RTT.*`
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### `SEGGER_RTT_Conf.h`
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### `SEGGER_RTT_printf.c`
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### `devHIH.*`
Driver for HIH6121-021-001 temperature sensor. Communication is completed via the I2C Interface.  

##### `devSSD1331.*`
Driver for the SSD1331 OLED display that allows characters to be created. 
Several function included are based on the driver found at `https://os.mbed.com/users/star297/code/ssd1331/file/4385fd242db0/ssd1331.cpp/` written by `Paul Staron`. 
Contains function `charactertoscreen` prints a character (either letter or number) to the screen. 
Contains function `clearscreen` to clear the display. 

##### `gpio_pins.c`
Definition of I/O pin configurations using the KSDK `gpio_output_pin_user_config_t` structure.

##### `gpio_pins.h`
Definition of I/O pin mappings and aliases for different I/O pins to symbolic names relevant to the Warp hardware design, via `GPIO_MAKE_PIN()`.

##### `startup_MKL03Z4.S`
Initialization assembler.

##### `warp-kl03-ksdk1.1-boot.c`
Contains initalisation that starts the device and initialises the sensors.
Contains function `runDevice()` that calls upon different sensors to obtain and display the results. Contains algorithm to determine if social distancing has been breached and raises an alert if it has been. 
Contains function `DistanceSensor()` that is the functionalisation for the Ultrasonic distance sensor, which is not included as a seperate driver because the sensor is analogue. 
Contains function `findSquareRoot()` that is used in the data processing algorithm as the speed of sound is proportional to the square root of the temperature. Algorithm based on that outlined at `https://www.tutorialspoint.com/learn_c_by_examples/square_root_program_in_c.htm`.

## Running the Project
Three prerequisites are needed to compile the project firmware. The first is an `arm cross-compiler`, the second is a `cmake` that works, and the third is a copy of the `SEGGER Jlink Commander`. 

There are three steps involved in compiling the firmware. The first is ensuring that variable `ARMGCC_DIR` is correct by running `export ARMGCC_DIR=<full path to the directory containing bin/arm-none-eabi-gcc>` where the full path can be set to `/usr/local`. The second is ensuring that the file `jlink.commands`, contained in `tools/scripts/jlink.commands`, has the correct full path to access the file that will be outputted by the compilier, `Warp.srec`. This is done inserting `loadfile <full path to the file Warp.srec>` into the `jlink.commands` file on line 3. The third step is navigating into the build directory by running `cd build/ksdk1.1/` and then running `./build.sh` to build the firmware. 

To run the firmware the following two executables need to be run, in seperate command windows, `JLinkExe -device MKL03Z32XXX4 -if SWD -speed 100000 -CommanderScript ../../tools/scripts/jlink.commands`, and then `JLinkRTTClient`. The application interface will appear in the second window. To start the program, press the `/` key. 

