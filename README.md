# Social Distancing Monitor
Sophie Oldroyd, Corpus Chrisit, svo22

## Summary
This respository contains the firmware for a social distancing montior. The aim of the project was to develop a method of helping a person monitor whether they are adhering to the social distancing guidline of 2m. In addition to the firmware, the project uses the following hardware: FRDMKL03 Development Board, Ultrasonic Distance Sensor, LED, Buzzer, Resistor, Capacitor, Temperature Sensor and Switch. The `FRDMKL03 Devleopment Board` contains the hardware and software to host the project. The `Ultrasonic Distance Sensor`sends out an ultrasound pulse and records the time taken for the reflected pulse to return. The distance can be determine from the Time of Flight. The output of the sensor is a pulse that's width is equal to the time taken for the pulse travel to the person, be reflected, and return to the sensor. The `LED` and `Buzzer` are used as alert mechanisms for when social distancing is breached. The mechanical `switch` is used to turn the output from the buzzer on and off, because it can be very invasive. 

## Repository Layout
The firmware for the project is an editted version of the `Warp-firmware` from the `Physical Computation Laboratory` at the `University of Cambridge` run by `Phillip Stanley-Marbell`. 
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
Basic driver for SSD1331 OLED display. 

##### `mbedSSD1331.*`
Driver for the SSD1331 OLED display that allows characters to be created. Based on the driver found at `https://os.mbed.com/users/star297/code/ssd1331/file/4385fd242db0/ssd1331.cpp/` written by `Paul Staron`. Unneccessary functions were removed. 

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
Contains function `findSQRT()` that is used in the data processing algorithm as the speed of sound is proportional to the square root of the temperature. 


## Running the Project


This is the firmware for the [Warp hardware](https://github.com/physical-computation/Warp-hardware) and its publicly available and unpublished derivatives. This firmware also runs on the Freescale/NXP FRDM KL03 evaluation board which we use for teaching at the University of Cambridge. When running on platforms other than Warp, only the sensors available in the corresponding hardware platform are accessible.

**Prerequisites:** You need an arm cross-compiler such as `arm-none-eabi-gcc` installed as well as a working `cmake` (installed, e.g., via `apt-get` on Linux or via [MacPorts](https://www.macports.org) on macOS). You will also need an installed copy of the SEGGER [JLink commander](https://www.segger.com/downloads/jlink/), `JlinkExe`, which is available for Linux, macOS, and Windows (here are direct links for downloading it for [macOS](https://www.segger.com/downloads/jlink/JLink_MacOSX.pkg), and [Linux tgz 64-bit](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.tgz)).

## 1.  Compiling the Warp firmware
First, make sure the environment variable `ARMGCC_DIR` is set correctly (you can check whether this is set correctly, e.g., via `echo $ARMGCC_DIR`; if this is unfamiliar, see [here](http://homepages.uc.edu/~thomam/Intro_Unix_Text/Env_Vars.html) or [here](https://www2.cs.duke.edu/csl/docs/csh.html)). If your `arm-none-eabi-gcc` is in `/usr/local/bin/arm-none-eabi-gcc`, then you want to set  `ARMGCC_DIR` to `/usr/local`. If your shell is `tcsh`:
```
  setenv ARMGCC_DIR <full path to the directory containing bin/arm-none-eabi-gcc>
```
Alternatively, if your shell is `bash`
```
  export ARMGCC_DIR=<full path to the directory containing bin/arm-none-eabi-gcc>
```
(You can check what your shell is, e.g., via `echo $SHELL`.) Second, edit the jlink command file, `tools/scripts/jlink.commands` to include the correct path.

Third, you should be able to build the Warp firmware by

	cd build/ksdk1.1/
	./build.sh

This copies the files from `Warp/src/boot/ksdk1.1.0/` into the KSDK tree, builds, and converts the binary to SREC. See 	`Warp/src/boot/ksdk1.1.0/README.md` for more. _When editing source, edit the files in `Warp/src/boot/ksdk1.1.0/`, not the files in the build location, since the latter are overwritten during each build._

> **NOTE:** If you run into a compile error such as `/usr/lib/gcc/arm-none-eabi/6.3.1/../../../arm-none-eabi/bin/ld: region
m_data overflowed by 112 bytes`, the error is that the firmware image size exceeded the KL03 memory size. Some arm-gcc cross compilers, particularly on Linux, generate firmware images that are quite large. Easiest fixes are either:
>
> - Comment out some of the driver includes in the list between lines 64 and 71 (these trigger the instantiation of various driver data structures which take up memory)
>
> or
>
> - Modify src/boot/ksdk1.1.0/CMakeLists.txt and reduce the default stack size, e.g., by changing all occurrences of "__stack_size__=0x300” to, e.g., "__stack_size__=0x100"



Fourth, you will need two terminal windows. In one shell window, run the firmware downloader:
```
  JLinkExe -device MKL03Z32XXX4 -if SWD -speed 100000 -CommanderScript ../../tools/scripts/jlink.commands
```
In the other shell window, launch the JLink RTT client<sup>&nbsp;<a href="#Notes">See note 1 below</a></sup>:

	JLinkRTTClient

## 2. Using the Warp firmware on the Freescale FRDMKL03 Board
The SEGGER firmware allows you to use SEGGER’s JLink software to load your own firmware to the board, even without using their specialized JLink programming cables. You can find the SEGGER firmware at the SEGGER Page for [OpenSDA firmware](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/opensda-sda-v2/).

To build the Warp firmware for the FRDM KL03, you will need to uncomment the `#define WARP_FRDMKL03` define in `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c`. When building for the FRDMKL03 board, you can also disable drivers for sensors that are not on the FRDMKL03 (i.e., disable all sensors except the MMA8451Q). The full set of diffs is:
```diff
diff --git a/src/boot/ksdk1.1.0/CMakeLists.txt b/src/boot/ksdk1.1.0/CMakeLists.txt
index 5cd6996..197e0a5 100755
--- a/src/boot/ksdk1.1.0/CMakeLists.txt
+++ b/src/boot/ksdk1.1.0/CMakeLists.txt
@@ -89,19 +89,19 @@ ADD_EXECUTABLE(Warp
     "${ProjDirPath}/../../../../platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S"
     "${ProjDirPath}/../../src/warp-kl03-ksdk1.1-boot.c"
     "${ProjDirPath}/../../src/warp-kl03-ksdk1.1-powermodes.c"
-    "${ProjDirPath}/../../src/devBMX055.c"
+#    "${ProjDirPath}/../../src/devBMX055.c"
 #    "${ProjDirPath}/../../src/devADXL362.c"
     "${ProjDirPath}/../../src/devMMA8451Q.c"
 #    "${ProjDirPath}/../../src/devLPS25H.c"
-    "${ProjDirPath}/../../src/devHDC1000.c"
-    "${ProjDirPath}/../../src/devMAG3110.c"
+#    "${ProjDirPath}/../../src/devHDC1000.c"
+#    "${ProjDirPath}/../../src/devMAG3110.c"
 #    "${ProjDirPath}/../../src/devSI7021.c"
-    "${ProjDirPath}/../../src/devL3GD20H.c"
-    "${ProjDirPath}/../../src/devBME680.c"
+#    "${ProjDirPath}/../../src/devL3GD20H.c"
+#    "${ProjDirPath}/../../src/devBME680.c"
 #    "${ProjDirPath}/../../src/devTCS34725.c"
 #    "${ProjDirPath}/../../src/devSI4705.c"
-    "${ProjDirPath}/../../src/devCCS811.c"
-    "${ProjDirPath}/../../src/devAMG8834.c"
+#    "${ProjDirPath}/../../src/devCCS811.c"
+#    "${ProjDirPath}/../../src/devAMG8834.c"
 #    "${ProjDirPath}/../../src/devRV8803C7.c"
 #    "${ProjDirPath}/../../src/devPAN1326.c"
 #    "${ProjDirPath}/../../src/devAS7262.c"
diff --git a/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c b/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c
index 87a27e1..42ce458 100755
--- a/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c
+++ b/src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c
@@ -55,7 +55,7 @@
 #include "SEGGER_RTT.h"
 #include "warp.h"
 
-//#define WARP_FRDMKL03
+#define WARP_FRDMKL03
```


## 3.  Editing the firmware
The firmware is currently all in `src/boot/ksdk1.1.0/`, in particular, see `src/boot/ksdk1.1.0/warp-kl03-ksdk1.1-boot.c` and the per-sensor drivers in `src/boot/ksdk1.1.0/dev*.[c,h]`.

The firmware builds on the Kinetis SDK. You can find more documentation on the Kinetis SDK in the document [doc/Kinetis SDK v.1.1 API Reference Manual.pdf](https://github.com/physical-computation/Warp-firmware/blob/master/doc/Kinetis%20SDK%20v.1.1%20API%20Reference%20Manual.pdf).

The firmware is designed for the Warp hardware platform, but will also run on the Freeacale FRDM KL03 development board. In that case, the only driver which is relevant is the one for the MMA8451Q. For more details about the structure of the firmware, see [src/boot/ksdk1.1.0/README.md](src/boot/ksdk1.1.0/README.md).

## 4.  Interacting with the boot menu
When the firmware boots, you will be dropped into a menu with a rich set of commands. The Warp boot menu allows you to conduct most of the experiments you will likely need without modifying the firmware:
````
[ *				W	a	r	p	(rev. b)			* ]
[  				      Cambridge / Physcomplab   				  ]

	Supply=0mV,	Default Target Read Register=0x00
	I2C=200kb/s,	SPI=200kb/s,	UART=1kb/s,	I2C Pull-Up=32768

	SIM->SCGC6=0x20000001		RTC->SR=0x10		RTC->TSR=0x5687132B
	MCG_C1=0x42			MCG_C2=0x00		MCG_S=0x06
	MCG_SC=0x00			MCG_MC=0x00		OSC_CR=0x00
	SMC_PMPROT=0x22			SMC_PMCTRL=0x40		SCB->SCR=0x00
	PMC_REGSC=0x00			SIM_SCGC4=0xF0000030	RTC->TPR=0xEE9

	0s in RTC Handler to-date,	0 Pmgr Errors
Select:
- 'a': set default sensor.
- 'b': set I2C baud rate.
- 'c': set SPI baud rate.
- 'd': set UART baud rate.
- 'e': set default register address.
- 'f': write byte to sensor.
- 'g': set default SSSUPPLY.
- 'h': powerdown command to all sensors.
- 'i': set pull-up enable value.
- 'j': repeat read reg 0x00 on sensor #3.
- 'k': sleep until reset.
- 'l': send repeated byte on I2C.
- 'm': send repeated byte on SPI.
- 'n': enable SSSUPPLY.
- 'o': disable SSSUPPLY.
- 'p': switch to VLPR mode.
- 'r': switch to RUN mode.
- 's': power up all sensors.
- 't': dump processor state.
- 'u': set I2C address.
- 'x': disable SWD and spin for 10 secs.
- 'z': dump all sensors data.
Enter selection>
````

### Acknowledgements
This research is supported by an Alan Turing Institute award TU/B/000096 under EPSRC grant EP/N510129/1, by Royal Society grant RG170136, and by EPSRC grants EP/P001246/1 and EP/R022534/1.
