#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState	deviceHIHState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;


	void
initHIH(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;

	return;
}
	
	
	WarpStatus
writeToTempSensor(uint8_t deviceRegister)
{
    
        i2c_status_t    status;

        	i2c_device_t slave =
	{
		.address = deviceHIHState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

        status = I2C_DRV_MasterSendDataBlocking(
                                0,
                                &slave,
                                NULL,
                                0,
                                NULL,
                                0,
                                5);
        if(status != kStatus_I2C_Success)
        {
                return kWarpStatusDeviceCommunicationFailed;
        }
	SEGGER_RTT_printf(0, "Write");

        	return kWarpStatusOK;
}



WarpStatus
readFromTempSensor(uint8_t deviceRegister, int numberOfBytes)
{
        static int      tempValues;
        i2c_status_t	status;

        i2c_device_t slave =
        {
                .address = deviceHIHState.i2cAddress,
                .baudRate_kbps = gWarpI2cBaudRateKbps
        };



        i2c_status_t    returnValue; 

        returnValue = I2C_DRV_MasterReceiveDataBlocking(
                                0 ,
                                &slave,
                                NULL,
                                0,
                                (uint8_t *)deviceHIHState.i2cBuffer,
                                numberOfBytes,
                                5);

        if(returnValue != kStatus_I2C_Success)
        {
                return kWarpStatusDeviceCommunicationFailed;

        }
        return kWarpStatusOK;

	}
