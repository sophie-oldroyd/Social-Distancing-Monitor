#ifndef WARP_BUILD_ENABLE_DEVHIH
#define WARP_BUILD_ENABLE_DEVHIH
#endif
WarpStatus readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus writeSensorRegisterINA219(uint8_t deviceRegister);
