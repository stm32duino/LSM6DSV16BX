# LSM6DSV16BX
Arduino library to support the LSM6DSV16BX 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C, I3C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

For I3C it is then required to create an I3C interface before accessing to the sensors:

    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LSM6DSV16BXSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LSM6DSV16BXSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the I3C bus is used with SETDASA (static-to-dynamic address assignment):

    LSM6DSV16BXSensor AccGyr(&I3C, LSM6DSV16BX_I3C_ADD_H);
    I3C.resetDynamicAddresses();
    I3C.assignDynamicAddress(AccGyr.getStaticAddress(), LSM6DSV16BX_DYNAMIC_ADDRESS);
    AccGyr.begin(LSM6DSV16BX_DYNAMIC_ADDRESS);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

An instance can be created and enabled when the I3C bus is used with ENTDAA (dynamic address discovery):

    LSM6DSV16BXSensor AccGyr(&I3C);
    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);
    I3C.discover(devices, 8, &found);
    // find dynAddr by matching LSM6DSV16BX_I3C_PID_H in discovered devices
    AccGyr.begin(dynAddr);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.Get_X_Axes(accelerometer);  
    AccGyr.Get_G_Axes(gyroscope);

## Examples

* LSM6DSV16BX_DataLog_Terminal_I2C: This application shows how to get data from LSM6DSV16BX accelerometer and gyroscope and print them on terminal.

* LSM6DSV16BX_6D_Orientation_I2C: This application shows how to use LSM6DSV16BX accelerometer to find out the 6D orientation and display data on a hyperterminal.

* LSM6DSV16BX_Double_Tap_Detection_I2C: This application shows how to detect the double tap event using the LSM6DSV16BX accelerometer.

* LSM6DSV16BX_Free_Fall_Detection_I2C: This application shows how to detect the free fall event using the LSM6DSV16BX accelerometer.

* LSM6DSV16BX_MLC_I2C: This application shows how to detect the activity using the LSM6DSV16BX Machine Learning Core.

* LSM6DSV16BX_Pedometer_I2C: This application shows how to use LSM6DSV16BX accelerometer to count steps.

* LSM6DSV16BX_Qvar_Polling_I2C: This application shows how to use LSM6DSV16BX Qvar features in polling mode.

* LSM6DSV16BX_Sensor_Fusion_I2C: This application shows how to use LSM6DSV16BX Sensor Fusion features for reading quaternions.

* LSM6DSV16BX_Single_Tap_Detection_I2C: This application shows how to detect the single tap event using the LSM6DSV16BX accelerometer.

* LSM6DSV16BX_Tilt_Detection_I2C: This application shows how to detect the tilt event using the LSM6DSV16BX accelerometer.

* LSM6DSV16BX_Wake_Up_Detection_I2C: This application shows how to detect the wake-up event using the LSM6DSV16BX accelerometer.

* LSM6DSV16BX_FIFO_Polling_I2C: This application shows how to get accelerometer and gyroscope data from FIFO in pooling mode and print them on terminal.

* LSM6DSV16BX_FIFO_Interrupt_I2C: This application shows how to get accelerometer and gyroscope data from FIFO using interrupt and print them on terminal.

* LSM6DSV16BX_Test_IMU_I2C: This application shows how to perform the accelerometer and gyroscope test.

* LSM6DSV16BX_Temperature_I2C: This application shows how to get temperature data from LSM6DSV16BX sensor.

* LSM6DSV16BX_Datalog_Terminal_I3C: This application shows how to use LSM6DSV16BX accelerometer and gyroscope over I3C using SETDASA.

* LSM6DSV16BX_Datalog_Terminal_I3C_ENTDAA: This application shows how to discover and use LSM6DSV16BX dynamic address over I3C.

## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSV16BX

The LSM6DSV16BX datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsv16bx.html