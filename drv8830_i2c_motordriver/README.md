# I2C DRV8830 motor driver Example

 
* This example will show you how to use I2C DRV8830 motor driver module
 
    * Read UART
        * 'w' : foword
        * 'd' : back
        * 'b' : brake
        * 'q' : standby
        * 'r' : reset falut
    * Use one I2C port(master mode) to write the DRV8830.
 
* Pin assignment:
 
    * master:
        * GPIO18 is assigned as the data signal of i2c master port
        * GPIO19 is assigned as the clock signal of i2c master port
 
* Connection:
 
    * connect GPIO18 to DRV8830 SDA
    * connect GPIO19 to DRV8830 SCL
    * no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 
* Test items:
 
    * read the uart data.
    * i2c master(ESP32) will write data to i2c DRB8830.
