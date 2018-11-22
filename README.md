master_i2c_fw : sample usage of SD21 clone
slave_i2c_fw  : SD21 clone firmware

SD21 Specifications:
 - I2C @ == 0xC2 == 11000010b
 - Register 0xN : speed servo N
 - Register 1xN : low byte servo N
 - Register 2xN : high byte servo N
 - Register 64: SW version
 - Register 65: Battery volts

 - timer 20ms
 - min 1000us pulse (theory)
 - max 2000us pulse (theory)
 - min  550us (cortex)
 - max 2600us

Amelioration:
 - OCP: 3A ?
 - Selectable I2C @
 - Calibration mode (all servo put on 0° with 1500ms pulse)

