#Windows OS
#install: pip install pyserial

import serial
import time

ser = serial.Serial('COM4',115200, timeout = 1)  # open serial port
time.sleep(2)    
print(ser.readline())

gcodes = []
gcodes.append('G28')
gcodes.append('GO1 X0 Y0 Z-310')
gcodes.append('GO1 X-54.2522146677235 Y-75.02207424274837 Z-250')
# gcodes.append('GO1 X-100 Y0 Z-260')
# gcodes.append('GO1 X100 Y0 Z-260')
# gcodes.append('GO1 X0 Y0 Z-260')
# gcodes.append('GO1 X0 Y-100 Z-260')
# gcodes.append('GO1 X0 Y100 Z-260')
# gcodes.append('GO1 X0.0 Y-50.0 Z-250')
# gcodes.append('GO1 X50.0 Y-50.0 Z-250')
# gcodes.append('GO1 X-50.0 Y0 Z-250')
# gcodes.append('GO1 X0 Y0 Z-250')
# gcodes.append('GO1 X50.0 Y0 Z-250')
# gcodes.append('GO1 X-50.0 Y50.0 Z-250')
# gcodes.append('GO1 X0 Y50.0 Z-250')
# gcodes.append('GO1 X50.0 Y50.0 Z-250')
# gcodes.append('G01 X0 Y-100 Z-250')
# gcodes.append('G01 X50 Y-100 Z-250')
# gcodes.append('G01 X-50 Y-100 Z-250')
# gcodes.append('G01 X50 Y100 Z-250')
# gcodes.append('G28')

for gcode in gcodes:
    print(gcode)
    gcodeLine = gcode + '\n'
    ser.write(gcodeLine.encode())
    time.sleep(1)
    while 1:
        response = ser.readline()
        print(response)
        if (response.find('Ok'.encode()) > -1):
            break

ser.close()