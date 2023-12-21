import serial

ser1 = serial.Serial('COM12',  115200, timeout=1)
ser = serial.Serial('COM16',  115200, timeout=1)
while 1:
    res = ser.readline()
    print(res.decode('Ascii'))
    res1 = res.decode('ascii')
    res3 = str(res1)
    ser1.write(res3.encode("Ascii"))

