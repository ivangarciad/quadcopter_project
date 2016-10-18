import sys
import struct
import serial
import time

class Serial_Handler:
    def __init__(self, serial_dev_name):
      self.PuertoSerieArduino = serial.Serial(serial_dev_name, 9600)

    def reader_process(self):
      while True:
        sArduino = self.PuertoSerieArduino.readline()
        print sArduino.rstrip('\n')

    def send_to_arduino(self, azimuth, elevation):
      if (time.time() - self.t1) > 0.1:
        data_to_send = struct.pack('cccccc', 'a', chr(azimuth), ';', 'e', chr(elevation), ';')
        self.PuertoSerieArduino.write(data_to_send)
        self.t1 = time.time()

if __name__== "__main__":
  if len(sys.argv) == 1:
    print "Please, intorude the serial dev name.\n"
  else:
    serial_handler = Serial_Handler(sys.argv[1])
    serial_handler.reader_process()
