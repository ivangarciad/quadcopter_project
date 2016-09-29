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

    def send_process(self):
      while True:
        user_input = input('Give me a command: ')
        print user_input
        data_to_send = struct.pack('c', chr(user_input))
        self.PuertoSerieArduino.write(data_to_send)

if __name__== "__main__":
  if len(sys.argv) == 1:
    print "Please, introduce the serial dev name.\n"
  else:
    serial_handler = Serial_Handler(sys.argv[1])
    #serial_handler.reader_process()
    serial_handler.send_process()
