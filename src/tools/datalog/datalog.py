import struct
import serial

class Datalog():
    def __init__(self):
      try:
        self.PuertoSerie = serial.Serial('/dev/ttyUSB0', 115200)
        print "Puerto serie configurado"
      except serial.serialutil.SerialException:
        print "No se ha encontrado el puerto serie para comunicar"

      self.PuertoSerie.open()
      self.size_package = 24;

    def read(self):
      while 1:
        data_readed = self.PuertoSerie.read(1)
        head = struct.unpack('c', data_readed)
        if hex(ord(head[0])) == '0x21':
          data_readed = self.PuertoSerie.read(self.size_package-1)
          print "Sincronizado"
          while 1:
            data_readed = self.PuertoSerie.read(self.size_package)
            head, head_2, timestamp, reference_pos, angulo_Accx, angulo_Accy, angulo_Accz, angulo_Gyrox, angulo_Gyroy, angulo_Gyroz, angulo_Filtrox, angulo_Filtroy, angulo_Filtroz = struct.unpack('<cchhhhhhhhhhh', data_readed)
            print hex(ord(head)) + " " + hex(ord(head)) + " " + hex(timestamp) + " " + hex(reference_pos) + " " + hex(angulo_Accx)


if __name__== "__main__":
  datalog = Datalog()
  datalog.read()
