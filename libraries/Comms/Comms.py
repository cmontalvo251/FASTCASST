import serial as S ##python3 -m pip install pyserial
import struct
import time
import random
import numpy as np

class Comms():
  def __init__(self,BaudRate=57600,port="/dev/ttyAMA0",period=1.0):
    self.lastTime = 0.0
    self.period = period #in seconds
    self.MAXLINE = 120
    print('Creating Serial Port = ',port,' BaudRate = ',BaudRate)
    self.SerialInit(port,BaudRate);
    self.hComm.flush()
    #//Call this for higher level control

  def bitsToFloat(self,b):
    if (b > 2147483647):
      b = -2147483648 + (b - 2147483647)
    s = struct.pack('>l', b)
    return struct.unpack('>f', s)[0]

  def SerialInit(self,ComPortName,BaudRate):
    try:
      print('Trying to open serial port.....')
      self.hComm = S.Serial(ComPortName,BaudRate);
      print('Success!!!!')
    except:
      print('Failed')
      self.hComm = None

  def SerialGetLine(self):
    print('ser.readline()....')
    rxline = self.hComm.readline()
    print('...Read')
    return rxline

  def SerialGetc(self,num=-1):
    #print('ser.read()...')
    if num == -1:
      rxchar = self.hComm.read()
    else:
      rxchar = self.hComm.read(num)
    #print('Read')
    return rxchar;

  def SerialPutc(self,txchar):
    if self.hComm is not None:
      print("Sending ASCII Code = " + str(ord(txchar)))
      self.hComm.write(txchar.encode("ascii"))

  def SerialPutString(self,string):
    for s in string:
      self.SerialPutc(s)

  def SerialGetNumber(self,echo=1):
    bytestring = ''
    rxchar = ''
    ##Read Until \r
    while rxchar != b'\x00\r':
      try:
        bytestring += str(rxchar.decode())
      except:
        pass
      rxchar = self.SerialGetc(2)
    #print('ByteString = ',bytestring)
    
    position = -1
    number = np.nan
    if len(bytestring) > 9:
      if bytestring[1] == ':':
        ##GET POSITION
        position = int(bytestring[0])
        ##CONVERT TO FLOATS
        hexdata = bytestring[2:10]
        #print('Hexdata = ',hexdata)
        integer = int(hexdata,16)
        number = self.bitsToFloat(integer)
    return number,position,bytestring

  def SerialGetLine_DEPRECATED(self,echo=1):
    rxline = self.SerialGetLine()
    return rxline,0

  def SerialGetNumber_DEPRECATED(self,echo=1):
    i = 0;
    j = 0;
    inchar = '\0';
    self.line = ''
    if (echo):
      print("Waiting for characters");
    while inchar != '\r' and i < self.MAXLINE:
      while inchar == '\0' and j < 1000:
        inchar = self.SerialGetc();
        #printf("j = %d i = %d inchar = %c chartoint = %d \n",j,i,inchar,int(inchar));
        j+=1
        if 1:#(echo):
          #if inchar == '\0':
          #  iinchar = 'null'
          #else:
          #  iinchar = int(inchar)
          print('Receving char = ',i,inchar)#,iinchar)
          #print("Receiving: i = %d char = %c chartoint = %d \n",i,inchar,int(inchar));
        #self.line += inchar
        i+=1
        print(i,j,time.monotonic())
    if (echo):
      print("Response received",self.line);
    #Convert to float 
    if len(self.line) > 11 and self.line[8] == '\r':
      hexdata = self.line[2:11]
      print('Hexdata = ',hexdata)
      integer = int(hexdata,16)
      value = bitsToFloat(integer)
      position = 0
    else:
      value = 0
      position = -1
    return value,position

      
  def SerialSendArray(self,number_array,echo=1):
    #union inparser inputvar; ##Need to look up how to do union inparser
    #in python and then convert the number to 8digit hex
    i = 0
    for n in number_array:
      #inputvar.floatversion = number_array[i];
      int_var = int(self.binary(n),2)
      print("Sending = " + str(n) + " " + str(int_var))
      #sprintf(outline,"n:%08x ",int_var);
      hexval=hex(int_var)
      hexval = hexval.replace('0x','')
      outline=str(i)+':'+hexval
      print("Hex = " + outline)
      self.SerialPutString(outline);
      self.SerialPutc('\r');
      i+=1
  def binary(self,num):
    return ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))
  def SerialPutHello(self,echo=1):
    self.SerialPutc('w');
    self.SerialPutc('\r');
  
if __name__ == '__main__':
  ser = Comms(57600,"/dev/ttyAMA0")
  number_array = [0,0]
  for n in range(0,2):
    number_array[n] = random.randint(1,100)
  ser.SerialSendArray(number_array)
