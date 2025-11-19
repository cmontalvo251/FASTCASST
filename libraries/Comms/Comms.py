import serial as S ##sudo python3 -m pip install pyserial (sudo needed for rpi)
import struct
import time
import random
import numpy as np
import sys
sys.path.append('../Util') ##Assuming we're launching this from within the Comms folder ../ would bring us to the library folder and then Util will put us in the Util folder
sys.path.append('../libraries/Util') ##Assuming we're importing this from either src/ or ground_station/ we need ../ to get to the parent folder and then libraries/Util to get to the Util folder
import util

class Comms():
  def __init__(self):
    self.lastTime = 0.0
    self.MAXLINE = 120
    self.fast_packet = np.zeros(8)
    
  def bitsToFloat(self,b):
    if (b > 2147483647):
      b = -2147483648 + (b - 2147483647)
    s = struct.pack('>l', b)
    return struct.unpack('>f', s)[0]

  def lineToFloat(self,line):
    ##Ok this loop works. Time to unpack properly
    #Remove first 2 chars because that's just the number
    #followed by a colon (Also remove the trailing two characters. They are \r\n I think)
    hexdata = line[2:-2]
    #Ok now what we do is convert the hex number to an int
    integer = int(hexdata,16)
    #Then finally we convert the int bits to float
    value = self.bitsToFloat(integer)
    #And print it for debugging
    #print('0x = ',hexdata,' intbits = ',integer,' float = ',value)
    return value

  def SerialInit(self,BaudRate=57600,ComPortName="/dev/ttyAMA0",period=1.0):
    print('Creating Serial Port = ',ComPortName,' BaudRate = ',BaudRate)
    self.period = period #in seconds
    try:
      print('Trying to open serial port.....')
      self.hComm = S.Serial(ComPortName,BaudRate);
      print('Success!!!!')
      self.hComm.flush()
    except:
      print('Failed')
      self.hComm = None
      return

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

  def SerialPutc(self,txchar,echo):
    if self.hComm is not None:
      if echo:
        print("Sending ASCII Code = " + str(ord(txchar)))
      self.hComm.write(txchar.encode("ascii"))

  def SerialPutString(self,string,echo=1):
    for s in string:
      self.SerialPutc(s,echo)

  def SerialGetNumber(self,echo=1):
    if self.hComm is None:
      return -1,-1,''
    bytestring = ''
    rxchar = ''
    ##Read Until \r
    #while rxchar != b'\x00\r': #Also think we should stop once we get a \r? \r is 13 in ASCII
    while rxchar != b'\r':
      try:
        bytestring += str(rxchar.decode())
      except:
        pass
      #rxchar = self.SerialGetc(2) #Why are we reading 2 bytes? I think we should be reading just 1 right?
      rxchar = self.SerialGetc(1) #Just read 1 byte at a time
      if echo:
        print('rxchar = ',rxchar)
    if echo:
      print('ByteString = ',bytestring)
    
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
      value = self.bitsToFloat(integer)
      position = 0
    else:
      value = 0
      position = -1
    return value,position

  def SerialSend(self,echo=1):
    self.SerialSendArray(self.fast_packet,echo)
      
  def SerialSendArray(self,number_array,echo=1):
    #union inparser inputvar; ##Need to look up how to do union inparser
    #in python and then convert the number to 8digit hex
    i = 0
    for n in number_array:
      #inputvar.floatversion = number_array[i];
      int_var = int(self.binary(n),2)
      if echo:
        print("Sending = " + str(n) + " " + str(int_var))
      #sprintf(outline,"n:%08x ",int_var);
      hexval=hex(int_var)
      hexval = hexval.replace('0x','')
      outline=str(i)+':'+hexval
      if echo:
        print("Hex = " + outline)
      self.SerialPutString(outline,0);
      self.SerialPutc('\r',0);
      i+=1
      
  def binary(self,num):
    return ''.join('{:0>8b}'.format(c) for c in struct.pack('!f', num))

  def SerialPutHello(self,echo=1):
    self.SerialPutc('w',0);
    self.SerialPutc('\r',0);
  
if __name__ == '__main__':

    ##Alright when running this main routine I want it to sort of be a debug mode
    ##So the pi will send numbers to the computer and the computer will read the numbers

    ##Kick off the serial object
    ser = Comms()

    ##Check to see if you're on the computer or on the rpi
    pc = util.isSIL()

    #Open the com port and baudrate
    baudRate = 57600
    if pc:
        port = "/dev/ttyUSB0"
    else:
        port = "/dev/ttyAMA0"
    ser.SerialInit(baudRate,port)

    number_array = [0,0]

    ##Now run different routines depending on what computer we're on
    while True:
      if pc:
        ##So how does SerialGetNumber work?
        #Ok so basically the way the code works is it reads 2 bytes at a time
        #until it reads \x00\r which is NULL,\r once it gets that it stops
        #and returns the bytestring
        #Once it has the bytestring it grabs everything between : and \r
        #and converts that to a 32 bit integer. Then converts the 32 bits
        #back to a 32 bit floating point number
        #So.....I think the reason why this isn't working is because SerialGetNumber is reading
        #2 bytes at a time and it's waiting for \x00\r
        #So I'm changing the serialgetnumber routine to read 1 byte at a time and waiting for \r
        
        ###Read serial
        value,position,bytestring = ser.SerialGetNumber(1)
        print('Value = ',value,'Position = ',position,' bytestring = ',bytestring)
        number_array[position] = value
        print('Number Array = ',number_array)
      else:
        ##Debug by sending two numbers
        for n in range(0,2):
          number_array[n] = random.randint(1,100)
        print('Number Array = ',number_array)
        ser.SerialSendArray(number_array)
        ##Normally you would do this
        #ser.fast_packet[0] = number
        #....
        #ser.fast_packet[7] = number
        #ser.SerialSend(0)

        #Let's dive into how SerialSend works
        ##SerialSend(echo) is just SerialSendArray(self.fast_packet,echo)
        #SerialSendArray loops through every number in the array
        #First it converts the number to it's IEE 754 standard of 32 bits
        #This is where you have the Sign (S-1bit), Exponent (E-8bits) and Fraction (M-23bits)
        #Then it takes the 32 bit number and converts it to an integer
        #Since it's 32 bits it would be a number between 0-4,294,967,295
        #But if you take 32 bits and break it into 8 bit groups you get
        # 8   8   8   8   so 4 groups of 8 bits. if you then further break down to 4 bits
        # 4 4 4 4 4 4 4 4 so 8 groups of 4 bits. So 2**4 = 16 which means you can convert
        # each set of 4 bits into hex values 0-F. Putting it all together you get
        # a floating point number converted to a 32 bit hex number with 8 digits
        # So wait. 8 digits? But like XXXX.XXXX is plenty of precision with 8 digits
        # because the decimal would be implied. So why are we doing this? Hmmmm...
        # Anyway. Say you take the number
        # number = 45
        # binary IEE 754 = 01000010001101000000000000000000
        # hex = 0x42340000
        # Then they take the index of the number in the array and append it and a :
        # They also add a \r
        # outline = 3:42340000\r
        # Ok so now we have 11 digits (\r) is one digit.
        # Once the string has been created the function SerialPutString is called
        # SerialPutString loops through the outline string and calls SerialPutc for every character
        # or in this case 11 times for the 11 digits from above
        # SerialPutc sends 1 byte at a time by sending the ascii value of each character
        # So you take a floating point number which is 32 bits or 8 bytes and you end up sending
        # 11 bytes over serial (8 bytes for the hex digits and then 3 bytes for index,:,\r)
        # Note that we can make this way better by just sending the byte array itself
        # struct.pack('>f',number) and then send 4 bytes for the number
        # If you have the hex values you can do this
        # bytearray.fromhex(str(hex)[2:]) - the 2: ignores the 0x that supercedes hex values

        ##Send numbers every second so you don't overload
        #the pc
        time.sleep(5.0)
	    
    
