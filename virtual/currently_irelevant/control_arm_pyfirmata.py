#example from here
#https://github.com/tino/pyFirmata

import pyfirmata
from pyfirmata import Arduino, util
#board = Arduino('/dev/tty0.usbserial-A6008rIF')
board = Arduino('/dev/ttyUSB0')
#board = Arduino("1d6b:0003")
print(board.get_firmata_version())
board.digital[13].write(1)
it = util.Iterator(board)
it.start()
board.analog[0].enable_reporting()
x=board.analog[0].read()
print(x)
x=7