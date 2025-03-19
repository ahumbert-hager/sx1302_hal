"lora_packet_time_on_air"

#https://docs.python.org/3/library/ctypes.html#ctypes.c_int
#https://docs.python.org/3/library/ctypes.html#ctypes-pointers
nb_symbols = c_double()

from ctypes import *
cdll.LoadLibrary("./libloragw.dll")
libloragw = CDLL("./libloragw.dll")
print(libloragw.test_uart())
