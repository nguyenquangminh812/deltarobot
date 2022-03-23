
import serial
import time
import serial.tools.list_ports
from ifk import IKinem
from fk import FKinem 
from draw_trajec import draw_line

ser1 = serial.Serial('/dev/ttyACM0',9600, timeout = 1)
while True:
	cur_point=[100,100,150]
	tar_point=[-150,-120,160] 
	data=str(draw_line(cur_point,tar_point,5))
	ser1.write(data.encode('utf-8'))
	time.sleep(2)
