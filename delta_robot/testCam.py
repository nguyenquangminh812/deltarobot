# -*- coding: utf-8 -*-
from __future__ import division
import cv2
import numpy as np
import serial
import time
import serial.tools.list_ports
from parameter import Tiso,vibuoc,sample
from draw_trajec import calVelSignal

#cam = cv2.VideoCapture(0)
ser1 = serial.Serial('/dev/ttyACM0',115200)
# ser1.reset_input_buffer()

startP = np.array([[100],[0],[-440]]);
#endP = np.array([[-100],[0],[-450]]);
endP = np.array([[0],[0],[-440]])

def CalStepVel(startP,endP):
    ThetaAndVel = np.zeros((sample+1,6));
    ThetaAndVel = calVelSignal(startP,endP);
    stepNum = np.zeros((sample +1,3));
    velNum = np.zeros((sample,3));
    

    calStepNum = np.zeros((sample,3));
    calVelNum = np.zeros((sample,3));

    for i in range(sample+1):
        stepNum[i] = ThetaAndVel[i][0:3];
    for i in range(sample):
        velNum[i] = ThetaAndVel[i+1][3:6];
    #print(stepNum);
    for i in range(sample):
        calStepNum[i][0] = int((stepNum[i+1][0] - stepNum[i][0])/(360 / 800 / vibuoc) *Tiso); 
        calStepNum[i][1] = int((stepNum[i][1] - stepNum[i+1][1])/(360 / 800 / vibuoc) *Tiso); 
        calStepNum[i][2] = int((stepNum[i+1][2] - stepNum[i][2])/(360 / 800 / vibuoc) *Tiso);
    # print(calStepNum) 

    for i in range(sample):
        calVelNum[i][0] = int(abs(velNum[i][0])*(sample+1));
        calVelNum[i][1] = int(abs(velNum[i][1])*(sample+1));
        calVelNum[i][2] = int(abs(velNum[i][2])*(sample+1));
	if calVelNum[i][0] == 0:
		calVelNum[i][0] = 1;
	if calVelNum[i][1] == 0:
		calVelNum[i][1] = 1;
	if calVelNum[i][2] == 0:
		calVelNum[i][2] = 1;
    
    # print(calVelNum);
    arrStep = np.reshape(calStepNum, -1, order='F')
    arrVel = np.reshape(calVelNum, -1, order='F')

    #arrStep= np.round(arrStep.transpose()).astype(int)
    arrStepVel = np.concatenate((arrStep,arrVel),axis=None)
    arrStepVel= np.round(arrStepVel.transpose()).astype(int)
    return arrStepVel;

def sendData(startP,endP):
    for i in range(2):
    	data = CalStepVel(startP,endP);
    	#data= np.round(data.transpose()).astype(int)
    	listToStr = ','.join([str(elem) for elem in data])
    	print(listToStr);
    	#strdata = listToStr+"\n"
    	ser1.write((listToStr+"\n").encode());
   	#ser1.write(("-31,-48,-64,-80,-95,-107,-115,-116,-109,-93,-68,-36,0,36,68,93,109,116,115,107,95,80,64,48,31,32,47,57,63,64,56,41,16,-16,-57,-102,-146,-184,-212,-226,-228,-217,-198,-173,-145,-117,-91,-68,-47,-29,-29,-47,-68,-91,-117,-145,-173,-198,-217,-228,-226,-212,-184,-146,-102,-57,-16,16,41,56,64,63,57,47,32,7,10,14,17,21,24,26,26,24,19,12,4,4,13,19,24,26,26,24,21,17,14,10,7,4,13,16,17,16,10,1,12,30,52,76,100,122,138,147,147,139,126,108,88,69,51,36,24,15,8,15,24,36,51,68,88,107,125,139,147,146,138,122,100,75,51,29,11,1,10,16,17,16,13,8"+"\n").encode());
    	time.sleep(0.5)
posX= -100
posY = 20

while True:
	
     endP = np.array([[posX],[posY],[-440]])
     sendData(startP,endP);
     time.sleep(8)
     posX += 2;
     posY -= 10;
     if(posY < -10):
     	break;		
