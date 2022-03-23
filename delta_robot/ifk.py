import math
import numpy as np

R = 415;
r = 38;
l = 415;
L = 192;
pi=np.pi

def IKinem(X, Y, Z):
    theta = np.zeros(4)
    x0 = X;
    y0 = Y;
    z0 = Z;
    theta1 = IKinemTh(x0,y0,z0);
    
    x0 = X*math.cos(2*pi/3) + Y*math.sin(2*pi/3);
    y0 = Y*math.cos(2*pi/3) - X*math.sin(2*pi/3);
    z0 =Z;
    theta2 = IKinemTh(x0,y0,z0);
    x0 = X*math.cos(2*pi/3) - Y*math.sin(2*pi/3);


    y0 =Y*math.cos(2*pi/3) + X*math.sin(2*pi/3);
    z0 = Z;
    theta3 = IKinemTh(x0,y0,z0);
    fl=0
    if (theta1 != None and theta1 != None and theta1 != None):
        fl = 0;
    else:
        fl = -1; 
    theta[0]= theta1
    theta[1] = theta2
    theta[2] = theta3
    theta[3] = fl
    return theta

def IKinemTh(x0, y0, z0):
    theta = 0
    y1 = -R;
    y0 = y0-r; 
    
    a = (pow(x0,2) + pow(y0,2) + pow(z0,2) + pow(L,2) - pow(l,2) - pow(y1,2))/(2*z0);
    b = (y1-y0)/z0;
   
    D = -pow((a + b*y1),2) + pow(L,2)*(pow(b,2)+1);
    if D < 0:
        theta = None;   
    else:  
        yj = (y1 - a*b - math.sqrt(D)) / (pow(b,2) + 1); 
        zj = a + b*yj;
        theta = math.atan(-zj/(y1-yj));
        if (yj>y1):
            theta = theta + np.pi;
        theta = np.rad2deg(theta);
    return theta

#theta = IKinem(-200, 100, -100)
#print(theta)
