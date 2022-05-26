import math
import numpy as np
from parameter import R,r,l,L,pi

# R = 145;
# r = 38;
# l = 415;
# L = 192;
# pi=np.pi



def FKinem(theta1, theta2, theta3):
    pos = np.zeros(4)
    t = R-r;
    theta1 = np.deg2rad(theta1);
    theta2 = np.deg2rad(theta2);
    theta3 = np.deg2rad(theta3);
    
    y1 = -(t + L*math.cos(theta1));
    z1 = - L * math.sin(theta1);
    
    y2 = (t + L*math.cos(theta2)) * math.sin(pi/6);
    x2 = y2 * math.tan(pi/3);
    z2 = -L * math.sin(theta2);
    
    y3 = (t + L*math.cos(theta3)) * math.sin(pi/6);
    x3 = -y3 * math.tan(pi/3);
    z3 = -L * math.sin(theta3);
    
    w1 = pow(y1,2) + pow(z1,2);
    w2 = pow(x2,2) + pow(y2,2) + pow(z2,2);
    w3 = pow(x3,2) + pow(y3,2) + pow(z3,2)
    
    dnm = (y2-y1)*x3 - (y3-y1)*x2;
    
    a1 = (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1);
    b1= -( (w2-w1)*(y3-y1) - (w3-w1)*(y2-y1) ) / 2;
    
    a2 = -(z2-z1)*x3 + (z3-z1)*x2;
    b2 = ( (w2-w1)*x3 - (w3-w1)*x2) / 2;
   
    a = pow(a1,2) + pow(a2,2) + pow(dnm,2);
    b = 2 * (a1*b1 + a2*(b2 - y1*dnm) - z1*pow(dnm,2));
    c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + pow(dnm,2)*(pow(z1,2) - l*l)

    d = b*b - 4*a*c;
    if d >= 0:
        Z = -0.5*(b + math.sqrt(d)) / a;
        X = (a1*Z + b1) / dnm;
        Y = (a2*Z + b2) / dnm;
        if (abs(X) > 300 or abs(Y) > 300 or  Z > 0 or Z < -500):
            fl = -1;
        else:
            fl = 0;
        
    else:
        X = None;
        Y = None;
        Z = None;
        fl = -1;
    
    pos[0] = X
    pos[1] = Y
    pos[2] = Z
    pos[3] = fl
    return pos

#pos = FKinem(-30,-40,-50)
#print (pos)
