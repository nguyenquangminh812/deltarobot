from __future__ import division
import numpy as np
from parameter import R,r,l,L,pi


# theta1dot = 0;
# theta2dot = 0;
# theta3dot = 0;

def CalThetaDot(q1,q2,q3,x,y,z,Xdot, Ydot, Zdot):

    wB = R;
    uB = 2*R;
    sB = np.sqrt(3)*uB;
    wP = r/2;
    uP = r;
    sP = np.sqrt(3)*uP;
    
    q1 = pi/180 *q1;
    q2 = pi/180 *q2;
    q3 = pi/180 *q3;

    a = wB - uP;
    b = sP/2-np.sqrt(3)/2*wB;
    c = wP - 1/2*wB;

    JX = np.zeros((3,3));
    JX[0,0] = x;
    JX[0,1] = y + a + L*np.cos(q1);
    JX[0,2] = z + L*np.sin(q1);

    JX[1,0] = 2*(x+b)-np.sqrt(3)*L*np.cos(q2);
    JX[1,1]= 2*(y+c) - L*np.cos(q2);
    JX[1,2] =2*(z+L*np.sin(q2));

    JX[2,0] = 2*(x-b)+np.sqrt(3)*L*np.cos(q3);
    JX[2,1] = 2*(y+c) - L*np.cos(q3);
    JX[2,2] = 2*(z+L*np.sin(q3));

    Jphi = np.zeros((3,3));
    Jphi[0,0] = L*((y+a)*np.sin(q1)-z*np.cos(q1));
    Jphi[1,1] = -L*((np.sqrt(3)*(x+b)+y+c)*np.sin(q2)+2*z*np.cos(q2));
    Jphi[2,2] = L*((np.sqrt(3)*(x-b)-y-c)*np.sin(q3)-2*z*np.cos(q3));

    Xd = np.array([[Xdot],[Ydot],[Zdot]]);
    theta1Dot = np.zeros(3);
    thetaDot = np.dot(np.dot(JX,np.linalg.inv(Jphi)),Xd);
    theta1Dot[0] = thetaDot[0]
    theta1Dot[1] = thetaDot[1]
    theta1Dot[2] = thetaDot[2]
    return theta1Dot;

