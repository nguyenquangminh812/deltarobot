import numpy as np
from ifk import IKinem
from fk import FKinem
from parameter import R,r,l,L,pi,sample
from velocityIKnem import CalThetaDot

# quỹ đạo hình elise
# startP = np.array([[100],[0],[-450]]);
# endP = np.array([[-100],[0],[-450]]);

Af = 1/2;
T = 0.5;
af = 0.125;

Ca = (1+4*af/pow(pi,2))/(af/pi+2/pow(pi,2));

af = (pow(pi,2)-2*Ca)/(Ca*pi-4*Ca);



x = 0;
xdot = 0;
x2dot = 0;

PosAndVel = np.zeros((sample+1,6))

def calPosElipe(startP,endP):
	for i in range(sample +1):
		t = i/sample*T
		u = t/T
		if u>=0 and u <= (Af*af):
			x = Ca*af/pi*u-2*Ca*Af*pow(af,2)/pow(pi,2)*np.sin(pi*u/(2*Af*af))
			xdot = Ca * af / pi / T - Ca * af / pi / T * np.cos(pi * t / T / Af / af / 0.2e1)
			#x2dot = Ca / pow(T, 2) / Af * np.sin(pi * t / T / Af / af / 0.2e1) / 0.2e1
		elif u>(Af*af) and u<=Af:
			x = 2*(Ca*Af*(1-2*af))/pow(pi,2)+Ca*af/pi*u-2*Ca*Af*pow((1-af),2)/pow(pi,2)*np.sin(pi*u/(2*Af*(1-af))+(1-2*af)*pi/(2*(1-af)));
			xdot = Ca * af / pi / T - Ca * (1 - af) / pi / T * np.cos(-(pi * t / T / Af / (1 - af)) / 0.2e1 - ((1 - 2 * af) * pi / (2 - 2 * af)));
			#x2dot = -Ca / (pow(T, 2)) / Af * np.sin(-(pi * t / T / Af / (1 - af)) / 0.2e1 - ((1 - 2 * af) * pi / (2 - 2 * af))) / 0.2e1;
		elif u>Af and u<=(1-af*(1-Af)):
			x = 1-2*Ca*(1-Af)*(1-2*af)/pow(pi,2)-Ca*af*(1-u)/pi+2*Ca*(1-Af)*pow((1-af),2)/pow(pi,2)*np.sin(pi*(1-u)/(2*(1-Af)*(1-af))+(1-2*af)*pi/(2*(1-af)));
			xdot = Ca * af / pi / T - 0.2e1 * Ca * (1 - Af) * (1 - af) / pi / T / (2 - 2 * Af) * np.cos((-pi * (1 - t / T) / (2 - 2 * Af) / (1 - af) - (1 - 2 * af) * pi / (2 - 2 * af)));
			#x2dot = 0.2e1 * Ca * (1 - Af) / (pow(T, 2)) / (pow(2 - 2 * Af) ,2) * np.sin((-pi * (1 - t / T) / (2 - 2 * Af) / (1 - af) - (1 - 2 * af) * pi / (2 - 2 * af)));
		elif u > (1-af*(1-Af)) and u <=1:
			x = 1-Ca*af*(1-u)/pi-2*Ca*(1-Af)*pow(af,2)/pow(pi,2)*np.sin(pi*(1-u)/(2*(1-Af)*af));
			xdot = Ca * af / pi / T + 0.2e1 * Ca * (1 - Af) * af / pi / T / (2 - 2 * Af) * np.cos((pi * (1 - t / T) / (2 - 2 * Af) / af));
			#x2dot = 0.2e1 * Ca * (1 - Af) / (pow(T, 2)) / (pow(2 - 2 * Af) , 2) * np.sin((pi * (1 - t / T) / (2 - 2 * Af) / af));
		s0 = (startP+endP)/2;
		phi = np.arctan((startP[1]-endP[1])/(startP[0]-endP[0]));
		A = 1/2*np.sqrt(pow((startP[0]-endP[0]),2)+pow((startP[1]-endP[1]),2));
		B = 50;
		C = (startP[2]-endP[2])/2;

		rot = np.array([[np.cos(phi),-np.sin(phi),0], [np.sin(phi) ,np.cos(phi) ,0], [0, 0, 1]]);
		ABC = np.array([[A, 0, 0], [0, 1, 0], [C, 0, B]]);
		ss0 = np.array([[np.cos(pi*x)], [0], [np.sin(pi*x)]]);
		ss0dot = np.array([[-xdot*pi*np.sin(pi*x)],[0], [xdot*pi*np.cos(pi*x)]]);

		s = s0 + np.dot(np.dot(rot,ABC),ss0)
		sdot = np.dot(np.dot(rot,ABC),ss0dot)

		PosAndVel[i][0] = s[0];
		PosAndVel[i][1] = s[1];
		PosAndVel[i][2] = s[2];
		PosAndVel[i][3] = sdot[0];
		PosAndVel[i][4] = sdot[1];
		PosAndVel[i][5] = sdot[2];

	return PosAndVel;

def calVelSignal(startP,endP):
	XAndDX = calPosElipe(startP,endP);
	
	qAndDq = np.zeros((sample+1,6));
	# dq = np.zeros((sample+1,3));
	for i in range(sample +1):
		qAndDq[i][0:3] = IKinem(XAndDX[i][0],XAndDX[i][1],XAndDX[i][2]);
		qAndDq[i][3:6] = CalThetaDot(qAndDq[i][0],qAndDq[i][1],qAndDq[i][2],XAndDX[i][0],XAndDX[i][1],XAndDX[i][2],XAndDX[i][3], XAndDX[i][4], XAndDX[i][5]);
	return qAndDq;


