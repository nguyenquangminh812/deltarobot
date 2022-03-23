import numpy as np
from ifk import IKinem
from fk import FKinem 

def draw_line(cur,tar,n):
	theta_list = np.zeros((n,4))
	for i in range(n):
		cur_tmp= cur
		cur_tmp[0]+= i*((tar[0]-cur[0])/n)
		cur_tmp[1]+= i*((tar[1]-cur[1])/n)
		cur_tmp[2]+= i*((tar[2]-cur[2])/n)
		theta_list[i] = IKinem(cur_tmp[0],cur_tmp[1],cur_tmp[2])
		print("position",FKinem(theta_list[i][0],theta_list[i][1],theta_list[i][2]))
	return theta_list

