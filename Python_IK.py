import numpy as np
from numpy import linalg


import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi
import numpy as np


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

import matplotlib

matplotlib.use("TkAgg")

global mat
mat=np.matrix


# ****** Coefficients ******


'''global d1, a2, a3, a7, d4, d5, d6
d1 =  0.1273
a2 = -0.612
a3 = -0.5723
a7 = 0.075
d4 =  0.163941
d5 =  0.1157
d6 =  0.0922

global d, a, alph

#d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) ur5
d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 mm
# a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) ur5
a =mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10 mm
#alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])  #ur5
alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10'''



global d, a, alph

d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823]) #ur5
#d = mat([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])#ur10 mm
a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0]) #ur5
#a =mat([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])#ur10 mm
alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])  #ur5
#alph = mat([pi/2, 0, 0, pi/2, -pi/2, 0 ]) # ur10

hand_size = 0.075

global d1, a2, a3, a7, d4, d5, d6
d1 =  d[0,0]
a2 = a[0,1]
a3 = a[0,2]
a7 = hand_size
d4 =  d[0,3]
d5 =  d[0,4]
d6 =  d[0,5]

# ************************************************** FORWARD KINEMATICS
def rpy(x, y, z):
    roll_x = mat([[1, 0, 0, 0], 
                [0, cos(x), -sin(x), 0],
                [0, sin(x), cos(x), 0],
                [0, 0, 0, 1]
    ])

    pitch_y = mat([
    [cos(y), 0, sin(y), 0],
    [0, 1, 0, 0],
    [-sin(y), 0, cos(y), 0],
    [0, 0, 0, 1]
    ])

    yaw_z = mat([
    [cos(z), -sin(z), 0, 0],
    [sin(z), cos(z), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
    ])

    return yaw_z * pitch_y * roll_x

def pos(g, x, y, z):
    g[0,3] = x
    g[1,3] = y
    g[2,3] = z
    return g

def AH( n,th,c  ):

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
	         [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)
      

  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
	    

  return A_i

def HTrans(th,c):  
  A_1=AH( 1,th,c  )
  A_2=AH( 2,th,c  )
  A_3=AH( 3,th,c  )
  A_4=AH( 4,th,c  )
  A_5=AH( 5,th,c  )
  A_6=AH( 6,th,c  )
      
  T_06=A_1*A_2*A_3*A_4*A_5*A_6

  return T_06

# ************************************************** INVERSE KINEMATICS 

def invKine(desired_pos):# T60
  th = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0,0, -d6, 1]).T-mat([0,0,0,1 ]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1,0], P_05[1-1,0])
  phi = acos(d4 /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0]))
  #The two solutions for theta1 correspond to the shoulder
  #being either left or right
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_16 = T_10 * desired_pos
    th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6)
    th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6)

  th = th.real
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_16 = linalg.inv( T_10 * desired_pos )
    th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
		  
  th = th.real

  # **** theta3 ****
  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH(1,th,c))
    T_65 = AH( 6,th,c)
    T_54 = AH( 5,th,c)
    T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
    P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
    t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
    th[2, c] = t3.real
    th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
    c = cl[i]
    T_10 = linalg.inv(AH( 1,th,c ))
    T_65 = linalg.inv(AH( 6,th,c))
    T_54 = linalg.inv(AH( 5,th,c))
    T_14 = (T_10 * desired_pos) * T_65 * T_54
    P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
    
    # theta 2
    th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
    # theta 4
    T_32 = linalg.inv(AH( 3,th,c))
    T_21 = linalg.inv(AH( 2,th,c))
    T_34 = T_32 * T_21 * T_14
    th[3, c] = atan2(T_34[1,0], T_34[0,0])
  th = th.real

  return th

def find_joints(th, c):
    A_1=AH( 1,th,c  )
    A_2=AH( 2,th,c  )
    A_3=AH( 3,th,c  )
    A_4=AH( 4,th,c  )
    A_5=AH( 5,th,c  )
    A_6=AH( 6,th,c  )

    T_01 = A_1
    T_02 = T_01 * A_2
    T_03 = T_02 * A_3
    T_04 = T_03 * A_4
    T_05 = T_04 * A_5
    T_06 = T_05 * A_6


    x = [T_01[0,3], T_02[0,3], T_03[0,3], T_04[0,3], T_05[0,3], T_06[0,3]]
    y = [T_01[1,3], T_02[1,3], T_03[1,3], T_04[1,3], T_05[1,3], T_06[1,3]]
    z = [T_01[2,3], T_02[2,3], T_03[2,3], T_04[2,3], T_05[2,3], T_06[2,3]]

    return x, y, z

def find_dtheta(th_current, th, c, steps):
   return (th_current - th[:,c])/steps

def find_dstep(th,c,xp,yp,zp, steps):
   xg, yg, zg = find_joints(th, c)
   dx = (mat(xg)-mat(xp))/steps
   dy = (mat(yg)-mat(yp))/steps
   dz = (mat(zg)-mat(zp))/steps
   return dx, dy,dz 

def update_linear(frame, sc, x, y, z, dx, dy,dz):
   for i in range(len(x)):
        x[i] += dx[0, i]
        y[i] += dy[0,i]
        z[i] += dz[0, i]
   sc._offsets3d = (x,y,z)
   return sc


   



# ************************************************** Plotting
def show(th, c):
    x,y,z = find_joints(th, c)

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Connect the points with lines
    ax.plot(x, y, z, c='gray', linewidth=20, label='Lines')
    ax.plot(x, y, z, 'bo', markersize=20, label='markers')
    # Scatter plot
    #ax.scatter(x, y, z, c='blue', marker='o',s = 500, label='Points')

    # Set labels for each axis
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Show the
    #  plot
    plt.show()
       
def show_movement(th, c, theta, steps):
    x, y, z = find_joints(theta, 0)

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Connect the points with lines
    ax.plot(x, y, z, c='gray', linewidth=20, label='Lines')
    
    # Scatter plot
    sc = ax.scatter(x, y, z, c='blue', marker='o',s = 500, label='Points')

    # Set labels for each axis
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    dx, dy, dz = find_dstep(th, c, x,y,z, steps)
    print(dx)

    animation = FuncAnimation(
        fig, update_linear, frames=range(100), fargs=(sc, x, y, z, dx, dy, dz)
    )
    # Show the
    #  plot
    plt.show()
# ************************************************** 
# main
# **************************************************

goal_1 = mat([[-0.5, 0.5, 0.5, 0.5],
        [0.5, -0.5, 0.5, 0.5],
        [0.5, 0.5, -0.5, 0.5],
        [0.0, 0.0, 0.0, 1.0]])


goal_2 = ([
    [-0.13909,	-0.85517,	-0.49934,	0.17395],
    [-0.98966,	0.13781,	0.03966,	0.63772],
    [0.03490,	0.49970,	-0.86550,	0.51277],
    [0.00000,	0.00000,	0.00000,	1.00000]
])

g1 = rpy(0,0,0)
g1 = pos(g1, 0.5, 0.5, 0.5)

goal = goal_2


print("goal =")
print(goal)

th = invKine(goal)
print("IK theta = ")
print(th)
c = 0
print("theta[c] = ")
print(th[:,c] * 180/pi)

T_06 = HTrans(th, c)
print("end positon = ")
print(T_06)

print("error = ")
print(T_06 - goal)

#show(th, c)

theta = mat([[0],[0],[0],[0],[0],[0]])
show(th, c)
