# -*- encoding: utf8
from numpy import *
from numpy import linalg as la
import pylab

def rotx(alpha):
	c, s = cos(alpha), sin(alpha)
	return mat([[1, 0, 0], [0, c, -s], [0, s, c]])

def roty(alpha):
	c, s = cos(alpha), sin(alpha)
	return mat([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotz(alpha):
	c, s = cos(alpha), sin(alpha)
	return mat([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def solve(theta_cam, x, y):
	theta_rob = pi/2
	theta_cam = -theta_cam
	f = 0.0037 / 0.000007 # 7 um = pixel size on image sensor

	K = mat([[f, 0, 320], [0, f, 240], [0, 0, 1]])
	Ki = la.inv(K)

	P_cam = mat([[0], [0.72], [-0.27]])
	P_rob = mat([[42], [0], [3.14]])
	R_cam = rotx(theta_cam)
	R_rob = roty(-theta_rob)
	#M = K*concatenate((R_cam, P_cam), 1)
	#M = M / M[2, 3]
	#print "hihii",M

	p = mat([[x], [y], [1]])
	print "p",p

	v = R_rob * R_cam * Ki * p
	print "v",v

	l = -P_cam[1, 0] / v[1, 0]
	print "l",l

	P0 = P_rob + R_rob * P_cam
	print "P0", P0

	P = P0 + l * v
	print "P",P
	print

	return P
def draw(theta_cam):
	P0 = solve(theta_cam, 0, 0)
	P1 = solve(theta_cam, 639, 0)
	P2 = solve(theta_cam, 639, 479)
	P3 = solve(theta_cam, 0, 479)

	pylab.plot([P0[0, 0], P1[0, 0], P2[0, 0], P3[0, 0], P0[0, 0]], [P0[2, 0], P1[2, 0], P2[2, 0], P3[2, 0], P0[2, 0]], label=str(theta_cam))

pylab.figure()
pylab.subplot(111, aspect="equal")
pylab.hold(True)
#pylab.plot([-.40, .40, .95, -.95, -.40], [-.45, -.45, -1.8, -1.8, -0.45], label="set 0.8")
#pylab.plot([-.37, .37, .50, -.50, -.37], [-.17, -.17, -.85, -.85, -.17], label="set 1.25")
draw(0.8 + 0.07)
draw(1.25 + 0.07)
#draw(1.0)
#draw(1.25)
#draw(pi/2)
pylab.legend()
pylab.savefig('camera.png')
pylab.show()
