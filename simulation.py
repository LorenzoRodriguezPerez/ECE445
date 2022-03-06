import math

def control(phi,wind_direction):
    """input: actual phi angle and wind direction
    output: alpha and beta angles needed to keep phi within the tolerance"""
    return (0,0)

def force_x(Fs,Fr,alpha,beta):
    """input: forces in sail and rudder, alpha and beta angles
    output: force in x axis"""
    return Fs*abs(math.sin(alpha)) - Fr*abs(math.sin(beta))

def force_y(Fs,Fr,alpha,beta):
    """input: forces in sail and rudder, alpha and beta angles
    output: force in y axis"""
    if (alpha >= 0, beta >= 0):
        F = Fs*abs(math.cos(alpha)) - Fr*abs(math.cos(beta))
    elif (alpha <= 0, beta >= 0):
        F = -Fs*abs(math.cos(alpha)) - Fr*abs(math.cos(beta))
    elif (alpha <= 0, beta <= 0):
        F = -Fs*abs(math.cos(alpha)) + Fr*abs(math.cos(beta))
    elif (alpha >= 0, beta <= 0):
        F = Fs*abs(math.cos(alpha)) + Fr*abs(math.cos(beta))
    return F

def Torque(Fs,Fr,alpha,beta,xgs,ygs,xgr,ygr):
    """input: forces in sail and rudder, alpha and beta angles, distances between the point of application of the forces and the center of mass
    output: torque in z axis"""
    if (alpha >= 0, beta >= 0):
        T = Fs*(abs(cos(alpha)*xgs)+abs(sin(alpha)*ygs)) + Fr*(-abs(cos(beta)*xgr)-abs(sin(beta)*ygr))
    elif (alpha <= 0, beta >= 0):
        T = Fs * (-abs(cos(alpha) * xgs) - abs(sin(alpha) * ygs)) + Fr * (-abs(cos(beta) * xgr) - abs(sin(beta) * ygr))
    elif (alpha <= 0, beta <= 0):
        T = Fs * (-abs(cos(alpha) * xgs) - abs(sin(alpha) * ygs)) + Fr * (+abs(cos(beta) * xgr) + abs(sin(beta) * ygr))
    elif (alpha >= 0, beta <= 0):
        T = Fs * (abs(cos(alpha) * xgs) + abs(sin(alpha) * ygs)) + Fr * (-abs(cos(beta) * xgr) - abs(sin(beta) * ygr))
    return T

M = 1 #estimate, moment of inertia (Kg*m^2)
m = 1 #estimate, mass of the boat (Kg)
L = 1 #estimate, sail length (m)
l = 1 #estimate, length between rudder and center of mass (m)
r = 1 #estimate, rudder length (m)
Fs = 1 #estimate, force in the sail (N)
Fr = 1 #estimate, for in the rudder (N)
beta = 0 #rudder angle (rad)
alpha = 0 #sail angle (rad)
x = [] #array containing x position in the different time periods (m)
y = [] #array containing y position in the different time periods (m)
vx = [] #array containing x velocity in the different time periods (m/s)
vy = [] #array containing y velocity in the different time periods (m/s)
phi = [] #array containing compass deviation angle in the different time periods (rad)
w = [] #array containing phi derivative in the different time periods (rad/s)
wind_direction = 0 #wind direction relative to the boat heading (rad)
dt = 1e-3 #time period used in discretized simulation (s)
t = [0] #array containing the different times [dt,2*dt,3*dt,...,T] (s)
T = 3 #simulation end time
int_fx = 0 #current x force integral value (N*s)
int_fy = 0 #current y force integral value (N*s)
int_t = 0 #current torque integral value (N*m*s)
int_fx_1 = 0 #previous x force integral value (N*s)
int_fy_1 = 0 #previous y force integral value (N*s)
int_t_1 = 0 #previous torque integral value (N*m*s)
int_vx = 0 #x velocity integral value (m)
int_vy = 0 #y velocity integral value (m)
int_w = 0 #w angular velocity integral value (rad)
int_vx_1 = 0 #previous x velocity integral value (m)
int_vy_1 = 0 #previous y velocity integral value (m)
int_w_1 = 0 #previous w integral value (m)

for i in range(int(T/dt)):
    # alpha and beta angles are calculated to keep phi aproximately 0
    (alpha,beta) = control(phi[i],wind_direction)

    #distances to the center or mass are recalculated according to the changes in alpha and beta
    xgs = (L / 2) * abs(math.cos(alpha))
    ygs = (l / 2) * abs(math.sin(alpha))
    xgr = l + (r / 2) * abs(math.cos(beta))
    ygr = (r / 2) * abs(math.sin(beta))

    #forces and torque are calculated
    forcex = force_x(Fs,Fr,alpha,beta)
    forcey = force_y(Fs,Fr,alpha,beta)
    torque = Torque(Fs,Fr,alpha,beta,xgs,ygs,xgr,ygr)

    #forces and torque integrals over time are estimated
    int_fx = intfx_1 + force_x*dt
    int_fy = intfy_1 + force_y*dt
    int_t = int_t_1 + torque*dt

    #velocities are calculated
    vx.append(int_fx/m)
    vy.append(int_fy/m)
    w.append(int_t/M)

    #velocities integrals over time are estimated
    int_vx = int_vx_1 + vx*dt
    int_vy = int_vy_1 + vy*dt
    int_w = int_w_1 + w*dt

    #positions and angle are calculated
    x.append(int_vx)
    y.append(int_vy)
    phi.append(int_w)

    #current values of integrals are stored for next iteration
    int_fx_1 = int_fx
    int_fy_1 = int_fy
    int_t_1 = int_t
    int_vx_1 = int_vx
    int_vy_1 = int_vy
    int_w_1 = int_w