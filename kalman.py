import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
from datetime import time
import os
import matplotlib
from celluloid import Camera

def to_polar(X,Y,Z):
    a = 6378137.000
    b = 6356752.3141
    c = a**2 / b
    e1 = np.sqrt((a**2 - b**2) / a**2)
    e2 = np.sqrt((a**2 - b**2) / b**2)
    p = np.sqrt(X**2 + Y**2)
    lam = np.arctan2(Y , X)
    beta = np.arctan2(a * Z , (b*p))
    fi = np.arctan2((Z+e2**2 * b *np.sin(beta)**3),(p-e1**2 * a * np.cos(beta)**3))
    N = c / np.sqrt(1 + e2**2 * np.cos(fi)**2)
    h = p / np.cos(fi) - N
    return fi,lam,h

def to_SagYukarı(fi,lam):
    a = 6378137.000
    b = 6356752.3141
    c = a**2 / b
    e1 = np.sqrt((a**2 - b**2) / a**2)
    e2 = np.sqrt((a**2 - b**2) / b**2)
    N = c / np.sqrt(1 + e2**2 * np.cos(fi)**2)
    der = 27 *np.pi/ 180
    A0 = 1- e1**2 /4 - 3*e1**4 /64 -5*e1**6 /256 -175*e1**8 /16384
    A2 = 3/8 *(e1**2 +e1**4 /4+ 15*e1**6 /128- 455*e1**8 /4096)
    A4 = 15/256*(e1**4 +3*e1**6 /4 - 77*e1**8/128)
    A6 = 35/3072*(e1**6 -41*e1**8 /32)
    A8 = (-315)/131072 *e1**8
    Sfi = a*(A0*fi-A2*np.sin(2*fi) + A4* np.sin(4*fi) - A6 *np.sin(6*fi) + A8 *np.sin(8*fi))
    lam = lam - der
    u = lam*np.cos(fi)
    t = np.tan(fi)
    n_squared = (e2**2)*np.cos(fi)**2
    x = N*(u+u**3 /6*(1-t**2 +n_squared)+ u**5 /120 *(5- 18*t**2 + t**4 +14*n_squared - 58*t**2
           * n_squared+ 13*n_squared**2 +4 *n_squared**3 -64 *n_squared**2 *t**2 -24*n_squared**3
           *t**2)+ u**7 /5040 *(61- 479*t**2 +179*t**4 -t**6))
    y = N*(Sfi/N+lam**2 *np.sin(fi)* np.cos(fi)/ 2+ lam**4 *np.sin(fi)* np.cos(fi)**3 / 24 *
           (5-t**2 +9*n_squared +4*n_squared**2)+lam**6 *np.sin(fi)* np.cos(fi)**5 /720 *(61-
           58 *t**2 +t**4 +270*n_squared +445*n_squared**2 +324*n_squared**3- 680*n_squared**2 *
           t**2 +88*n_squared**4 -600*n_squared**4 -600*n_squared**3 *t**2 -192*n_squared**4 *t**2)+
           lam**8 *np.sin(fi)* np.cos(fi)**7 /40320 *(1385- 311*t**2 +543*t**4 -t**6))

    conv = lam*np.sin(fi)*(1+ lam**2 *np.cos(fi)**2 /3 *(1+3*n_squared+2*n_squared**2)+lam**4 * np.cos(fi)**4 /15* (2-t**2))

    yukari = y*0.9996
    saga = x*0.9996 + 500000
    return saga,yukari
def grafnav_read(fname):
    f = open(fname,"r")
    data = f.readlines()
    f.close()
    for i in range(len(data)):
        if len(data[i].split()) != 0  and data[i].split()[0] == "(HMS)":
            break
        else:
            continue

    del data[:i+1]
    easting = []
    northing = []
    height = []
    epoch = []

    for i in range(len(data)):
        if len(data[i].split()) == 0:
            break
        else:
            epoch.append(datetime.strptime(("2021-10-26"+" "+str(data[i].split()[0])),"%Y-%m-%d %H:%M:%S.%f"))
            height.append(float(data[i].split()[3]))
            easting.append(float(data[i].split()[1]))
            northing.append(float(data[i].split()[2]))
    return epoch, easting, northing, height

def output_read(fname):
    # PPP-WIZARD output reader (verbose on)
    f = open(fname,"r")
    data = f.readlines()
    f.close()

    easting = []
    northing = []
    height = []
    X_list = []
    Y_list = []
    Z_list = []
    XS_list = []
    YS_list = []
    ZS_list = []
    epoch = []

    for i in range(len(data)):
        epoch.append(datetime.strptime(("20"+str(data[i].split()[0])+" "+str(data[i].split()[1])),"%Y-%m-%d %H:%M:%S.%f"))
        X = float(data[i].split()[12])
        Y = float(data[i].split()[15])
        Z = float(data[i].split()[18])
        X_list.append(X)
        Y_list.append(Y)
        Z_list.append(Z)
        XS_list.append(float(data[i].split()[14]))
        YS_list.append(float(data[i].split()[17]))
        ZS_list.append(float(data[i].split()[20]))
        fi,lam,h = to_polar(X,Y,Z)
        S,Y = to_SagYukarı(fi,lam)
        height.append(h)
        easting.append(S)
        northing.append(Y)
    return epoch, easting, northing, height, X_list, Y_list, Z_list,XS_list,YS_list,ZS_list

def Sdisp2vel(data,dt = 1):
    vel = []
    i = 1
    while i < (len(data)):
        vel.append((data[i-1]**2 + data[i]**2)**0.5 /dt)
        i+=1
    return vel


def disp2vel(data,dt = 1):
    vel = []
    i = 1
    while i < (len(data)):
        vel.append((data[i] - data[i-1])/dt)
        i+=1
    return vel

def vel2acc(data,dt = 1):
    acc = []
    i = 1
    while i < (len(data)):
        acc.append((data[i]-data[i-1])/dt)
        i+=1
    return acc
def graph_E_N2(var1,var2,Svar1,Svar2,Tvar1,Tvar2,fname):
    var1 = np.array(var1)
    var2 = np.array(var2)
    Svar1 = np.array(Svar1)
    Svar2 = np.array(Svar2)
    Tvar1 = np.array(Tvar1)
    Tvar2 = np.array(Tvar2)

    #matplotlib.use('Agg')
    fig = plt.figure(figsize=(10,7))
    plt.rcParams.update({'font.size': 9})

    plt.grid(True, linestyle='--',linewidth=0.2)
    plt.title("2D Scatter Plot",  fontsize=10)
    plt.ylabel("Northing (\u0394X) (cm)",  fontsize=9)
    plt.xlabel("Easting (\u0394Y) (cm)",  fontsize=9)

    xax = plt.gca().axes.get_xaxis()
    xax = xax.set_visible(False)

    # set visibility of y-axis as False
    yax = plt.gca().axes.get_yaxis()
    yax = yax.set_visible(False)
    for i in range(len(var1)):
        plt.scatter(var2[:i],var1[:i],c='b',marker="o",s = 2,label="rover")
        plt.scatter(Svar2[:i],Svar1[:i],c='r',marker="o", s = 4,label="filtered rover")
        plt.scatter(Tvar2[:i],Tvar1[:i],c='g',marker="o", s = 2,label="Predicted rover")

        plt.pause(0.001)
    plt.show()

def graph_E_N(var1,var2,Svar1,Svar2,Tvar1,Tvar2,Fvar1,Fvar2,fname):
    var1 = np.array(var1)
    var2 = np.array(var2)
    Svar1 = np.array(Svar1)
    Svar2 = np.array(Svar2)
    Tvar1 = np.array(Tvar1)
    Tvar2 = np.array(Tvar2)
    Fvar1 = np.array(Fvar1)
    Fvar2 = np.array(Fvar2)

    fig = plt.figure(figsize=(20,15))
    plt.plot(var2,var1,color='b',marker="o", linestyle="None",markersize = 2,label="rover")
    plt.plot(Svar2,Svar1,color='r',marker="o", linestyle="None", markersize = 4,label="filtered rover")
    plt.plot(Tvar2,Tvar1,color='g',marker="o", linestyle="None",markersize = 2,label="Predicted rover")
    plt.plot(Fvar2,Fvar1,color='black',marker="o", linestyle="None",markersize = 2,label="Fixed rover")
    plt.grid(True, linestyle='--',linewidth=0.2)
    plt.legend(loc=1, handlelength=1,  fontsize=12,markerscale=1)
    plt.title("2D Scatter Plot",  fontsize=12)
    plt.ylabel("Northing (\u0394X) (cm)",  fontsize=12)
    plt.xlabel("Easting (\u0394Y) (cm)",  fontsize=12)
    xax = plt.gca().axes.get_xaxis()
    xax = xax.set_visible(False)
    yax = plt.gca().axes.get_yaxis()
    yax = yax.set_visible(False)
    plt.tight_layout()
    plt.savefig("kalman2d.png")

def kalman(Xmatrx, obs_err, init, pro_err,aX):
    i           = int(np.size(Xmatrx)/len(Xmatrx))

    A           = np.identity(i)
    pc = 0
    while pc < i:
        A[pc][pc+1] = init[1]
        pc += 2

    B           = np.identity(i)
    pc = 0
    while pc < i:
        B[pc][pc] = 0.5 * init[1]**2
        B[pc+1][pc+1] = init[1]
        pc += 2

    H           =  np.identity(i)
    for pc in range(i):
        if pc % 2 != 0:
            H[pc][pc] = 0
    C           = np.identity(i)
    I           = np.identity(i)

    X_kp        = np.ndarray((i,1), dtype = float)
    X_k_1       = np.array(Xmatrx[0]).reshape((-1,1))
    P_k_1       = np.ndarray((i,i), dtype = float)
    for k in range(i):
        P_k_1[k][k] = pro_err[k]**2

    R           = np.ndarray((i,i), dtype = float)

    x_k         = np.ndarray((len(Xmatrx)-1,3), dtype = float)
    x_p         = np.ndarray((len(Xmatrx)-1,3), dtype = float)
    x_m         = np.ndarray((len(Xmatrx)-1,3), dtype = float)

    k, j        = 0, 1
    while j < len(Xmatrx):
        for pc in range(i):
            R[pc][pc]           = obs_err[k][pc]**2
        X_kp            = np.dot(A, X_k_1) + np.dot(B, np.array([aX[k]]).T)
        P_kp            = (np.dot(np.dot(A, P_k_1), np.transpose(A)))
        for pc in range(i):
            for pk in range(i):
                if pc !=pk:
                    P_kp[pc][pk] = 0


        KG          = np.divide(np.dot(P_kp, H), (np.dot(np.dot(H, P_kp), np.transpose(H)) + R))
        for pc in range(i):
            for pk in range(i):
                if pc !=pk:
                    KG[pc][pk] = 0

        Y_k         = np.dot(H, Xmatrx[j].reshape((-1, 1)))
        X_k         = X_kp + np.dot(KG, (Y_k - np.dot(H, X_kp)))
        P_k         = np.dot((I - np.dot(KG, H)), P_kp)

        X_k_1       = X_k
        P_k_1       = P_k
        x_k[k]      = X_k[0][0],X_k[2][0],X_k[4][0]
        x_p[k]      = X_kp[0][0],X_kp[2][0],X_kp[4][0]
        x_m[k]      = Xmatrx[j][0],Xmatrx[j][2],Xmatrx[j][4]

        j += 1
        k += 1
    return x_m, x_k, x_p
epoch, easting, northing, height, X_list, Y_list, Z_list,XS_list,YS_list,ZS_list = output_read("output_lowlevel_itu_kampus_RAIM")

np.seterr(divide = 'ignore', invalid = 'ignore')

vX = disp2vel(X_list)
aX = disp2vel(vX)
SvX = Sdisp2vel(XS_list)
SaX = Sdisp2vel(SvX)
SaX.append(0)
aX.append(0)
del X_list[-1]
del XS_list[-1]

vY = disp2vel(Y_list)
aY = disp2vel(vY)
SvY = Sdisp2vel(YS_list)
SaY = Sdisp2vel(SvY)
SaY.append(0)
aY.append(0)
del Y_list[-1]
del YS_list[-1]

vZ = disp2vel(Z_list)
aZ = disp2vel(vZ)
SvZ = Sdisp2vel(ZS_list)
SaZ = Sdisp2vel(SvZ)
SaZ.append(0)
aZ.append(0)
del Z_list[-1]
del ZS_list[-1]

X               = np.array((list(zip(X_list, vX, Y_list, vY ,Z_list, vZ))))
obs_err         = np.array((list(zip(XS_list, SvX,YS_list, SvY, ZS_list, SvZ))), dtype = float)
pro_err         = np.array(([np.mean(XS_list), np.mean(SvX), np.mean(YS_list), np.mean(SvY), np.mean(ZS_list), np.mean(SvZ)]), dtype = float)
init            = np.array(([10],[1],[0]), dtype = float)
A               = np.array((list(zip(aX, aX, aY, aY ,aZ, aZ))))
x_m, x_k, x_p   = kalman(X, obs_err, init, pro_err,A)

height = []
northing = []
easting = []
height2 = []
northing2 = []
easting2 = []
height3 = []
northing3 = []
easting3 = []

for i in range(len(x_m)):

    fi,lam,h = to_polar(x_m[i][0],x_m[i][1],x_m[i][2])
    S,Y = to_SagYukarı(fi,lam)
    height.append(h)
    easting.append(S)
    northing.append(Y)

for i in range(len(x_k)):

    fi,lam,h = to_polar(x_k[i][0],x_k[i][1],x_k[i][2])
    S,Y = to_SagYukarı(fi,lam)
    height2.append(h)
    easting2.append(S)
    northing2.append(Y)

for i in range(len(x_p)):
    fi,lam,h = to_polar(x_p[i][0],x_p[i][1],x_p[i][2])
    S,Y = to_SagYukarı(fi,lam)
    height3.append(h)
    easting3.append(S)
    northing3.append(Y)

epoch4, easting4, northing4, height4 = grafnav_read("grafnav-brodcastorbitilePPK-chc-rt-ppp-rinex304v2.txt")

graph_E_N(northing,easting,northing2,easting2,northing3,easting3,northing4,easting4,"kalman")
fig = plt.figure()
ax = plt.axes(projection ='3d')

ax.plot3D(northing3,easting3, height3, 'green')
plt.show()
ax.plot3D(northing4,easting4, height4, 'green')
plt.show()
