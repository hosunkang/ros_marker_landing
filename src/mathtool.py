#!/usr/bin/python
import math
import numpy as np

class mathtool:
    def __init__(self):
        self.Rxpi = self.rotationMatrix(math.pi, 'x')
        self.Rypi = self.rotationMatrix(math.pi, 'y')

    def ra2de(self,ra):
        return ra*180/math.pi

    def de2ra(self,de):
        return de*math.pi/180

    def Rmat2euler_yxz(self,rmat):
        [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]] = rmat
        if r23 < 1:
            if r23 > -1:
                x = math.asin(-r23)
                y = math.atan2(r13,r33)
                z = math.atan2(r21,r22)
            else:
                x = math.pi/2
                y = -math.atan2(-r12,r11)
                z = 0
        else:
            x = -math.pi/2
            y = math.atan2(-r12,r11)
            z = 0
        return x,y,z

    def Rmat2euler_zyx(self,rmat):
        [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]] = rmat
        if r31 < 1:
            if r31 > -1:
                x = math.atan2(r32,r33)
                y = math.asin(-r31)
                z = math.atan2(r21,r11)
            else:
                x = 0
                y = math.pi/2
                z = -math.atan2(-r23,r22)
        else:
            x = 0
            y = -math.pi/2
            z = math.atan2(-r23,r22)
        return self.ra2de(x), self.ra2de(y), self.ra2de(z)    

    def rotationMatrix(self, angle, flag):
        if flag == 'x':
            mat = np.array([[1,0,0],
                            [0,round(math.cos(angle),3),-round(math.sin(angle),3)],
                            [0,round(math.sin(angle),3),round(math.cos(angle),3)]])
        elif flag == 'y':
            mat = np.array([[round(math.cos(angle),3),0,round(math.sin(angle),3)],
                            [0,1,0],
                            [-round(math.sin(angle),3),0,round(math.cos(angle),3)]])
        elif flag == 'z':
            mat = np.array([[round(math.cos(angle),3),-round(math.sin(angle),3),0],
                            [round(math.sin(angle),3),round(math.cos(angle),3),0],
                            [0,0,1]])
        return mat

    def getleg(self, Rcm,imu):
        imu = [self.de2ra(imu[0]), self.de2ra(-imu[1]), self.de2ra(-imu[2])]
        Rlb = np.dot(np.dot(self.Rxpi,self.rotationMatrix(imu[1], 'y')), self.rotationMatrix(imu[0],'x'))
        Rbc = self.Rypi
        Rlm = np.dot(np.dot(Rlb,  Rbc), Rcm)
        a,b,c = self.Rmat2euler_yxz(Rlm)
        #Rlt1 = np.dot(np.dot(rotationMatrix(b, 'y'), rotationMatrix(a, 'x')), Rxpi)
        Rlt1 = np.dot(np.dot(Rlm, self.rotationMatrix(-c, 'z')), self.Rxpi)
        Rlt2 = np.dot(Rlt1, self.rotationMatrix(math.pi,'z'))

        return self.Rmat2euler_zyx(Rlt1)

    def mean(self, datas):
        return sum(datas)/len(datas)

    def lpf(self, ts, pre_y, x, tau=0.1):
        y = (tau * pre_y + ts * x) / (tau + ts)
        return y
    
    def imulpf(self, ts, pre_imu, imu, tau=0.1):
        prev_x, prev_y, prev_z = pre_imu
        x, y, z = imu
        new_x = round(self.lpf(ts, prev_x, x, tau),2)
        new_y = round(self.lpf(ts, prev_y, y, tau),2)
        new_z = round(self.lpf(ts, prev_z, z, tau),2)
        return [new_x, new_y, new_z]