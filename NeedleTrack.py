#!/usr/bin/env python3.7

import kivy
kivy.require('1.11.1')

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.graphics import *
from math import tan, pi, atan, asin

import symbol
import string
from numpy import linalg
from sympy import *
from numpy.linalg import norm
import numpy as np
import math

from kivy.properties import NumericProperty, ObjectProperty, ListProperty

from kivy.config import Config
Config.set('graphics', 'resizable', '0')
Config.set('graphics', 'width', '1920')
Config.set('graphics', 'height', '1080')

import time, json, sys, serial

#"#Tools,ToolInfo,Frame#,PortHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,Markers,State,Tx,Ty,Tz"
# 21.305      -3.568      -2.408     169.623       3.359    -129.423

class Tracker(BoxLayout):
    
    try:
        afile = open('Settings.json', 'r')
        cfg = json.load(afile)
        afile.close()
    except:
        print('Impossible to load Settings.json file!')
        sys.exit(0)

    # EM tracker
    origin_needle = [0, 0, 0]
    needle_pos = [0, 0, 0]     # x y z
    needle_rot = [0, 0, 0]   # yaw pitch roll in degree

    target_pos = [0, 0, 0]  # x y z

    # Cobot
    cobot_origin_needle = [0, 0, 0]
    cobot_needle_pos = [0, 0, 0]     # x y z
    cobot_needle_rot = [0, 0, 0]   # yaw pitch roll in degree

    data_prob_scaling = cfg['probe']['data_prob_scaling'] # For small movement within the prostate with the needle
    scaling = cfg['view']['scaling'] # View

    image_width = 1920*scaling
    image_height = 1661*scaling
    
    origin_coronal_view = (900*scaling, 1400*scaling)
    offset_coronal_view = (330*scaling, 330*scaling)
    origin_sagital_view = (2880*scaling, 1200*scaling)
    offset_sagital_view = (465*scaling, 200*scaling)
    size_view = (1920*scaling, 1661*scaling)

    needle_size = cfg['view']['needle_size']
    needle_line_width = cfg['view']['needle_line_width']

    track_file_pathname = cfg['probe']['track_file_pathname']
    track_old_ID = -1
    track_enable = False
    cobot_track_enable = False

    target_size = cfg['view']['target_size']
    target_on = False

    refresh_time = cfg['probe']['refresh_time']     

    # Graphics
    CoronalAxisH = ListProperty([0, origin_coronal_view[1], size_view[0], origin_coronal_view[1]])
    CoronalAxisV = ListProperty([origin_coronal_view[0], offset_coronal_view[0], origin_coronal_view[0], size_view[1]+offset_coronal_view[1]])
    SagitalAxisH = ListProperty([size_view[0], origin_sagital_view[1], 2*size_view[0], origin_sagital_view[1]])
    SagitalAxisV = ListProperty([origin_sagital_view[0], offset_sagital_view[0], origin_sagital_view[0], size_view[1]+offset_sagital_view[1]])

    CoronalTargetCircle = ListProperty([0, 0, target_size])
    CoronalTargetLine1 = ListProperty([0, 0, 0, 0])
    CoronalTargetLine2 = ListProperty([0, 0, 0, 0])
    SagitalTargetCircle = ListProperty([0, 0, target_size])
    SagitalTargetLine1 = ListProperty([0, 0, 0, 0])
    SagitalTargetLine2 = ListProperty([0, 0, 0, 0])
    TargetOpacity = NumericProperty(0)

    # Needle from EM tracker
    CoronalNeedlePath = ListProperty([0, 0, 0, 0])
    CoronalNeedlePos = ListProperty([0, 0, 0, 0])

    SagitalNeedlePath = ListProperty([0, 0, 0, 0])
    SagitalNeedlePos = ListProperty([0, 0, 0, 0])
    NeedleOpacity = NumericProperty(0)

    # Needle from cobot
    cobot_CoronalNeedlePath = ListProperty([0, 0, 0, 0])
    cobot_CoronalNeedlePos = ListProperty([0, 0, 0, 0])

    cobot_SagitalNeedlePath = ListProperty([0, 0, 0, 0])
    cobot_SagitalNeedlePos = ListProperty([0, 0, 0, 0])
    cobot_NeedleOpacity = NumericProperty(0)

    ########## COBOT ###################################

    # fonction MGI
    def cobot_MGI(self, q):
        x = q[0]
        y = q[1]
        z = q[2]
        alfa = q[3]
        phi = q[4]
        psy = q[5]

        l2 = 0.1375
        l1 = 0.12
        R = 0.0300
        b = 0.03
        r = 0.0900
        tetaa = np.pi / 6
        tetac = np.pi / 3
        tetad = 0
        lamda1 = np.array([-tetac/2, tetac/2, ((2*np.pi)/3)-(tetac/2), ((2*np.pi)/3)+(tetac/2), -((2*np.pi)/3)-(tetac/2) ,-((2 * np.pi)/3)+(tetac /2)])
        lamda2 = np.array([-tetaa /2, tetaa / 2, ((2 * 	np.pi) / 3) - (tetaa / 2), ((2 * np.pi) / 3) + (tetaa / 2), - ((2 * np.pi) / 3) - (tetaa / 2) ,- ((2 * np.pi) / 3) + (tetaa / 2)])
        lamda3 = np.array([-tetad, tetad,((2 *np.pi) / 3) - (tetad), ((2 * np.pi) / 3) + (tetad), ((4 * np.pi) / 3) - (tetad), ((4 * np.pi) / 3) + (tetad)])
        ROT = np.array([[np.cos(phi) * np.cos(psy), np.cos(phi)*np.sin(psy) ,- np.sin(phi)],[ np.cos(psy) * np.sin(alfa) * np.sin(phi) - np.cos(alfa) * np.sin(psy), np.cos(alfa) * np.cos(psy) + np.sin(alfa) * np.sin(phi) * np.sin(psy), np.cos(phi) * np.sin(alfa)],[np.sin(alfa) * np.sin(psy) + np.cos(alfa) * np.cos(psy) * np.sin(phi),np.cos(alfa) * np.sin(phi) * np.sin(psy) - np.cos(psy) * np.sin(alfa), np.cos(alfa) * np.cos(phi)]])
        Xt = np.array([[x], [y], [z]])
        a=R*np.cos(lamda1)
        b=R*np.sin(lamda1)
        c=np.zeros((1,6))
        CM=np.vstack((a,b,c))
        H = np.dot(ROT,CM)
        CF= np.tile(Xt,6)
        CL=np.sum((CF,H), axis=0)

        Cx = CL[0,:]
        Cy = CL[1,:]
        Cz = CL[2,:]

        U=[]
        V=[]
        W=[]
        teta=[]

        for i in range(6):
            U.append(Cx[i]* np.cos(lamda3[i])+Cy[i]* np.sin(lamda3[i])-r*np.cos(lamda3[i]-lamda2[i]))
            V.append(Cz[i])
            W.append((((Cx[i]-r*np.cos(lamda2[i]))**2 + (Cy[i]-r*np.sin(lamda2[i]))**2 + Cz[i]**2 + l1**2 - l2**2)/(2*l1)))

        for i in range(6):
            teta.append((2*np.arctan((V[i]-np.sqrt(V[i]**2-W[i]**2+U[i]**2))/(U[i]+W[i])))) 

        return(teta)

    # fonction jacobienne
    def cobot_Jacob(self, teta, q):
        x = q[0]
        y = q[1]
        z = q[2]
        alfa = q[3]
        phi = q[4]
        psy = q[5]

        R=0.03
        r=0.09
        l2=0.1375
        l1=0.12
        tetac = np.pi / 3
        tetaa = np.pi / 6
        tetad = 0

        lamda1 = np.array([-tetac/2, tetac/2, ((2*	np.pi) /3) - (tetac/2), ((2*np.pi)/3) + (tetac/2), - ((2*np.pi)/3) - (tetac/2) ,- ((2 * np.pi)/3) + (tetac /2)])
        lamda2 = np.array([-tetaa /2, tetaa / 2, ((2 * 	np.pi) / 3) - (tetaa / 2), ((2 * np.pi) / 3) + (tetaa / 2), - ((2 * np.pi) / 3) - (tetaa / 2) ,- ((2 * np.pi) / 3) + (tetaa / 2)])
        lamda3 = np.array([-tetad, tetad,((2 *np.pi) / 3) - (tetad), ((2 * np.pi) / 3) + (tetad), ((4 * np.pi) / 3) - (tetad), ((4 * np.pi) / 3) + (tetad)])
        ROT = np.array([[np.cos(phi) * np.cos(psy), np.cos(phi) * np.sin(psy) ,- np.sin(phi)],[ np.cos(psy) * np.sin(alfa) * np.sin(phi) - np.cos(alfa) * np.sin(psy), np.cos(alfa) * np.cos(psy) + np.sin(alfa) * np.sin(phi) * np.sin(psy), np.cos(phi) * np.sin(alfa)],[ np.sin(alfa) * np.sin(psy) + np.cos(alfa) * np.cos(psy) * np.sin(phi),np.cos(alfa) * np.sin(phi) * np.sin(psy) - np.cos(psy) * np.sin(alfa), np.cos(alfa) * np.cos(phi)]])
        Xt = np.array([[x], [y], [z]])
        a=R * np.cos(lamda1)
        b=R * np.sin(lamda1)
        c=np.zeros((1,6))
        CM=np.vstack((a,b,c))
        H = np.dot(ROT,CM)
        A=np.vstack((r * np.cos(lamda2),r * np.sin(lamda2),np.zeros((1,6))))
        B=np.vstack((l1*np.cos(teta)*np.cos(lamda3),l1*np.cos(teta)*np.sin(lamda3),l1*np.sin(teta)))
        M=A + B
        H = np.dot(ROT,CM)
        CF= np.tile(Xt,6)
        CL=np.sum((CF,H), axis=0)
        S=CL - M

        L=[]

        for i in range(6):
            L.append(np.cross(H[:,i],S[:,i]))

        Jx=np.concatenate((S.T,L), axis=1)
        Jq= np.zeros((6,6))
        for i in range(6):
            for j in range(6):
                if i==j:
                    Jq[i,j]=l1*np.dot(S[:,i],np.array([[-np.sin(teta[i])*np.cos(lamda3[i])], [-np.sin(teta[i])*np.sin(lamda3[i])],[np.cos(teta[i])]]))

        Jq_in=np.linalg.inv(Jq)
        J = np.linalg.inv(np.dot(Jq_in,Jx))

        return(J)

    # fonction newtonraphson 
    def cobot_NewtonRaph(self,v,teta,no_itr,error):
        qa = self.cobot_MGI(v)
        F = np.array(teta) - np.array(qa)
        K = self.cobot_Jacob(qa,v)
        S = np.dot(K,F)
        v1 = v + S
        err = np.linalg.norm(F)
        v = v1
        i = 1
        while err > error:
            qa = self.cobot_MGI(v)
            F = np.array(teta) - np.array(qa)
            K = self.cobot_Jacob(qa, v)
            S = np.dot(K,F)
            v1 = v + S
            err = np.linalg.norm(F)
            v = v1

            if i > no_itr:
                v = v1
                break

            i += 1

        return(v,i,err)

    # def cobot_init_com(self):
    #     with serial.Serial(self.cfg['cobot']['COMID'], self.cfg['cobot']['baudrate'], timeout = 1) as ser:

        
    #     self.ser = serial.Serial()
    #     self.ser.baudrate = self.cfg['cobot']['baudrate']
    #     self.ser.port = self.cfg['cobot']['COMID']
    #     self.ser.timeout = 1

    #     self.ser.open()

    # def cobot_exit_com(self):
    #     self.ser.close()

    # Read data from cobot
    def get_data_cobot(self):
        Probe = {
            'id': -1,
            'flag': 'lost',
            'x': 0,
            'y': 0,
            'z': 0,
            'yaw': 0,
            'pitch': 0,
            'roll': 0
        }
         
        # Read joint angles 
        # if self.ser.is_open and self.ser.in_waiting:

        with serial.Serial(self.cfg['cobot']['COMID'], self.cfg['cobot']['baudrate'], timeout = 1) as ser:
            line = ser.readline()
            ser.close()
        print(line)
        # Ex. of packet
        # packet = b'12.3;45.6;78.9;1.2;34.5;67.8\n'
        raw = line.decode('utf-8', 'backslashreplace')
        raw = raw.strip().split(';')
        print(raw)
        angles = np.array(raw, 'float32')
        angles *= (pi / 180.0)  # in rad

        # TODO a revoir
        #if angles[0] == "" or angles[2] == "Missing":
        #    return Probe  # empty

        # Convert in 3D Cart. space
        v = np.array([0,0,0.06,0,0,0])
        data = self.cobot_NewtonRaph(v, angles, self.cfg['cobot']['model_iter'], self.cfg['cobot']['model_err'])

        #######

        Probe['id'] = 0
        Probe['x'] =  data[0] * 1e-03  # mm
        Probe['y'] =  data[1] * 1e-03  # mm
        Probe['z'] =  data[2] * 1e-03  # mm
        Probe['yaw'] = data[3] * 180.0 / pi    # deg
        Probe['pitch'] = data[4] * 180.0 / pi
        Probe['roll'] = data[5] * 180.0 / pi
        Probe['flag'] = 'Enable'
            
        return Probe

    # def get_data_cobot_DEBUG(self):
    #     Probe = {
    #         'id': 1,
    #         'flag': 'Enable',
    #         'x': 0,
    #         'y': 0,
    #         'z': 0,
    #         'yaw': 0,
    #         'pitch': 0,
    #         'roll': 0
    #     }

    #     return Probe

    ########## EM TRACKER ##############################

    # Read data from probe
    def get_data_probe(self):
        Probe = {
            'id': -1,
            'flag': 'lost',
            'x': 0,
            'y': 0,
            'z': 0,
            'yaw': 0,
            'pitch': 0,
            'roll': 0
        }
         
        f = open(self.track_file_pathname, "r")
        data = f.readline().split(",")
        f.close()

        if data[0] == "" or data[2] == "Missing":
            return Probe  # empty

        # Fill data
        xprob = float(data[8])
        yprob = float(data[9])
        zprob = float(data[10])

        yaw, pitch, roll = self.convert_quaternion_to_euler(float(data[4]), float(data[5]), float(data[6]), float(data[7]))

        if yaw >= 0:
            yaw = 90.0-yaw
        else:
            yaw = -90.0-yaw

        Probe['id'] = int(data[0])
        Probe['x'] =  zprob
        Probe['y'] = -xprob   # Not the same frame that the prob
        Probe['z'] =  yprob
        Probe['yaw'] = -yaw
        Probe['pitch'] = pitch
        Probe['roll'] = roll
        Probe['flag'] = 'Enable'
            
        return Probe

    # The first screen is at the pos 0
    def convert_to_coronal_frame(self, pos):
        gbl_pos = [0, 0]

        gbl_pos[0] = (pos[0]*self.data_prob_scaling) - (self.origin_needle[0]*self.data_prob_scaling) + self.origin_coronal_view[0]
        gbl_pos[1] = (pos[2]*self.data_prob_scaling) - (self.origin_needle[2]*self.data_prob_scaling) + self.origin_coronal_view[1]

        return gbl_pos
        
    # The second screen is at the pos width
    def convert_to_sagital_frame(self, pos):
        gbl_pos = [0, 0]

        #gbl_pos[0] = self.origin_sagital_view[0] - pos[2] - self.origin_needle[2]
        gbl_pos[0] = (self.origin_needle[2]*self.data_prob_scaling) + self.origin_sagital_view[0] - (pos[2]*self.data_prob_scaling)
        gbl_pos[1] = (pos[1]*self.data_prob_scaling) - (self.origin_needle[1]*self.data_prob_scaling) + self.origin_sagital_view[1]

        return gbl_pos    

    # The first screen is at the pos 0
    def convert_to_coronal_frame_cobot(self, pos):
        gbl_pos = [0, 0]

        gbl_pos[0] = (pos[0]*self.data_prob_scaling) - (self.cobot_origin_needle[0]*self.data_prob_scaling) + self.origin_coronal_view[0]
        gbl_pos[1] = (pos[2]*self.data_prob_scaling) - (self.cobot_origin_needle[2]*self.data_prob_scaling) + self.origin_coronal_view[1]

        return gbl_pos
        
    # The second screen is at the pos width
    def convert_to_sagital_frame_cobot(self, pos):
        gbl_pos = [0, 0]

        #gbl_pos[0] = self.origin_sagital_view[0] - pos[2] - self.origin_needle[2]
        gbl_pos[0] = (self.cobot_origin_needle[2]*self.data_prob_scaling) + self.origin_sagital_view[0] - (pos[2]*self.data_prob_scaling)
        gbl_pos[1] = (pos[1]*self.data_prob_scaling) - (self.cobot_origin_needle[1]*self.data_prob_scaling) + self.origin_sagital_view[1]

        return gbl_pos    

    # Quaternion to Euler
    def convert_quaternion_to_euler(self, q0, q1, q2, q3):
        u = 2*(q0*q1 + q2*q3)
        v = 1 - 2*(q1*q1 + q2*q2)
        Phi = 180.0 * atan(u / v) / pi

        u = 2*(q0*q2 - q3*q1)
        Theta = 180.0 * asin(u) / pi

        u = 2*(q0*q3 + q1*q2)
        v = 1 - 2*(q2*q2 + q3*q3)
        Psi = 180.0 * atan(u / v) / pi

        return (Phi, Theta, Psi)

    # Draw needle and needle path
    def update_needle(self):
        px, py = self.convert_to_coronal_frame(self.needle_pos)
        tan_yaw = tan(pi/180*self.needle_rot[0])
        dy1 = (py - self.offset_coronal_view[0]) * tan_yaw
        dy2 = (self.size_view[1] - py + self.offset_coronal_view[1]) * tan_yaw

        # Path
        self.CoronalNeedlePath[0] = px-dy1
        self.CoronalNeedlePath[1] = self.offset_coronal_view[0]
        self.CoronalNeedlePath[2] = px+dy2
        self.CoronalNeedlePath[3] = self.image_height+self.offset_coronal_view[1]

        # Needle
        self.CoronalNeedlePos[0] = px-dy1
        self.CoronalNeedlePos[1] = self.offset_coronal_view[0]
        self.CoronalNeedlePos[2] = px
        self.CoronalNeedlePos[3] = py

        px, py = self.convert_to_sagital_frame(self.needle_pos)
        tan_pitch = tan(pi/180*self.needle_rot[1])
        dy1 = (px-self.size_view[0]) * tan_pitch
        dy2 = (2*self.size_view[0] - px) * tan_pitch

        # Path
        self.SagitalNeedlePath[0] = self.size_view[0]
        self.SagitalNeedlePath[1] = py-dy1
        self.SagitalNeedlePath[2] = 2*self.size_view[0]
        self.SagitalNeedlePath[3] = py+dy2

        # Needle
        self.SagitalNeedlePos[0] = px
        self.SagitalNeedlePos[1] = py
        self.SagitalNeedlePos[2] = 2*self.size_view[0]
        self.SagitalNeedlePos[3] = py+dy2

    # Cobot - Draw needle and needle path
    def update_needle_cobot(self):
        px, py = self.convert_to_coronal_frame_cobot(self.cobot_needle_pos)
        tan_yaw = tan(pi/180*self.cobot_needle_rot[0])
        dy1 = (py - self.offset_coronal_view[0]) * tan_yaw
        dy2 = (self.size_view[1] - py + self.offset_coronal_view[1]) * tan_yaw

        # Path
        self.cobot_CoronalNeedlePath[0] = px-dy1
        self.cobot_CoronalNeedlePath[1] = self.offset_coronal_view[0]
        self.cobot_CoronalNeedlePath[2] = px+dy2
        self.cobot_CoronalNeedlePath[3] = self.image_height+self.offset_coronal_view[1]

        # Needle
        self.cobot_CoronalNeedlePos[0] = px-dy1
        self.cobot_CoronalNeedlePos[1] = self.offset_coronal_view[0]
        self.cobot_CoronalNeedlePos[2] = px
        self.cobot_CoronalNeedlePos[3] = py

        px, py = self.convert_to_sagital_frame_cobot(self.cobot_needle_pos)
        tan_pitch = tan(pi/180*self.cobot_needle_rot[1])
        dy1 = (px-self.size_view[0]) * tan_pitch
        dy2 = (2*self.size_view[0] - px) * tan_pitch

        # Path
        self.cobot_SagitalNeedlePath[0] = self.size_view[0]
        self.cobot_SagitalNeedlePath[1] = py-dy1
        self.cobot_SagitalNeedlePath[2] = 2*self.size_view[0]
        self.cobot_SagitalNeedlePath[3] = py+dy2

        # Needle
        self.cobot_SagitalNeedlePos[0] = px
        self.cobot_SagitalNeedlePos[1] = py
        self.cobot_SagitalNeedlePos[2] = 2*self.size_view[0]
        self.cobot_SagitalNeedlePos[3] = py+dy2
      
    # Update target
    def update_target(self):   
        px, py = self.convert_to_coronal_frame(self.target_pos)
        hsize = 0.5*self.needle_size

        self.CoronalTargetCircle[0] = px
        self.CoronalTargetCircle[1] = py
        
        self.CoronalTargetLine1[0] = px-hsize
        self.CoronalTargetLine1[1] = py
        self.CoronalTargetLine1[2] = px+hsize
        self.CoronalTargetLine1[3] = py
       
        self.CoronalTargetLine2[0] = px
        self.CoronalTargetLine2[1] = py-hsize
        self.CoronalTargetLine2[2] = px
        self.CoronalTargetLine2[3] = py+hsize

        px, py = self.convert_to_sagital_frame(self.target_pos)
            
        self.SagitalTargetCircle[0] = px
        self.SagitalTargetCircle[1] = py
        
        self.SagitalTargetLine1[0] = px-hsize
        self.SagitalTargetLine1[1] = py
        self.SagitalTargetLine1[2] = px+hsize
        self.SagitalTargetLine1[3] = py
       
        self.SagitalTargetLine2[0] = px
        self.SagitalTargetLine2[1] = py-hsize
        self.SagitalTargetLine2[2] = px
        self.SagitalTargetLine2[3] = py+hsize

        # Get distance error
        dx = self.needle_pos[0]-self.target_pos[0]
        dy = self.needle_pos[1]-self.target_pos[1]
        dz = self.needle_pos[2]-self.target_pos[2]
        DistErr = (dx*dx + dy*dy + dz*dz)**(0.5)
        self.ids['lbl_error'].text = 'error: %5.1f mm' % DistErr

        # If cobot
        if self.cobot_track_enable is True:
            dx = self.cobot_needle_pos[0]-self.target_pos[0]+self.origin_needle[0]
            dy = self.cobot_needle_pos[1]-self.target_pos[1]+self.origin_needle[1]
            dz = self.cobot_needle_pos[2]-self.target_pos[2]+self.origin_needle[2]
            DistErr = (dx*dx + dy*dy + dz*dz)**(0.5)
            self.ids['lbl_error_cobot'].text = 'error: %5.1f mm' % DistErr

    # Init tracking
    def init_tracking(self):
        # start clock
        Clock.schedule_interval(self.update_tracking, self.refresh_time)

    def update_tracking(self, dt):
        
        # EM Tracker
        if self.track_enable is True:
            
            # Read the probe
            data = self.get_data_probe()

            # check if the probe is not lost
            if data['flag'] == 'lost':
                # update GUI and do nothing
                self.ids['lbl_track'].text = '[color=ff0000]Lost...[/color]'
                return

            # update only if new data from the probe
            if data['id'] != self.track_old_ID:
                self.track_old_ID = data['id']

                self.needle_pos[0] = data['x']
                self.needle_pos[1] = data['y']
                self.needle_pos[2] = data['z']

                self.needle_rot[0] = data['yaw']
                self.needle_rot[1] = data['pitch']
                self.needle_rot[2] = data['roll']

                txt = 'pos %5.1f %5.1f %5.1f mm  -  rot %5.1f %5.1f %5.1f deg' % (data['x'], data['y'], data['z'], data['yaw'], data['pitch'], data['roll'])
                self.ids['lbl_track'].text = '[color=ffffff]%s[/color]' % txt

                # Update the view
                self.update_needle()

        # Cobot tracker
        if self.cobot_track_enable is True:
            # Read the cobot
            data = self.get_data_cobot()

            # Assign new values
            self.cobot_needle_pos[0] = data['x']
            self.cobot_needle_pos[1] = data['y']
            self.cobot_needle_pos[2] = data['z']

            self.cobot_needle_rot[0] = data['yaw']
            self.cobot_needle_rot[1] = data['pitch']
            self.cobot_needle_rot[2] = data['roll']

            txt = 'pos %5.1f %5.1f %5.1f mm  -  rot %5.1f %5.1f %5.1f deg' % (data['x'], data['y'], data['z'], data['yaw'], data['pitch'], data['roll'])
            self.ids['lbl_track_cobot'].text = '[color=ffffff]%s[/color]' % txt

            # Update the view
            self.update_needle_cobot()

        if self.target_on is True:
            self.update_target()

    ### Button action
    def cmd_start(self):
        self.ids.bt_stop.disabled = False
        self.ids.bt_calibrate.disabled = False
        self.ids.bt_target.disabled = False
        self.ids.bt_start.disabled = True

        self.track_enable = True

        # Calibrate needle before tracking
        self.cmd_calibrate()

        # Show needle
        self.NeedleOpacity = 1.0 

    def cmd_start_cobot(self):
        self.ids.bt_stop_cobot.disabled = False
        self.ids.bt_calibrate_cobot.disabled = False
        self.ids.bt_start_cobot.disabled = True

        self.cobot_track_enable = True

        # Init com before tracking
        # self.cobot_init_com()

        # Calibrate needle before tracking
        self.cmd_calibrate_cobot()

        # Show needle
        self.cobot_NeedleOpacity = 1.0 

    def cmd_stop(self):
        self.ids.bt_stop.disabled = True
        self.ids.bt_calibrate.disabled = True
        self.ids.bt_target.disabled = True
        self.ids.bt_start.disabled = False

        self.ids['lbl_track'].text = 'Pos x y z  Rot y p r'
        self.track_enable = False
        self.NeedleOpacity = 0.0

    def cmd_stop_cobot(self):
        self.ids.bt_stop_cobot.disabled = True
        self.ids.bt_calibrate_cobot.disabled = True
        self.ids.bt_start_cobot.disabled = False

        self.ids['lbl_track_cobot'].text = 'Pos x y z  Rot y p r'
        self.track_enable_cobot = False
        self.cobot_NeedleOpacity = 0.0

        self.cobot_exit_com()

    def cmd_calibrate(self):
        # Read the probe
        data = self.get_data_probe()

        # check if the probe is not lost
        if data['flag'] == 'lost':
            return
        else:
            self.origin_needle[0] = data['x']
            self.origin_needle[1] = data['y']
            self.origin_needle[2] = data['z']

    def cmd_calibrate_cobot(self):
        # Read the cobot
        data = self.get_data_cobot()

        # check if the probe is not lost
        if data['flag'] == 'lost':
            return
        else:
            self.cobot_origin_needle[0] = data['x']
            self.cobot_origin_needle[1] = data['y']
            self.cobot_origin_needle[2] = data['z']

    def cmd_target(self):
        if self.target_on is False:
            # Read the probe
            data = self.get_data_probe()

            # check if the probe is not lost
            if data['flag'] == 'lost':
                return
            else:
                self.target_pos[0] = data['x']
                self.target_pos[1] = data['y']
                self.target_pos[2] = data['z']

                self.target_on = True
                self.TargetOpacity = 1.0
        else:
            self.target_on = False
            self.TargetOpacity = 0.0

    def cmd_plan(self):

        # Starting point Coronal view
        CoX0 = self.cfg['plan']['CoX0']
        CoY0 = self.cfg['plan']['CoY0']

        # Starting point Sagital view
        SaX0 = self.cfg['plan']['SaX0']
        SaY0 = self.cfg['plan']['SaY0']
        
        # Seed dimension
        Sw = self.cfg['plan']['Sw']
        Sh = self.cfg['plan']['Sh']
        # Padding
        Px = self.cfg['plan']['Px']
        Py = self.cfg['plan']['Py']

        # Coronal view
        #          Needle 1                Needle 2
        CoPlanX = [CoX0, CoX0, CoX0, CoX0, CoX0+Px, CoX0+Px, CoX0+Px, CoX0+Px]
        #           Needle 1                            Needle 2
        CoPlanY = [CoY0, CoY0+Py, CoY0+3*Py, CoY0+4*Py, CoY0, CoY0+Py, CoY0+2*Py, CoY0+3*Py]

        # Sagital view
        #       Needles
        SaPlanX = [SaX0, SaX0+Py, SaX0+2*Py, SaX0+3*Py, SaX0+4*Py]
        #       Needles
        SaPlanY = [SaY0, SaY0, SaY0, SaY0, SaY0]

        with self.canvas:
            Color(1, 0, 0)
            # Coronal view
            for i in range(len(CoPlanX)):
                px = CoPlanX[i]
                py = CoPlanY[i]
                Rectangle(pos=(px, py), size=(Sw, Sh))

            # Sagital view
            for i in range(len(SaPlanX)):
                px = SaPlanX[i]
                py = SaPlanY[i]
                Rectangle(pos=(px, py), size=(Sh, Sw))


class NeedleTrackApp(App):
    def build(self):

        tracker = Tracker()

        tracker.init_tracking() 

        return tracker

if __name__ in ('__main__'):
    NeedleTrackApp().run()