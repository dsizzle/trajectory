"""
edit_control.py

Contains EditorController class.
"""
import math
import os

from PyQt5 import QtGui, QtCore, QtWidgets

import ui

M_TO_YDS = 1.09361


class MainController(object):
    """
    EditorController is the main Controller for pyTalic Editor
    """
    def __init__(self, w, h, label, script_path):
        self.__label = label
        self.__ui = ui.MainInterface(self, w, h, label)

        self.__ui.create_ui()

        self.angle = 0.0
        self.force = 0.0    #N  = kg*m/s^2
        self.spin = 4000.0 #rpm
        self.gravity = 0.0
        self.mass = 0.0459   #kg
        self.contact_t = .0005  #time of contact
        self.cd = 0
        self.liftFactor = 0.36
        self.__t_step = 0.05
        self.radius = 0.04267 / 2.  # 42.67 mm diameter
        self.air_density = 1.225 #kg/m^3  - changes based on temperature, altitude, humidity
        self.air_visc = 1.802e-5
        self.ball_xsect = math.pi * self.radius * self.radius
        self.mu = 0.1

        self.__ui.repaint()

    def activate(self):
        self.__ui.show()
        self.__ui.activateWindow()
        self.__ui.raise_()
  
    def mouse_event(self, event):
        event.accept()

    def wheel_event(self, event):
        event.accept()

    def key_event(self, event):
        event.accept()

    def paintEvent(self, event):
        QtWidgets.QMainWindow.paintEvent(self, event)

    def quit_cb(self, event):
        self.__ui.close()

        return True

    def getUI(self):
        return self.__ui

    def go(self, event):
        print("GO")
        print(self.__ui.angleSpinBox.value())

        # self.__ui.frame.angle = self.__ui.angleSpinBox.value()
        # self.__ui.frame.force = self.__ui.forceSpinBox.value()
        # self.__ui.frame.spin = self.__ui.spinSpinBox.value()
        # self.__ui.frame.gravity = self.__ui.gravitySpinBox.value()
        # self.__ui.frame.cd = self.__ui.cdSpinBox.value()
        # self.__ui.frame.liftFactor = self.__ui.factorSpinBox.value()
        # self.__ui.frame.mu = self.__ui.muSpinBox.value()

        self.__ui.frame.clear = False
        traj = self.hit() #repaint()
        
        self.__ui.frame.setPath(traj)
        self.__ui.repaint()
        #self.__ui.frame_2.hit() 

    def clear(self, event):
        self.__ui.frame.clear = True
        self.__ui.frame.repaint()

    def hit(self):
        if not self.__ui:
            return

        self.angle = self.__ui.angleSpinBox.value()
        self.force = self.__ui.forceSpinBox.value()
        self.spin = self.__ui.spinSpinBox.value()
        self.gravity = self.__ui.gravitySpinBox.value()
        self.cd = self.__ui.cdSpinBox.value()
        self.liftFactor = self.__ui.factorSpinBox.value()
        self.mu = self.__ui.muSpinBox.value()

        traj = [QtCore.QPointF(0.0, 0.0)]

        moment_chg = self.force * self.contact_t
        v0 = moment_chg / self.mass 
        launch_angle = math.radians(self.angle)

        print("init: ")
        print(v0, math.degrees(launch_angle), self.spin)
        (hit_angle, v, cur_spin, carry, height) = self.__launch(traj, v0, launch_angle, self.spin)
        print("hit: ")
        print(v, math.degrees(hit_angle), cur_spin)
        hit_angle_from_horiz = (math.pi/2.) + hit_angle
        (rebound_angle, v2, new_spin) = self.__bounce3(v, hit_angle_from_horiz, cur_spin)
        print("1st bounce: ")
        print(v2, math.degrees(rebound_angle), new_spin)
        rebound_from_horiz = (math.pi/2.) - rebound_angle
        (hit_angle, v, cur_spin, x, y) = self.__launch(traj, v2, rebound_from_horiz, new_spin, False, False)#, False, False)
        hit_angle_from_horiz = (math.pi/2.) + hit_angle
        (rebound_angle, v2, new_spin) = self.__bounce3(v, hit_angle_from_horiz, cur_spin)
        rebound_from_horiz = (math.pi/2.) - rebound_angle
        (hit_angle, v, cur_spin, x, y) = self.__launch(traj, v2, rebound_from_horiz, new_spin, False, False)
        
        # (rebound_angle, v2, new_spin) = self.__bounce(v, -hit_angle, cur_spin)

        # (hit_angle, v, cur_spin) = self.__launch(traj, v2, rebound_angle, new_spin)

        # (rebound_angle, v2, new_spin) = self.__bounce(v, -hit_angle, cur_spin)

        # (hit_angle, v, cur_spin) = self.__launch(traj, v2, rebound_angle, new_spin)

        self.__ui.distanceField.setText(str(carry*M_TO_YDS))
        self.__ui.heightField.setText(str(height*M_TO_YDS))
        
        # self.mu = 0.4
        # print("=================================")
        # (rebound_angle, v2, new_spin) = self.__bounce3(18.6, math.radians(44.4), 4621)

        # self.mu = 0.4
        # print("---------------------------------")
        # wr = 224 * 60. / (2. * math.pi)
        # (rebound_angle, v2, new_spin) = self.__bounce3(26.8, math.radians(68.0), wr)
        # # rebound_from_horiz = (math.pi/2.) - rebound_angle
        # # self.__launch(traj, v2, rebound_from_horiz, new_spin)

        # self.mu = 0.4
        # print("+++++++++++++++++++++++++++++++++")
        # wr = 1000 * 60. / (2. * math.pi)
        # (rebound_angle, v2, new_spin) = self.__bounce3(23.8, math.radians(34.0), wr)

        return traj

    def __bounce2(self, v, in_angle, spin_rate):
        e = .120
        angle = in_angle #(math.pi/2) - in_angle
        #print(math.degrees(in_angle), math.degrees(angle))
        vy = v * math.cos(angle)
        vx = v * math.sin(angle)
        if v <= 20:
            e = .510 - .0375*vy + .000903*vy*vy

        print (e, v, vx, vy)
        
        mu = self.mu 

        f = math.sqrt(1-e)
        uy = -f*vy

        vcx = vx + spin_rate*self.radius    # tangential contact speed
        if abs(vcx) <= 2.5*mu*(1+f)*abs(vy):
            # Within friction limit, sticky contact.
            ux = vx - 0.4*vcx
            wout = spin_rate - 0.6*vcx/self.radius
        else:
            # Exceeds friction limit, skidding contact.
            sign_vcx = (-1 if vcx < 0 else 1)
            pom = mu*(1+f)*abs(vy) * sign_vcx
            dvcx = 2.5*pom
            ux = vx - 0.4*dvcx
            wout = spin_rate - 0.6*dvcx/self.radius

        fc = abs(vcx) / (2.5*(1+f)*abs(vy)) if vy != 0 else 1000

        rebound_angle = -math.atan(ux/uy)
        v2 = math.sqrt(ux * ux + uy * uy)

        return rebound_angle, v2, -wout

    def __bounce3(self, vi, in_angle, spin_rate):
        mu = self.mu

        print("mu:", mu)
        
        theta = in_angle
        wi =  2.*math.pi*spin_rate/60.

        vix = vi*math.sin(theta)
        viy = -vi*math.cos(theta)
        phi = abs(math.degrees(math.atan(vix/viy)))
        theta_c = math.radians(15.4*(vi/18.6)*(phi/44.4))
        vixp = vix*math.cos(theta_c)-abs(viy)*math.sin(theta_c)
        viyp = vix*math.sin(theta_c)+abs(viy)*math.cos(theta_c)

        if abs(viyp) > 20:
            e = .120
        else:
            e = .510-.0375*abs(viyp)+.000903*(viyp*viyp)

        mu_c = (2.*(vixp+self.radius*wi))/(7.*(1.+e)*abs(viyp))
        
        print ("vi", "vix", "viy", "theta", "phi")
        print (vi, vix, viy, math.degrees(theta), phi)

        print ("vixp", "viyp", "theta_c")
        print (vixp, viyp, math.degrees(theta_c))

        print ("wi", "r")
        print (wi, self.radius)

        print ("e", "mu", "mu_c")
        print (e, mu, mu_c)
        if mu < mu_c:
            print("bouncing")
            vrxp = vixp-mu*abs(viyp)*(1.+e)
            vryp = e*abs(viyp) 
            wr = wi-(5.*mu)/(2.*self.radius)*abs(viyp)*(1.+e)
        else:
            print("rolling")
            vrxp = (5./7.)*vixp-(2./7.)*self.radius*wi
            vryp = e*abs(viyp)
            wr = -(vrxp/self.radius)

        print ("vrxp", "vryp", "wr")
        print (vrxp, vryp, wr)

        vrx = vrxp*math.cos(theta_c)-vryp*math.sin(theta_c)
        vry = vrxp*math.sin(theta_c)+vryp*math.cos(theta_c)

        print ("vrx", "vry")
        print (vrx, vry)

        rebound_angle = math.atan(vry/vrx) - theta_c
        v2 = math.sqrt(vrx*vrx + vry*vry)
        new_spin = wr

        print("rebound_angle", "v2", "new_spin")
        print(math.degrees(rebound_angle), v2, new_spin)

        print("bounce distance:", (2./-self.gravity)*vrx*vry)
        return rebound_angle, v2, new_spin

    def __drag(self, Re, S=0):
        Re_cr = 0.6e5
        c_d1 = 0.25
        c_d3 = 0.1e-3
        c_d2 = (Re-Re_cr)*c_d3
        k1 = Re*c_d3
        k2 = Re*c_d3-c_d2
            
        if Re < Re_cr:
            D_Re = k1 + c_d1*k1 - 1
        else:
            D_Re = k2 + c_d1*k2 - 0.0225*c_d2 - 1

        c_d = .2136 * (-2.1 * math.exp(-.12*(D_Re + S + 0.35)) + \
                8.9 * math.exp(-.22*(D_Re + 0.35)))

        return c_d

    def __launch(self, traj, v0, angle, spin_rate, lift=True, drag=True):
        x = traj[-1].x()
        y = traj[-1].y()  

        spin0 = 2.*math.pi*spin_rate/60.  

        v0x = v0 * math.cos(angle)
        v0y = v0 * math.sin(angle)
        a0x = 0
        a0y = self.gravity

        vx = v0x
        vy = v0y
        ax = a0x
        ay = a0y
        t = 0
        spin_t = spin0
        y_max = 0
        in_air = True
        dist = x
        
        while in_air:
            v2 = vx * vx + vy * vy
            v = math.sqrt(v2)
            Re = (self.air_density * v * (self.radius + self.radius))/self.air_visc
            
            spin_t = spin0-(.00002)*(spin_t*v/self.radius)*t
            
            #print(Re)
            #spin_t = spin0 * math.exp(-t/30.)
            spin_ratio = abs(float(self.radius*spin_t/v))
            
            try:
                Cl = self.__interpolate(v,spin_t)
                Cd = self.__interpolate(v,spin_t)
            except:
                Cl = 0.3
                Cd = 0.3

            f_drag_lift = .5 * self.air_density*self.ball_xsect*v2
        
            c_drag = 21.12/Re + 6.3/math.sqrt(Re) + self.cd
            accel_drag = f_drag_lift * c_drag #/ self.mass

            if not drag:
                accel_drag = 0

            accel_dragx = accel_drag * math.cos(angle) / self.mass
            accel_dragy = accel_drag * math.sin(angle) / self.mass
            
            c_lift = .05+math.sqrt(.0025+self.liftFactor*spin_ratio)
            #c_lift = -3.25*spin_ratio*spin_ratio + 1.99*spin_ratio
            
            accel_lift = f_drag_lift * c_lift #/ self.mass
            
            if spin_rate < 0:
                accel_lift = 0 - accel_lift

            if not lift:
                accel_lift = 0

            accel_liftx = accel_lift * math.cos(angle) / self.mass
            accel_lifty = accel_lift * math.sin(angle) / self.mass
            
            ax = a0x - accel_dragx - accel_lifty
            ay = a0y - accel_dragy + accel_liftx

            vx = vx + self.__t_step * ax
            vy = vy + self.__t_step * ay       

            xl = x
            yl = y
            x = x + self.__t_step * vx
            y = y + self.__t_step * vy
            
            if y < 0:
                in_air = False
                y = 0
                t_step_y0 = -float(yl / vy)
                x = xl + t_step_y0 * vx

            dx = xl - x
            dy = yl - y
            
            if dx == 0:
                dx = 0.00001

            angle = math.atan(dy/dx)

            traj.append(QtCore.QPointF(x, y))
            
            if y > y_max:
                y_max = y
            
            t += self.__t_step

        dist = x - dist
        print ("distance: ", dist, "max height:", y_max)
        print ("spin0: ", spin0, "final spin:", spin_t)

        return angle, v, spin_t*60./(2.*math.pi), dist, y_max