import math
#import pandas as pd

from PyQt5 import QtCore, QtGui, QtWidgets

# cl = {'13.7': [-0.06268752565831914,0.055642777549703815,0.34289220573573387,0.3912245616092897,0.40737082869039926,0.4861527183156411]
#      ,'21.6': [0.11171661486129825, 0.1589027036537446, 0.3184203272535334, 0.36117969620550106, 0.38180898481027503,0.4529329658084568]
#      ,'29.9': [0.17064952202216876, 0.23448771333059615, 0.2906328074599729, 0.3311911324848981, 0.3474218520907863, 0.39066330420503176]
#      ,'38.4': [0.13164565128144956, 0.18782593396281733, 0.23167878716790796, 0.2767271714268958, 0.3074159873321211, 0.3483684241393465]
#      ,'46.9': [0.1667647058823527, 0.12731394053134704, 0.20291038648759585, 0.23234736965573854, 0.26519852208081623, 0.3106128672805114]
#      ,'55.2': [0.1162101929505599, 0.1455697613043222, 0.17845962113658997, 0.20681367661720707, 0.23408480441029833, 0.27395783238519716]
#      ,'63.1': [0.11398803589232298, 0.1400108498035304, 0.16513019764236692, 0.1890329012961115, 0.21630754794440193, 0.25287197231833886]
#      ,'71.9': [0.11619611752976344, 0.13446601372353506, 0.15290481496686392, 0.1756999589466891, 0.1963046155650694, 0.23286552108380731]
#      ,'80.2': [0.10731452700721356, 0.12563368717377266, 0.14624186264735206, 0.16458213594510585, 0.18408627059996463, 0.21619582429183026]
#      ,'89.1': [0.10955779719664527, 0.12563368717377266, 0.14402322444431392, 0.15457187261744154, 0.17186440677966086, 0.20286991965280599]}
# cldf = pd.DataFrame(data=cl,index=['1011.55','1881.78','2799.63','3673.1','4603.46','6184.94'])

class Canvas(QtWidgets.QFrame):
    def __init__(self, parent):
        QtWidgets.QFrame.__init__(self, parent)
        
        self.__bg_color = QtGui.QColor(120, 120, 120)
        self.__bg_brush = QtGui.QBrush(self.__bg_color, QtCore.Qt.SolidPattern)
        self.__pen = QtGui.QPen(QtGui.QColor(255, 128, 128), 2, QtCore.Qt.SolidLine)
        self.__pen2 = QtGui.QPen(QtGui.QColor(128, 128, 255), 2, QtCore.Qt.SolidLine)
        self.__parent = parent

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
        self.__paintPath = QtGui.QPainterPath()
        self.clear = False
        self.mu = 0.1

    def hit(self):
        traj = [QtCore.QPointF(0.0, 0.0)]

        moment_chg = self.force * self.contact_t
        v0 = moment_chg / self.mass 
        launch_angle = math.radians(self.angle)

        print("init: ")
        print(v0, math.degrees(launch_angle), self.spin)
        (hit_angle, v, cur_spin) = self.__launch(traj, v0, launch_angle, self.spin)
        print("hit: ")
        print(v, math.degrees(hit_angle), cur_spin)
        hit_angle_from_horiz = (math.pi/2.) + hit_angle
        (rebound_angle, v2, new_spin) = self.__bounce3(v, hit_angle_from_horiz, cur_spin)
        print("1st bounce: ")
        print(v2, math.degrees(rebound_angle), new_spin)
        rebound_from_horiz = (math.pi/2.) - rebound_angle
        (hit_angle, v, cur_spin) = self.__launch(traj, v2, rebound_from_horiz, new_spin, False, False)#, False, False)
        hit_angle_from_horiz = (math.pi/2.) + hit_angle
        (rebound_angle, v2, new_spin) = self.__bounce3(v, hit_angle_from_horiz, cur_spin)
        rebound_from_horiz = (math.pi/2.) - rebound_angle
        (hit_angle, v, cur_spin) = self.__launch(traj, v2, rebound_from_horiz, new_spin, False, False)
        
        # (rebound_angle, v2, new_spin) = self.__bounce(v, -hit_angle, cur_spin)

        # (hit_angle, v, cur_spin) = self.__launch(traj, v2, rebound_angle, new_spin)

        # (rebound_angle, v2, new_spin) = self.__bounce(v, -hit_angle, cur_spin)

        # (hit_angle, v, cur_spin) = self.__launch(traj, v2, rebound_angle, new_spin)

        botY = self.frameRect().bottom()
        self.__paintPath = QtGui.QPainterPath()
        self.__paintPath.moveTo(traj[0].x(), botY-traj[0].y())
        for point in traj[1:]:
            #print(point)
            self.__paintPath.lineTo(point.x()*2, botY-point.y()*2)

        self.repaint()

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

    # def __find_nearest(self,velocity,spin):
    #     LowSpin = 0
    #     HighSpin = 6200
    #     LowVel = 0
    #     HighVel = 90

    #     for values in cldf.index:
    #         if spin > values:
    #             LowSpin = values
    #         if spin < values and values < HighSpin:
    #             HighSpin = values

    #     for elements in cldf.columns:
    #         if velocity > elements:
    #             LowVel = elements
    #         if velocity < elements and elements < HighVel:
    #             HighVel = elements

    #     return LowSpin,HighSpin,LowVel,HighVel

    # def __interpolate(self, velocity, spin):
    #     y1,y2,x1,x2 = find_nearest(velocity,spin)
    #     f11 = cldf[x1][y1]
    #     f12 = cldf[x1][y2]
    #     f21 = cldf[x2][y1]
    #     f22 = cldf[x2][y2]

    #     return (1/((x2-x1)*(y2-y1))) * ( f11*(x2-velocity)*(y2-spin) + f21*(velocity-x1)*(y2-spin)
    #                                 + f12*(x2-velocity)*(spin-y1) + f22*(velocity-x1)*(spin-y1))



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
        
            #c_drag = self.__drag(Re, spin_ratio)
            #print(Re, c_drag)
            #c_drag = self.cd + (.25 * spin_ratio)
            c_drag = 21.12/Re + 6.3/math.sqrt(Re) + self.cd
            #print (c_drag)
            #c_drag = c_drag - (spin_ratio/1000.)
            #print (c_drag)
            #print ("-")
            #print(Re, c_drag, cd2)
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

        return angle, v, spin_t*60./(2.*math.pi)

    def resizeEvent(self, event):
        self.repaint()

    def paintEvent(self, event):
        dc = QtGui.QPainter()

        dc.begin(self)
        dc.setRenderHint(QtGui.QPainter.Antialiasing)

        dc.setBackground(self.__bg_brush)
        dc.eraseRect(self.frameRect())
        dc.save()
    
        dc.strokePath(self.__paintPath, self.__pen2)
    
        dc.restore()
        dc.end()

        QtWidgets.QFrame.paintEvent(self, event)
    
