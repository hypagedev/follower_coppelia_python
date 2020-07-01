import sim
import sys
import time
import cv2
import numpy as np
import math

class Bot:
    def __init__(self, ip="127.0.0.1", port=19999):
        self.sen_Izq = "LeftSensor"
        self.sen_Der = "RightSensor"
        self.sen_Mid = "MiddleSensor"
        self.camaraSuperior = "Vision_sensor"
        self.ultrasonido = "Proximity_sensor"
        self.mot_Izq = "DynamicLeftJoint"
        self.mot_Der = "DynamicRightJoint"
        self.ip = ip
        self.port = port
        self.velI = 0.03
        self.velD = 0.03
        self.delta = 0.1
        self.radio_rueda = 0.027
        self.dmax = 0.35 #distancia maxima al sensor
        self.dmin = 0.08 #distancia minima del objeto al sensor

        self.connect()
        self.init_elements()

    def connect(self):
        print ('Programa inicio')
        sim.simxFinish(-1) # cerrar todas las conexiones
        # Conectar a CoppeliaSim
        self.clientID=sim.simxStart(self.ip,self.port,True,True,5000,5)
        self.activo = True
        if(self.clientID == -1):
            print("Imposible conectar")
            self.activo = False

    def init_elements(self):
        if(self.activo == False):
            return
        #Guardar la referencia de la camara
        _, self.camLeft = sim.simxGetObjectHandle(self.clientID, self.sen_Izq, sim.simx_opmode_oneshot_wait)
        _, self.camMid = sim.simxGetObjectHandle(self.clientID,self.sen_Mid, sim.simx_opmode_oneshot_wait)
        _, self.camRight = sim.simxGetObjectHandle(self.clientID, self.sen_Der, sim.simx_opmode_oneshot_wait)
        _, self.motorRight = sim.simxGetObjectHandle(self.clientID, self.mot_Der, sim.simx_opmode_oneshot_wait)
        _, self.motorLeft = sim.simxGetObjectHandle(self.clientID, self.mot_Izq, sim.simx_opmode_oneshot_wait)
        _, self.camSup = sim.simxGetObjectHandle(self.clientID, self.camaraSuperior, sim.simx_opmode_oneshot_wait)
        _, self.ultra = sim.simxGetObjectHandle(self.clientID, self.ultrasonido, sim.simx_opmode_oneshot_wait)

        imgL = self.sensor_ir(self.camLeft)
        imgM = self.sensor_ir(self.camMid)
        imgR = self.sensor_ir(self.camRight)
        img = self.get_image(self.camSup)
        time.sleep(1)

    def get_image(self, idcam):
        if(self.activo == False):
            return
        _, resolution, image=sim.simxGetVisionSensorImage(self.clientID, idcam, 0, sim.simx_opmode_streaming)
        if(len(resolution) == 0):
            print("No pudo conectarse a la camara {}".format(idcam))
            return None
        img = np.array(image, dtype = np.uint8)
        img.resize([resolution[0], resolution[1], 3])
        img = np.fliplr(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def execute(self):
        if(self.activo == False):
            return

        while(1):
            img = self.get_image(self.camSup)

            cv2.imshow('Image', img)

            tecla = cv2.waitKey(5) & 0xFF
            if tecla == 27:
                break
            #self.vel_follow()
            d = self.get_distance(self.ultra,self.dmax)
            print("distancia sensado: {}".format(d))


        #cerrar
        sim.simxFinish(self.clientID)


    def mover_tecladov2(self, clave): #mover a posicion
        dir = [0 , 0] # der, izq
        delta = 0.05

        if(clave == 119 or clave == 87 ): #w=119 W=87 up
            dir[1] += 1
            dir[0] += 1
        if(clave == 115 or clave == 83 ): #s=97 S=65 down
            dir[1] -= 1
            dir[0] -= 1

        if(clave == 97 or clave == 65 ): #a=97 A=65 izq
            dir[0] += 1
            dir[1] -= 1

        if(clave == 100 or clave == 68 ): #d=97 D=65 der
            dir[0] -= 1
            dir[1] += 1

        print("Inicio: {} {}".format(self.velI, self.velD))
        self.velD += dir[0]*delta
        self.velI += dir[1]*delta
        sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, self.velD, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, self.velI, sim.simx_opmode_oneshot)

    def sensor_ir(self, idcam):
        if(self.activo == False):
            return

        _, resolution, image=sim.simxGetVisionSensorImage(self.clientID, idcam, 1, sim.simx_opmode_streaming)
        if(len(resolution) == 0):
            print("No pudo conectarse a la camara {}".format(idcam))
            return None
        img = np.array(image, dtype = np.uint8)
        return img[0]

    def vel_follow(self):
        kline = 0.01
        v = 0.01
        imgM = self.sensor_ir(self.camMid)/255.0
        imgL = self.sensor_ir(self.camLeft)/255.0
        imgR = self.sensor_ir(self.camRight)/255.0

        A = imgL - imgM
        B = imgR - imgM

        w = -kline*(A - B)
        vl = v - self.delta*w
        vr = v + self.delta*w
        d = self.get_distance(self.ultra,self.dmax)
        print("distancia sensado: {}".format(d))
        #ajustando la velocidad a la deteccion de obstaculo
        vl = self.get_velocity(d,vl)
        vr = self.get_velocity(d,vr)
        wL = vl/self.radio_rueda
        wR = vr/self.radio_rueda
        print("A: {}, B: {}, w: {} , vl: {} , vr: {}, wL: {}, wR:{}".format(A,B, w,vl,vr,wL,wR))
        sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, wR, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, wL, sim.simx_opmode_oneshot)

    #ajusta la velocidad acorde a la deteccion del sensor
    def get_velocity(self,d, vmax):
        if(d < self.dmin):
            return 0
        else:
            return vmax*d/self.dmax

    def get_distance(self,sensor,max_dist):
        _, sta, point, objh, vec = sim.simxReadProximitySensor(self.clientID, sensor, sim.simx_opmode_streaming)

        if(sta ==  False): #no se detecto nada
            distance = max_dist
        else:
            distance = math.sqrt( point[0]**2 + point[1]**2 + point[2]**2)
        return distance


def main():
    ex = Bot()
    ex.execute()


if __name__ == '__main__':
    main()
