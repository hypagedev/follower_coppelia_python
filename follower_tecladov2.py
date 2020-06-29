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
        self.mot_Izq = "DynamicLeftJoint"
        self.mot_Der = "DynamicRightJoint"
        self.ip = ip
        self.port = port
        self.velI = 10*math.pi/180.0
        self.velD = 10*math.pi/180.0
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

        imgL = self.get_image(self.camLeft)
        imgM = self.get_image(self.camMid)
        imgR = self.get_image(self.camRight)
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
            elif tecla != 255:
                    self.mover_tecladov2(tecla)

        #cerrar
        sim.simxFinish(self.clientID)

    def mover_teclado(self, clave):
        if(clave == 97 or clave == 65 ): #a=97 A=65
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, self.velD, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, -self.velI, sim.simx_opmode_oneshot)
        if(clave == 119 or clave == 87 ): #w=119 W=87
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, self.velD, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, self.velI, sim.simx_opmode_oneshot)
        if(clave == 115 or clave == 83 ): #s=97 S=65
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, -self.velD, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, -self.velI, sim.simx_opmode_oneshot)
        if(clave == 100 or clave == 68 ): #d=97 D=65
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, -self.velD, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, self.velI, sim.simx_opmode_oneshot)
        if(clave == 48 ): #0=48
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, 0, sim.simx_opmode_oneshot)

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

def main():
    ex = Bot()
    ex.execute()


if __name__ == '__main__':
    main()
