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
        self.mot_Izq = "DynamicLeftJoint"
        self.mot_Der = "DynamicRightJoint"
        self.mot_Izq_Fijo = "LeftJoint"
        self.mot_Der_Fijo = "RightJoint"
        self.robotName = "LineTracer"
        self.camaraSuperior = "Camara_sup"
        self.ip = ip
        self.port = port
        self.velI = 0.03
        self.velD = 0.03
        self.delta = 0.1
        self.speed = 0.0015
        self.radio_rueda = 0.027

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
        _, self.motorRightFijo = sim.simxGetObjectHandle(self.clientID, self.mot_Der_Fijo, sim.simx_opmode_oneshot_wait)
        _, self.motorLeftFijo = sim.simxGetObjectHandle(self.clientID, self.mot_Izq_Fijo, sim.simx_opmode_oneshot_wait)
        _, self.robot = sim.simxGetObjectHandle(self.clientID, self.robotName, sim.simx_opmode_oneshot_wait)
        _, self.camSup = sim.simxGetObjectHandle(self.clientID, self.camaraSuperior, sim.simx_opmode_oneshot_wait)

        self.velocidadId(self.motorLeftFijo)
        self.velocidadId(self.motorRightFijo)
        img = self.get_image(self.camSup)
        imgL = self.sensor_ir(self.camLeft)
        imgM = self.sensor_ir(self.camMid)
        imgR = self.sensor_ir(self.camRight)

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
        img = np.rot90(img,3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def posicionId(self, id):
        _ , pos = sim.simxGetObjectPosition(self.clientID, id, -1, sim.simx_opmode_streaming )
        return pos

    def velocidadId(self, id):
        _ , pos = sim.simxGetJointPosition(self.clientID, id, sim.simx_opmode_streaming )
        return pos

    def vel_follow(self):
        kline = 0.01
        gray = 0.5
        v = 0.01
        imgM = self.sensor_ir(self.camMid)/255.0
        imgL = self.sensor_ir(self.camLeft)/255.0
        imgR = self.sensor_ir(self.camRight)/255.0

        A = imgL - imgM
        B = imgR - imgM

        w = -kline*(A - B)
        vl = v - self.delta*w
        vr = v + self.delta*w
        wL = vl/self.radio_rueda
        wR = vr/self.radio_rueda
        print("A: {}, B: {}, w: {} , vl: {} , vr: {}, wL: {}, wR:{}".format(A,B, w,vl,vr,wL,wR))
        sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, wR, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, wL, sim.simx_opmode_oneshot)

    def sensor_ir(self, idcam):
        if(self.activo == False):
            return

        _, resolution, image=sim.simxGetVisionSensorImage(self.clientID, idcam, 1, sim.simx_opmode_streaming)
        if(len(resolution) == 0):
            print("No pudo conectarse a la camara {}".format(idcam))
            return None
        img = np.array(image, dtype = np.uint8)
        return img[0]

    def execute(self):
        if(self.activo == False):
            return

        while(1):
            #Guardar frame de la camara, rotarlo y convertirlo a BGR

            img = self.get_image(self.camSup)

            #Mostrar frame y salir con "ESC"
            cv2.imshow('Camara superior', img)

            tecla = cv2.waitKey(5) & 0xFF
            if tecla == 27:
                break

            self.vel_follow()

        #cerrar
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot)
        time.sleep(1)
        sim.simxFinish(self.clientID)

    def mover_robot_tecladov1(self, clave):
        if(clave == 97 or clave == 65 ): #a=97 A=65
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, math.pi/2.0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, math.pi/-2.0, sim.simx_opmode_oneshot)

        if(clave == 119 or clave == 87 ): #w=119 W=87
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, math.pi/2.0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, math.pi/2.0, sim.simx_opmode_oneshot)


        if(clave == 115 or clave == 83 ): #s=97 S=65
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, math.pi/-2.0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, math.pi/-2.0, sim.simx_opmode_oneshot)

        if(clave == 100 or clave == 68 ): #d=97 D=65
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, -math.pi/-2.0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, -math.pi/2.0, sim.simx_opmode_oneshot)

        if(clave == 48 ): #0=48
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, 0, sim.simx_opmode_oneshot)

    def mover_robot_tecladov2(self, clave): #mover a posicion
        dir = [0 , 0] # izq, der

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
        self.velD += dir[0]*self.delta
        self.velI += dir[1]*self.delta
        print("Fin: {} {}".format(self.velI, self.velD))

        posI = self.velI/self.radio_rueda
        posD = self.velD/self.radio_rueda

        sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, posD, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, posI, sim.simx_opmode_oneshot)

        #sim.simxSetJointPosition(self.clientID, self.motorLeftFijo, posI,sim.simx_opmode_oneshot )
        #sim.simxSetJointPosition(self.clientID, self.motorRightFijo, posD,sim.simx_opmode_oneshot )


def main():
    ex = Bot()
    ex.execute()


if __name__ == '__main__':
    main()
