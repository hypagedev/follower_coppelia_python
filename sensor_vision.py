import sim
import sys
import time
import cv2
import numpy as np


def get_image(clientID, idcam):
    _, resolution, image=sim.simxGetVisionSensorImage(clientID, idcam, 0, sim.simx_opmode_buffer)
    if(len(resolution) == 0):
        print(idcam)
        return None
    img = np.array(image, dtype = np.uint8)
    img.resize([resolution[0], resolution[1], 3])
    img = np.fliplr(img)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img

print ('Programa inicio')
sim.simxFinish(-1) # cerrar todas las conexiones
# Conectar a CoppeliaSim
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Conectado al API del servidor remoto')


    imgL = get_image(clientID,camLeft)
    imgM = get_image(clientID,camMid)
    imgR = get_image(clientID,camRight)

    while(1):
        #Guardar frame de la camara, rotarlo y convertirlo a BGR
        imgL = get_image(clientID,camLeft)
        imgM = get_image(clientID,camMid)
        imgR = get_image(clientID,camRight)

        #Mostrar frame y salir con "ESC"
        cv2.imshow('Left', imgL)
        cv2.imshow('Right', imgR)
        cv2.imshow('Middle', imgM)

        tecla = cv2.waitKey(5) & 0xFF
        if tecla == 27:
            break
    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    sys.exit('Failed connecting to remote API server')
print ('Program ended')
