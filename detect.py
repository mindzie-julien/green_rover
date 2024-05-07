import numpy as np
import cv2
import math
import time
import computer as pc

# Checks if a matrix is a valid rotation matrix
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot (Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
#The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] * R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[2, 11])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])
####################################
def positionRobot(id,dx,dy,deg): #id:id d'aruco x,y : distances entre aruco et camera dans le repere de l'aruco; deg : angle entre l'axe y de camera et aruco
    #idaruco=>>cordonnee aruco
    #cordonnee aruco (xA,yA,degAruco=90)
    if 20 in id:
        xA = 735
        yA = 2000 - 490
    elif 21 in id:
        xA = 3000 - 735
        yA = 2000 - 490
    elif 22 in id:
        xA = 735
        yA = 490
    elif 23 in id:
        xA = 3000 - 735
        yA = 490
    xC=xA+dx #xC = x de camera
    yC=yA+dy #yC = y de camera
    d=0	#ditance entre le centre de robot et la camera
    ori = 90 + deg #orientation de robot dans le repere de tapis
    xR = -math.sin(deg)*d + xC
    yR = -math.cos(deg)*d + yC

    return xR,yR,ori

####################################
marker_size = 100 #la taille reelle de l'aruco##########################

with open ( 'camera_cal.npy', 'rb') as f: #.npy fichier genere par le code de calibration, reference de mesure de distance et angle
    camera_matrix = np. load(f)
    camera_distortion = np. load(f)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

cap = cv2.VideoCapture(0)
fps = cap.get(cv2.CAP_PROP_FPS) #method to show fps of your video
print(f"{fps=}")
loopNumber = 0
while True :
    markerIndex = -1
    ret, frame = cap.read()  # grab a frame
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to Grayscale
    # -- Find all the aruco markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)
    #print(corners,ids,rejected)
    markerIndex = []

    if ids is not None:
#        print(f"{ids}")
        for i in range(ids.size):
            if ids[i] in [20, 21, 22, 23]:
                markerIndex.append(i)
#                print(f"{ids[i]=}")
    if markerIndex != []:
        corners = np.array([corners[i] for i in markerIndex])
        ids = np.array([ids[i] for i in markerIndex])

#        cv2.aruco.drawDetectedMarkers(gray_frame, corners, ids)  # draw a box around all the detected markers
        # get pose of all single markers
        rvec_list_all, tvec_list_all, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        rvec = rvec_list_all[0][0]
        tvec = tvec_list_all[0][0]

        cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 100)

        rvec_flipped = rvec * -1
        tvec_flipped = tvec * -1
        rotation_matrix, jacobian = cv2.Rodrigues(rvec_flipped)
        realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
        #affiche les caractéristiques de la camera pour la fenêtre d'affichage
        tvec_str = "x=%4.0f|y=%4.0f|degres=%4.0f"%(realworld_tvec[0], realworld_tvec[1], math.degrees (yaw))
        xR, yR, ori = positionRobot(ids,realworld_tvec[0], realworld_tvec[1], math.degrees (yaw))
        data = {}
        data["coord_x"] = xR
        data["coord_y"] = yR
        data["angle"] = ori
        
        
        loopNumber = loopNumber + 1
        if(loopNumber == 15):
            loopNumber = 0
            pc.setDataToFile("updatePosition.json", data)
            print(xR, yR, ori)
            print("coord_x" + str(data["coord_x"]))
            print("coord_y" + str(data["coord_y "]))
            print("angle" + str(data["angle"]))

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break

cap.release()
cv2.destroyAllwindows()
