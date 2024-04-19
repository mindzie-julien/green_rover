import numpy as np

import cv2
import math

import time



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

marker_size = 100

with open ( 'camera_cal.npy', 'rb') as f:
    camera_matrix = np. load(f)
    camera_distortion = np. load(f)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

cap = cv2.VideoCapture(0)
fps = cap.get(cv2.CAP_PROP_FPS) #method to show fps of your video
print(f"{fps=}")
#Taille de la video de la cam
camera_width = 640
camera_height = 480
camera_frame_rate = 20

cap.set(2, camera_width)
cap.set(4, camera_height)
cap.set(5, camera_frame_rate)

while True :
    markerIndex = -1
    ret, frame = cap.read()  # grab a frame
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to Grayscale
    


    # -- Find all the aruco markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, ca>
#    print(corners,ids,rejected)
    markerIndex = []
    
    if ids is not None:
        print(f"{ids}")
        for i in range(ids.size):
            if ids[i] in [20, 21, 22, 23, 47, 13, 36]:
                markerIndex.append(i)
                print(f"{ids[i]=}")


    if markerIndex != []:

        corners = np.array([corners[i] for i in markerIndex])
        ids = np.array([ids[i] for i in markerIndex])

        cv2.aruco.drawDetectedMarkers(gray_frame, corners, ids)  # draw a box a>

        # get pose of all single markers
        rvec_list_all, tvec_list_all, objPoints = cv2.aruco.estimatePoseSingleM>

        rvec = rvec_list_all[0][0]
        tvec = tvec_list_all[0][0]

        cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, >

        rvec_flipped = rvec * -1
        tvec_flipped = tvec * -1
        rotation_matrix, jacobian = cv2.Rodrigues(rvec_flipped)
        realworld_tvec = np.dot(rotation_matrix, tvec_flipped)

        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)

        if(realworld_tvec[0] > 0):
            #affiche les caractéristiques de la camera pour la fenêtre d'affich>
            tvec_str = "x=%4.0f|y=%4.0f|degres=%4.0f"%(realworld_tvec[0], realw>

            print(f"{tvec_str=}")
            #paramètres des changements d'écritures sur la fenêtre d'afichage
            cv2.putText(gray_frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN>

   # cv2.imshow("frame",gray_frame) permet de lancer la fenêtre d'affichage de >
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break

cap.release()
cv2.destroyAllwindows()

