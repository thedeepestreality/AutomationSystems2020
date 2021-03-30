import pybullet as p
import pybullet_data
import numpy as np
from camera import Camera
import cv2

marker_urdf_path= "./marker.urdf"

physicsClient = p.connect(p.DIRECT)#or p.GUI for graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10) 
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF(marker_urdf_path, useFixedBase=True)

camera = Camera()
data = camera.get_frame()
# RGB -> BGR UMat for opencv
img = cv2.UMat(np.asarray(data[:,:,[2,1,0]]))
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
cv2.aruco.drawDetectedMarkers(img, corners, markerIds)

# markerLength = 1
# distCoeffs = np.zeros([0,0,0,0])
# cameraMatrix = np.array([[1,0,100],[0,1,100],[0,0,1]])
# rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(	corners, markerLength, cameraMatrix, distCoeffs)

cv2.imshow('Marker', img)

# wait for key press
cv2.waitKey(0) 
cv2.destroyAllWindows()

p.disconnect()
