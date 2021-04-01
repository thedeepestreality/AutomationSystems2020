import pybullet as p
import pybullet_data
import numpy as np
from camera import Camera
import cv2
import math

marker_urdf_path= "./marker.urdf"

physicsClient = p.connect(p.DIRECT)#or p.GUI for graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10) 
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF(marker_urdf_path, useFixedBase=True)

fov = 126.8
sz = {
    'width': 2000,
    'height': 2000
}
camera = Camera(size=sz, fov=fov)
data = camera.get_frame()
# RGB -> BGR UMat for opencv
img = np.asarray(data[:,:,[2,1,0]], dtype=np.uint8)
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

corners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)

img = cv2.UMat(img)
cv2.aruco.drawDetectedMarkers(img, corners, markerIds)

focal_length = 1/math.tan(math.radians(fov)/2)
focal_length = sz['height']/4
print(f"focal: {focal_length}")

print(f"corners: {corners}")

markerLength = 3*0.2/4
distCoeffs = np.zeros([0,0,0,0])
cameraMatrix = np.array([[focal_length, 0, sz['width']/2],
                         [0, focal_length, sz['height']/2],
                         [0, 0, 1]])
rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(	corners, markerLength, cameraMatrix, distCoeffs)
print(tvecs)

res = cv2.resize(img,(int(sz['width']/4),int(sz['height']/4)))
cv2.imshow('Marker', res)

# wait for key press
cv2.waitKey(0) 
cv2.destroyAllWindows()

p.disconnect()
