import cv2
import cv2.aruco as aruco

frame = cv2.imread("images/t.jpg")
# frame = cv2.imread("test.jpg")
frame = cv2.resize(frame, (640, 360))

parameters =  cv2.aruco.DetectorParameters_create()

# Detect the markers in the image
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
frame = aruco.drawDetectedMarkers(frame, markerCorners)

print(markerIds, markerCorners)

cv2.imshow("te", frame)
cv2.waitKey()