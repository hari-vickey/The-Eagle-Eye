import numpy as np
import cv2

def crop_arena(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 50, 255,cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    areals = []
    for i in range(len(contours)):
        cnt = contours[i]
        area=cv2.contourArea(cnt)
        areals.append(area)
    ind = areals.index(max(areals))
    del areals[ind]
    del contours[ind]
    in1 = areals.index(max(areals))
    rect = cv2.minAreaRect(contours[in1])
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    frame = frame[box[1][1]:box[3][1], box[1][0]:box[3][0]]
    return thresh
image = cv2.imread("test_real.png")
croped_image = crop_arena(image)
cv2.imshow("Arena", croped_image)
cv2.waitKey()
cv2.destroyAllWindows()
