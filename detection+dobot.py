import cv2
import numpy as np
import DobotDllType as dType


def MoveAndWait(api, mode, x, y, z, r, isQueued):
    dType.SetPTPCmd(api, mode, x, y, z, r, isQueued=isQueued)
    while(True):
        pos = dType.GetPose(api)
        xp = pos[0]
        yp = pos[1]
        zp = pos[2]
        rp = pos[3]
        cond = np.isclose(x, xp, atol=1e-1) and np.isclose(y, yp, atol=1e-1)
        cond = cond and np.isclose(
            z, zp, atol=1e-1) and np.isclose(r, rp, atol=1e-1)
        print(pos[0:4], [x,y,z,r])
        if cond:
            break


def HomeAndWait(api, mode, isQueued):
    dType.SetHOMECmd(api, 1, isQueued=1)
    pHome = dType.GetHOMEParams(api)
    [xh, yh, zh, rh] = pHome[0:4]
    while(True):
        pos = dType.GetPose(api)
        xp = pos[0]
        yp = pos[1]
        zp = pos[2]
        rp = pos[3]
        cond = np.isclose(xh, xp, atol=1e-1) and np.isclose(yh, yp, atol=1e-1)
        cond = cond and np.isclose(
            zh, zp, atol=1e-1) and np.isclose(rh, rp, atol=1e-1)
        if cond:
            break


CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

api = dType.load()
# Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:", CON_STR[state])

# movement points are get quite easily from dobotstudio GetPoint (ctrl+p)
xDrop = [139.6354, 245.4159, 143.8797, 234.8857, 234.8857]
yDrop = [116.3886, 116.1774, 222.3796, 201.7403, 201.7403]
zDrop = [55.6426, 49.5736, 59.0661, 41.5329, 41.5329]
rDrop = [39.8118, 25.3324, 57.0971, 40.6588, 40.6588]

dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, 0)
dType.SetPTPCoordinateParams(api, 200, 200, 200, 200, 0)
dType.SetPTPJumpParams(api, 10, 200, 0)
dType.SetPTPCommonParams(api, 100, 100, 0)
dType.SetEndEffectorSuctionCup(api, enableCtrl=1,  on=0, isQueued=1)
HomeAndWait(api, 1, 1)


def nothing(x):
    # any operation
    pass




cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 0, 180, nothing)
cv2.createTrackbar("L-S", "Trackbars", 66, 255, nothing)
cv2.createTrackbar("L-V", "Trackbars", 134, 255, nothing)
cv2.createTrackbar("U-H", "Trackbars", 180, 180, nothing)
cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U-V", "Trackbars", 243, 255, nothing)
clahe = cv2.createCLAHE(2.0, (5,5))

font = cv2.FONT_HERSHEY_COMPLEX

while True:
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    

    l_h = cv2.getTrackbarPos("L-H", "Trackbars")
    l_s = cv2.getTrackbarPos("L-S", "Trackbars")
    l_v = cv2.getTrackbarPos("L-V", "Trackbars")
    u_h = cv2.getTrackbarPos("U-H", "Trackbars")
    u_s = cv2.getTrackbarPos("U-S", "Trackbars")
    u_v = cv2.getTrackbarPos("U-V", "Trackbars")

    lower_red = np.array([l_h, l_s, l_v])
    upper_red = np.array([u_h, u_s, u_v])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)

    # Contours detection
    if int(cv2.__version__[0]) > 3:
        # Opencv 4.x.x
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    else:
        # Opencv 3.x.x
        _, contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 1000:
            cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            if len(approx) == 3:
                cv2.putText(frame, "Triangle", (x, y), font, 1, (0, 0, 0))
                # triangle

                MoveAndWait(api, 1, 240.0, 60, 19.91, -1.3, isQueued=1)
                MoveAndWait(api, 1, 240.0, 60, -65, -0.58, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=1, isQueued=1)
                MoveAndWait(api, 1, 209, 45, 35, -0.58, isQueued=1)
                MoveAndWait(api, 1, 220, -110, 35, -30.2, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=0, isQueued=1)
                dType.dSleep(1000)
            elif len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                # aspectRatio = float(w)/h
                # print(aspectRatio)
                # if aspectRatio >= 0.95 and aspectRatio < 1.05:
                cv2.putText(frame, "square", (x, y), font, 1, (0, 0, 0))
                MoveAndWait(api, 1, 240.0, 60, 19.91, -1.3, isQueued=1)
                MoveAndWait(api, 1, 240.0, 60, -65, -0.58, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=1, isQueued=1)
                MoveAndWait(api, 1, 209, 45, 35, -0.58, isQueued=1)

                MoveAndWait(api, 1, 160, -100, 35, -30.2,
                            isQueued=1)  # above pickup
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=0, isQueued=1)
                dType.dSleep(1000)
                # else:
                # cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))
                # ###rectangle

                # MoveAndWait(api, 1, 240.0,60,19.91,-1.3, isQueued=1)
                # MoveAndWait(api, 1, 240.0,60,-65,-0.58, isQueued=1)
                # dType.SetEndEffectorSuctionCup(api, enableCtrl=1,  on=1, isQueued=1)
                # MoveAndWait(api, 1, 209,45,35,-0.58, isQueued=1)

                # MoveAndWait(api, 1, 160,-100,35,-30.2, isQueued=1) # above pickup
                # dType.SetEndEffectorSuctionCup(api, enableCtrl=1,  on=0, isQueued=1)
                # dType.dSleep(1000)
            elif 9 < len(approx) < 15:
                cv2.putText(frame, "star", (x, y), font, 1, (0, 0, 0))
            elif len(approx) == 5:

                cv2.putText(frame, "pentagon", (x, y),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                MoveAndWait(api, 1, 240.0, 60, 19.91, -1.3, isQueued=1)
                MoveAndWait(api, 1, 240, 60, -65, -0.58, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=1, isQueued=1)
                MoveAndWait(api, 1, 209, 45, 35, -0.58, isQueued=1)
                MoveAndWait(api, 1, 270, -160, 35, -30.2, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=0, isQueued=1)
                dType.dSleep(1000)
            elif len(approx) == 6:

                cv2.putText(frame, "Hexagone", (x, y),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                MoveAndWait(api, 1, 240.0, 60, 19.91, -1.3, isQueued=1)
                MoveAndWait(api, 1, 240.0, 60, -65, -0.58, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=1, isQueued=1)
                MoveAndWait(api, 1, 209, 45, 35, -0.58, isQueued=1)
                MoveAndWait(api, 1, 270, -160, 35, -30.2, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=0, isQueued=1)
                dType.dSleep(1000)

            # elif 16 < len(approx) < 50:
                #cv2.putText(frame, "Circle", (x, y), font, 1, (0, 0, 0))
            else:

                cv2.putText(frame, "Circle", (x, y), font, 1, (0, 0, 0))
                # circle
                MoveAndWait(api, 1, 240.0, 60, 19.91, -1.3, isQueued=1)
                MoveAndWait(api, 1, 240.0, 60, -65, -0.58, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=1, isQueued=1)
                MoveAndWait(api, 1, 209, 45, 35, -0.58, isQueued=1)
                MoveAndWait(api, 1, 170, -190, 35, -30.2, isQueued=1)
                dType.SetEndEffectorSuctionCup(
                    api, enableCtrl=1,  on=0, isQueued=1)
                dType.dSleep(1000)
                # circle

        else:
            # dType.dSleep(1000)
            continue

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    cap.release()

    key = cv2.waitKey(1)
    if key == 27:
        break

# cap.release()
dType.DisconnectDobot(api)
cv2.destroyAllWindows()
