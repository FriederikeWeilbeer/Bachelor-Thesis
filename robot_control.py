# importing the modules
import cv2
import cv2.aruco as aruco
import zmq

DEBUG = False


# sets up zmq publisher
def setUpPublisher():
    global socket
    port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:%s" % port)


# returns coordinates of the corners and id of the aruco markers
def findArucoMarkers(img, arucoDict, arucoParam):
    # convert the video image to a gray image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    # draw boxes around detected markers
    aruco.drawDetectedMarkers(img, corners)
    return corners, ids

# keyboard listener
def keyboardListener():
    global socket
    k = cv2.waitKey(4) & 0xFF
    # left arrow key to turn left
    if k == 81:
        socket.send_string("14 left")
    # right arrow key to turn right
    if k == 83:
        socket.send_string("14 right")
    # 's' key to start the robot
    if k == ord('s'):
        socket.send_string("14 start")


# send corner coordinates with id as strings
def sendPosition(corners, ids):
    global socket
    try:
        for i in range(0, len(ids)):
            socket.send_string(str(ids[i][0]) +
                               " " + str(corners[i][0][0][0]) + " " + str(corners[i][0][0][1]) +
                               " " + str(corners[i][0][1][0]) + " " + str(corners[i][0][1][1]) +
                               " " + str(corners[i][0][2][0]) + " " + str(corners[i][0][2][1]) +
                               " " + str(corners[i][0][3][0]) + " " + str(corners[i][0][3][1]))
    except Exception as err:
        if DEBUG:
            print("No markers found.")



def main():
    global socket, cooX, cooY
    socket = None
    cooX = None
    cooY = None
    setUpPublisher()
    # setup aruco Dictionary
    key = getattr(aruco, f'DICT_{6}X{6}_{250}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()

    # setup video capture
    cap = cv2.VideoCapture(0)

    while True:
        # get video image
        success, img = cap.read()
        # reading the image
        corners, ids = findArucoMarkers(img, arucoDict, arucoParam)

        # keyboard listener
        keyboardListener()

        # displaying the image
        cv2.imshow('image', img)

        sendPosition(corners, ids)
        # wait for a key to be pressed to exit
        k = cv2.waitKey(4) & 0xff
        if k == 27 or k == ord('q'):
            socket.send_string("42 quit")
            print("send quit")
            break

    # close the window
    cv2.destroyAllWindows()


# driver function
if __name__ == "__main__":
    main()
