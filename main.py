import cv2
import dlib
import time
from adafruit_servokit import ServoKit

MIN_X_LEFT = 70 #80
MAX_X_LEFT = 50 #35
MIN_Y_LEFT = 90 #90
MAX_Y_LEFT = 55 #40

MIN_X_RIGHT = 125 #140
MAX_X_RIGHT = 100 #100
MIN_Y_RIGHT = 10  #0
MAX_Y_RIGHT = 45  #60

class Robot:
    def __init__(self, camera, detector):
        servos = setupServo()
        self.servoLeftX = servos[0]
        self.servoLeftY = servos[1]
        self.servoRightX = servos[2]
        self.servoRightY = servos[3]
        self.detector = detector
        self.cam = camera
        cv2.CAP_PROP_BRIGHTNESS = 100

    def moveServo(self, servo, value):
        servo.angle = value

    def squint(self):
        self.servoLeftX.angle = 35
        self.servoLeftY.angle = 70
        self.servoRightX.angle = 140
        self.servoRightY.angle = 25
        print("Squinting")

    def setPosition(self, x, y):
        self.servoLeftX.angle = x * (MAX_X_LEFT - MIN_X_LEFT) + MIN_X_LEFT
        print("Servo left X angle:", x * (MAX_X_LEFT - MIN_X_LEFT) + MIN_X_LEFT)
        self.servoLeftY.angle = y * (MAX_Y_LEFT - MIN_Y_LEFT) + MIN_Y_LEFT
        print("Servo left Y angle:", y * (MAX_Y_LEFT - MIN_Y_LEFT) + MIN_Y_LEFT)
        self.servoRightX.angle = x * (MAX_X_RIGHT - MIN_X_RIGHT) + MIN_X_RIGHT
        print("Servo right X angle:", x * (MAX_X_RIGHT - MIN_X_RIGHT) + MIN_X_RIGHT)
        self.servoRightY.angle = y * (MAX_Y_RIGHT - MIN_Y_RIGHT) + MIN_Y_RIGHT
        print("Servo right Y angle:", y * (MAX_Y_RIGHT - MIN_Y_RIGHT) + MIN_Y_RIGHT)
        print("Position set to:", x, y)

def mouseCallback(event, x, y, flags, param):
    robot, mode = param["robot"], param["mode"]

    if event == cv2.EVENT_LBUTTONDOWN:
        param["mode"] = "Tracking" if mode == "Pointing" else "Pointing"
        print(f"Mode changed to: {param['mode']}")
    elif event == cv2.EVENT_RBUTTONDOWN:
        param["mode"] = "Tracking" if mode == "Squinting" else "Squinting"
        print(f"Mode changed to: {param['mode']}")
    elif event == cv2.EVENT_MOUSEMOVE and mode == "Pointing":
        positionX = x/640
        positionY = y/480
        robot.setPosition(positionX, positionY)
        print(f"Tracking coordinates: {x, y}")


def setupServo():
    kit = ServoKit(channels=16)
    servoLeftX = kit.servo[0]
    servoLeftY = kit.servo[1]
    servoRightX = kit.servo[2]
    servoRightY = kit.servo[3]
    print("Servo setup complete")
    return [servoLeftX, servoLeftY, servoRightX, servoRightY]


def getFaceCoordinates(detector, cam):
    ret, frame = cam.read()
    cv2.normalize(frame, frame, 30, 245, cv2.NORM_MINMAX)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)
    if len(faces) > 0:
        x1 = faces[0].left()
        y1 = faces[0].top()
        x2 = faces[0].right()
        y2 = faces[0].bottom()
        return {"frame":frame, "x1":x1, "y1":y1, "x2":x2, "y2":y2}
    return {"frame":frame, "x1":None, "y1":None,"y1":None, "y2":None}


def calculatePosition(x1, x2, y1, y2, frame_width, frame_height):
    positionX = (x1+x2)/2
    positionY = (y1+y2)/2
    X = positionX/frame_width
    Y = positionY/frame_height
    return {"X": X, "Y": Y}

def main():
    try:
        cam = cv2.VideoCapture(0)
    except:
        print("Camera not found")
        exit()

    detector = dlib.get_frontal_face_detector()
    robot = Robot(cam, detector)
    last_time = time.time()

    params = {"mode": "Tracking", "robot": robot}

    cv2.namedWindow('Get Coordinates')
    cv2.setMouseCallback('Get Coordinates', mouseCallback, param=params)
    cv2.resizeWindow('Get Coordiantes', 640, 520)
    print("Starting")
    while True:
        val = getFaceCoordinates(detector, cam)
        frame = val["frame"]
        if params["mode"] == "Tracking":
            if val["x1"] is not None:
                x1, y1, x2, y2 = val["x1"], val["y1"], val["x2"], val["y2"]
                if x1 is not None:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    face_position = calculatePosition(x1, x2, y1, y2, frame.shape[1], frame.shape[0])
                    robot.setPosition(face_position["X"], face_position["Y"])
                    last_time = time.time()
                else:
                    if time.time() - last_time > 5:
                        params["mode"] = "Squinting"
                        robot.squint()
        if params["mode"] == "Squinting":
          robot.squint()    
        cv2.imshow("Get Coordinates", frame)
        if cv2.waitKey(1) & 0xFF == ord("q") or cv2.getWindowProperty('Get Coordinates', cv2.WND_PROP_VISIBLE) < 1:
            break


if __name__ == "__main__":
    main()
