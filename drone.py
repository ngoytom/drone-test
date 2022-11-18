from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import cv2, math, time

# Speed of the drone
S = 60
FPS = 120

width = 640 
height = 480 
deadZone = 100 

frameWidth = width
frameHeight = height

global imgContour
global dir;
def empty(a):
    pass

class FrontEnd(object):
    """ Control Drone Through Keyboard
        T: Takeoff
        L: Land
        Arrow keys: Forward, backward, left and right.
        A + D: Rotations (yaw)
        W + S: Up and down.
    """

    def __init__(self):
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        self.tello = Tello()

        # Drone velocitiy
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = False

        cv2.namedWindow("HSV")
        cv2.resizeWindow("HSV",640,240)
        cv2.createTrackbar("HUE Min","HSV",20,179,empty)
        cv2.createTrackbar("HUE Max","HSV",40,179,empty)
        cv2.createTrackbar("SAT Min","HSV",148,255,empty)
        cv2.createTrackbar("SAT Max","HSV",255,255,empty)
        cv2.createTrackbar("VALUE Min","HSV",89,255,empty)
        cv2.createTrackbar("VALUE Max","HSV",255,255,empty)
 
        cv2.namedWindow("Parameters")
        cv2.resizeWindow("Parameters",640,240)
        cv2.createTrackbar("Threshold1","Parameters",166,255,empty)
        cv2.createTrackbar("Threshold2","Parameters",171,255,empty)
        cv2.createTrackbar("Area","Parameters",1750,30000,empty)

        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    def getContours(self,img,imgContour):
        global dir
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            areaMin = cv2.getTrackbarPos("Area", "Parameters")
            if area > areaMin:
                cv2.drawContours(imgContour, cnt, -1, (255, 255, 0), 7)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                #print(len(approx))
                x , y , w, h = cv2.boundingRect(approx)
                cx = int(x + (w / 2)) 
                cy = int(y + (h / 2)) 
    
                if (cx <int(frameWidth/2)-deadZone):
                    cv2.putText(imgContour, " left " , (20, 50), cv2.FONT_HERSHEY_PLAIN,2,(0, 0, 255), 3)
                    dir = 1
                elif (cx > int(frameWidth / 2) + deadZone):
                    cv2.putText(imgContour, " right ", (20, 50), cv2.FONT_HERSHEY_PLAIN,2,(0, 0, 255), 3)
                    dir = 2
                elif (cy < int(frameHeight / 2) - deadZone):
                    cv2.putText(imgContour, " up ", (20, 50), cv2.FONT_HERSHEY_PLAIN,2,(0, 0, 255), 3)
                    dir = 3
                elif (cy > int(frameHeight / 2) + deadZone):
                    cv2.putText(imgContour, " down ", (20, 50), cv2.FONT_HERSHEY_PLAIN, 2,(0, 0, 255), 3)
                    dir = 4
                else: dir=0
    
                cv2.rectangle(imgContour, (x , y ), (x + w , y + h ), (0, 255, 0), 5)
            
    def run(self):
        self.tello.connect()
        self.tello.set_speed(self.speed)

        self.tello.streamoff()
        self.tello.streamon()

        frame_read = self.tello.get_frame_read()

        while True:
            # Get image from Tello
            myFrame = frame_read.frame
            img = cv2.resize(myFrame, (width, height))
            imgContour = img.copy()
            imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos("HUE Min","HSV")
            h_max = cv2.getTrackbarPos("HUE Max", "HSV")
            s_min = cv2.getTrackbarPos("SAT Min", "HSV")
            s_max = cv2.getTrackbarPos("SAT Max", "HSV")
            v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
            v_max = cv2.getTrackbarPos("VALUE Max", "HSV")
 
            lower = np.array([h_min,s_min,v_min])
            upper = np.array([h_max,s_max,v_max])
            mask = cv2.inRange(imgHsv,lower,upper)
            result = cv2.bitwise_and(img,img, mask = mask)
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
 
            imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
            imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
            threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
            threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
            imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
            kernel = np.ones((5, 5))
            imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
            self.getContours(imgDil, imgContour)

            cv2.imshow("dill", imgDil)
            cv2.imshow("contour", imgContour)

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                #elif event.type == pygame.QUIT:
                #    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    #if event.key == pygame.K_ESCAPE:
                    #    should_stop = True
                    #else:
                    self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                break

            self.screen.fill([0, 0, 0])

            time.sleep(1 / FPS)

        # deallocate resources
        self.tello.end()

    # update velocities based on key pressed
    def keydown(self, key):
        if key == pygame.K_UP:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_DOWN:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_s:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    # update velocities based on key released
    def keyup(self, key):
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.up_down_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.for_back_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            not self.tello.land()
            self.send_rc_control = False

    # send velocities to Tello.
    def update(self):
        if dir == 1:
            me.yaw_velocity = -S
        elif dir == 2:
            me.yaw_velocity = S
        elif dir == 3:
            me.up_down_velocity= S
        elif dir == 4:
            me.up_down_velocity= -S
        else:
            me.left_right_velocity = 0; me.for_back_velocity = 0; me.up_down_velocity = 0; me.yaw_velocity = 0
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                self.up_down_velocity, self.yaw_velocity)

def main():
    frontend = FrontEnd()
    frontend.run()

if __name__ == '__main__':
    main()