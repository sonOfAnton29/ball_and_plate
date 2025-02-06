import cv2
import numpy as np


def nothing(x):
    pass


class Ball_handler:

    def __init__(self):
        cv2.namedWindow("Processed Frame", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("pid_menu", cv2.WINDOW_AUTOSIZE)

        cv2.createTrackbar("on/off", "pid_menu", 0, 1, nothing)

        # to count frames with no detection / false negatives
        self.no_det_count = 0

        self.last_pos = (0, 0)
        

    def ball_finder(self, frame, mouse_pos):

        ball_center = (-1, -1)

        # crop frame to reduce the risk of detecting outliers 
        frame = frame[30 : frame.shape[0] - 5, 75 : frame.shape[1] - 95] 

        ### NOTE
        ###!!! the commented code is when you need to warp the frame !!!###

        # # Define source points (the four corners of the region to warp)
        # src_pts = np.float32([[20, 20], [frame.shape[0] - 20, 20], [20, frame.shape[1]-20], [frame.shape[0] - 20, frame.shape[1] - 20]])

        # # Define destination points (where you want to warp the image)
        # dst_pts = np.float32([[0, 0], [500, 0], [0, 500], [500, 500]])

        # # Get perspective transformation matrix
        # M = cv2.getPerspectiveTransform(src_pts, dst_pts)

        # frame = cv2.warpPerspective(frame, M, (500, 500))
        # cv2.imshow("warped", frame)

        ###!!! end of warp !!!###

        # convert BGR frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # this threshold condition was specific to our need so you may comment it 
        gray[gray > 150] = 255 

        # Apply Gaussian blur to reduce noise
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # Detect circles using Hough Transform
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp = 1.2, minDist = 30,
                                param1 = 50, param2 = 30, minRadius = 28, maxRadius = 37)

        # If circles are detected, draw them
        if circles is not None:
            circles = np.uint16(np.around(circles))  # Round values
            for (x, y, r) in circles[0, :]:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)  # Draw outer circle
                cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)  # Draw center
                frame = cv2.putText(frame, str(f"{r}, {x}, {y}"), (x-20, y-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

                ball_center = [int(x), int(y)]

                self.last_pos = ball_center

        ### NOTE
        ### !!! the commented code is when you want to use Contours instead of HoughCircle algorithm !!!###
        ### some conditions are specific to our situation so may delete or alter ###

        # # Apply Gaussian Blur
        # blurred = cv2.GaussianBlur(gray, (15, 15), 0)

        # # Threshold the image
        # _, thresh = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)

        # # Find contours
        # contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.imshow("cr", thresh)

        # for contour in contours:

        #     if cv2.contourArea(contour) > 50:  # Ignore small objects
        #         # Get bounding box or minimum enclosing circle
        #         (x, y), radius = cv2.minEnclosingCircle(contour)
        #         center = (int(x), int(y))
        #         radius = int(radius)

        #         if (radius < 35 and radius > 25):
        #         #    ( (25 < center[0] < frame.shape[0] - 25) &\
        #         #      (25 < center[1] < frame.shape[1] - 25)): 

        #             ball_radius = radius
        #             ball_center = center
                    
        #             # Draw the circle and center
        #             cv2.circle(frame, center, ball_radius, (0, 255, 0), 2)
        #             # Using cv2.putText() method
        #             image = cv2.putText(frame, str(ball_radius), (center[0]-20, center[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        #             cv2.circle(frame, center, 5, (255, 0, 0), -1)

        #             # Print or send the ball's coordinates
        #             # print(f"Ball position: {center}")

        #             self.last_pos = ball_center

        ####!!! end of Contours !!!###


        cv2.circle(frame, mouse_pos, 5, (0, 0, 255), -1)
        

        cv2.imshow("Processed Frame", frame)
        print(f"this is ball center : {ball_center}")

        # counting frames with no detection
        if ball_center != self.last_pos: 
            self.no_det_count += 1

        # if there were 5 frames with no detection send below data which resets system to ideal state
        if self.no_det_count == 5:
            self.no_det_count = 0
            self.last_pos = (2000, 2000)

        return self.last_pos
