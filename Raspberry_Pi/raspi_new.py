import cv2
import torch
import serial
import time
import numpy as np
import math
import argparse
import os
from ultralytics import YOLO

from utils import parse_args  # import the parse_args function from utils.py

class yolocarcontroller:
    def __init__(self, args, model_path, serial_port, baud_rate=9600, conf_thres=0.25, iou_thres=0.45):

        # initialize the yolocarcontroller class with parserd arguments
        self.args = args

        # load the yolo model for object detection
        try:
            # attempt to load yolo model from specified path
            self.model = YOLO(model_path)  
            # print confirmation of successful model loading
            print(f"model loaded: {model_path}")  
        except Exception as e:
             # raise error if model loading fails
            raise RuntimeError(f"failed to load model: {e}") 

        # initialize serial communication with arduino
        self.ser = None  # initialize serial connection as None
        if serial_port != 'demo':  # check if serial port is not in demo mode
            try:
                self.ser = serial.Serial(serial_port, baud_rate, timeout=5)  # establish serial connection
                time.sleep(2)  # wait 2 seconds for serial connection to stabilize
                print(f"serial connected: {serial_port}")  # print confirmation of serial connection
            except Exception as e:
                print("failed to connect serial, enter demo mode")  # print failure message and switch to demo mode
                self.ser = None  # set serial connection to None in demo mode

    def wait_for_arduino_response(self):

        # check if serial connection exists
        if self.ser:
            # check if there is data waiting in the serial buffer
            if self.ser.in_waiting:
                # read and decode the incoming message
                line = self.ser.readline().decode('utf-8').strip()
                # extract the first character of the message
                firstchar = line[0]
                
                # check if message starts with 'M' (mode command)
                if firstchar == 'M':
                    # extract the message type after 'M:'
                    type = line[2:]
                    
                    # check if message is 'start'
                    if type == "start":
                        # print start command
                        print("received 'start' from arduino")
                        # enable auto mode
                        self.args.is_auto = True
                        # reset stop auto flag
                        self.args.stop_auto = False
                        # return True to indicate auto mode start
                        return True
                    
                    # check if message is 'stop'
                    elif type == "stop":
                        # print stop command
                        print("received 'stop' from arduino")
                        # set stop auto flag
                        self.args.stop_auto = True  
                        # return False to indicate auto mode stop
                        return False  
                    
                elif firstchar == "p":  # check if message starts with 'p' (position command)
                    self.args.action_complete = True  # mark current action as complete
                    if line == "p_f":  # check if message is 'p_f' (position forward complete)
                        self.args.target_on = False  # reset target tracking flag
                    return True  # return True to indicate action completion
                
                # check if message starts with 'c' (cruise command)
                elif firstchar == "c": 
                    self.args.action_complete = True  # mark current action as complete
                    self.args.cruise_complete = True  # mark cruise action as complete
                    return True  # return True to indicate cruise completion
                elif line == 'w_a':  # check if message is 'w_a' (waiting acknowledgment)
                    return None  # return None to indicate connection check
                else:
                    print(f"received '{line}' from arduino")  # print any other received message
                    return line  # return the raw message for further processing

    def send_command(self, command):
        # Function to send a command to Arduino via serial
        if self.ser:  # Check if serial connection exists
            self.ser.write((command + "@").encode('utf-8'))  # Send command with '@' terminator, encoded as UTF-8
            print(f"sent command: {command}")  # Print confirmation of sent command
        time.sleep(self.args.command_delay)  # Wait for the specified command delay (0.1 seconds)

    def compute_ball_position(self, x, y, ball_box_size):
        """
        Compute the ball's position in 3D space using similar triangles.
        :param x: x-coordinate of the ball center in pixels
        :param y: y-coordinate of the ball center in pixels
        :param ball_box_size: width of the ball's bounding box in pixels
        :return: tuple (d, d_ground, x_actual, forward_distance) in meters
        """
        # calculate camera center coordinate
        cx = self.args.img_width / 2.0
        cy = self.args.img_height / 2.0

        a = ball_box_size / self.args.actual_ball_size  # Calculate scaling factor 'a' (pixel size / real size)
        d = self.args.EFL / a  # Calculate distance from camera to ball using focal length and scaling factor
        d_ground = math.sqrt(max(d**2 - self.args.h**2, 0))  # Calculate ground distance using Pythagorean theorem
        rel_x_pixels = x - cx  # Calculate lateral offset in pixels from camera center
        x_actual = rel_x_pixels / a  # Convert pixel offset to actual distance in meters
        if abs(x_actual) < self.args.align_threshold:  # Check if lateral offset is within alignment threshold
            forward_distance = d_ground  # Set forward distance to ground distance if aligned
        else:
            forward_distance = math.sqrt(max(d_ground**2 - x_actual**2, 0))  # Adjust forward distance for offset
        return d, d_ground, x_actual, forward_distance  # Return all calculated distances

    def opencv_circle(self, img):
        """
        Detect circles in an image using OpenCV's HoughCircles (not currently used in main loop).
        :param img: input image frame
        :return: detected circles or None if no circles found
        """
        img = cv2.resize(img, None, fx=0.5, fy=0.5)  # Resize image to half size for faster processing
        GrayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale
        GrayImage = cv2.blur(GrayImage, (7, 7))  # Apply blur to reduce noise
        circles = cv2.HoughCircles(GrayImage, cv2.HOUGH_GRADIENT, 1, minDist=20, param1=70, param2=50, minRadius=20, maxRadius=80)  # Detect circles
        return circles  # Return detected circles

    def run_camera_loop(self):
        # main loop to process camera frames and control the car
        try:
            while True:  # start infinite loop to continuously check arduino and process frames
                self.send_command('w')  # send 'w' command to check arduino connection
                response = self.wait_for_arduino_response()  # wait for arduino response
                if response is None and not self.args.is_auto:  # check if connection is valid in manual mode
                    print("Connection valid")  # print confirmation of valid connection
                elif response == True:  # check if auto mode is triggered
                    print("Enter auto mode.")  # print confirmation of entering auto mode
                    while True:  # start inner loop for auto mode operation
                        self.wait_for_arduino_response()  # continuously check arduino responses
                        if (self.args.is_auto and not self.args.target_on and self.args.action_complete) or self.args.first_time:  # check if ready to process a frame
                            cap = cv2.VideoCapture(0)  # open camera at index 0
                            if self.args.first_time:  # check if this is the first iteration
                                self.args.first_time = False  # reset first-time flag
                            self.args.action_complete = False  # reset action completion flag
                            self.args.cruise_complete = False  # reset cruise completion flag
                            self.args.both_side = False  # reset both sides flag
                            self.args.have_left = False  # reset left side flag
                            self.args.have_right = False  # reset right side flag
                            ret, frame = cap.read()  # capture a frame from the camera
                            yolo_balls = self.process_frame(frame)  # process frame to detect balls using yolo
                            cap.release()  # release the camera resource
                            cv2.destroyAllWindows()  # close any opencv windows
                            balls = yolo_balls  # assign detected balls to variable
                            print(balls)  # print list of detected balls
                            if len(balls) != 0:  # check if any balls were detected
                                best_ball, In = self.select_ball(balls)  # select the best ball to track
                                print(In)  # print whether the ball is within target range
                                if In:  # check if ball is within target range
                                    while True:  # start loop to wait for action completion
                                        self.wait_for_arduino_response()  # check arduino response
                                        if not self.args.target_on and self.args.action_complete:  # check if action is complete
                                            break  # exit loop if action is complete
                                        if self.args.stop_auto:  # check if auto mode should stop
                                            self.args.first_time = True  # reset first-time flag
                                            self.args.target_on = False  # reset target tracking flag
                                            self.args.action_complete = True  # set action completion flag
                                            self.args.is_auto = False  # disable auto mode
                                            break  # exit loop if stopping auto mode
                                else:
                                    command = self.choose_command(best_ball)  # choose command based on ball position
                                    self.send_command(command)  # send command to arduino
                            else:
                                print(self.args.c_cmd)  # print current cruise command if no balls detected
                                self.send_command(self.args.c_cmd)  # send cruise command to arduino
                                while True:  # start loop to wait for cruise completion
                                    self.wait_for_arduino_response()  # check arduino response
                                    if self.args.stop_auto:  # check if auto mode should stop
                                        break  # exit loop if stopping auto mode
                                    if self.args.cruise_complete:  # check if cruise action is complete
                                        print(self.args.count)  # print cruise action counter
                                        if self.args.count == 6:  # check if counter reaches 6
                                            self.args.c_cmd = "C-F"  # switch to cruise forward
                                            self.args.count = 0  # reset counter
                                        elif self.args.c_cmd == "C-F":  # check if currently cruising forward
                                            self.args.c_cmd = "C-R"  # switch back to cruise right
                                        elif self.args.c_cmd == "C-R":  # check if currently cruising right
                                            self.args.c_cmd = "C-R"  # continue cruising right
                                            self.args.count += 1  # increment counter
                                        break  # exit loop after updating cruise command
                        elif self.args.stop_auto:  # check if auto mode should stop
                            self.args.first_time = True  # reset first-time flag
                            self.args.target_on = False  # reset target tracking flag
                            self.args.action_complete = True  # set action completion flag
                            self.args.is_auto = False  # disable auto mode
                            self.args.stop_auto = False  # reset stop auto flag
                            break  # exit inner loop
        finally:
            cap.release()  # release camera resource in case of exception
            cv2.destroyAllWindows()  # close any opencv windows
            if self.ser:  # check if serial connection exists
                self.ser.close()  # close serial connection

    def process_frame(self, frame):
        # function to process a frame and detect balls using yolo
        results = self.model.predict(source=frame)  # run yolo model prediction on the frame
        det = self.process_detections(results[0])  # process detection results
        balls = []  # initialize empty list to store ball data
        if det is not None and len(det) > 0:  # check if detections exist
            for *cls, conf, box in det:  # iterate over each detection
                if cls[0] == 'ball':  # check if detected object is a ball (class 'ball')
                    box = [b.item() for b in box]  # convert box coordinates to python scalars
                    x = (box[0] + box[2]) / 2.0  # calculate ball center x-coordinate
                    y = (box[1] + box[3]) / 2.0  # calculate ball center y-coordinate
                    ball_box_size = abs(box[2] - box[0])  # calculate ball bounding box width
                    try:
                        d, d_ground, x_actual, forward_distance = self.compute_ball_position(x, y, ball_box_size)  # compute ball position
                        balls.append((d, d_ground, x_actual, forward_distance, ball_box_size))  # add ball data to list
                    except Exception as e:
                        print("compute position error:", e)  # print error if position computation fails
        return balls  # return list of detected balls

    def select_ball(self, balls):
        # Function to select the best ball to track from detected balls
        for ball in balls:  # Iterate over each detected ball
            d, d_ground, x_actual, forward_distance, ball_box_size = ball  # Unpack ball data
            if x_actual > -0.1:  # Check if ball is on the right side
                self.args.have_right = True  # Set right side flag
            if x_actual < -0.1:  # Check if ball is on the left side
                self.args.have_left = True  # Set left side flag
            if self.args.have_left and self.args.have_right:  # Check if balls are on both sides
                self.args.both_side = True  # Set both sides flag
            if x_actual < self.args.left_offset and x_actual > self.args.right_offset:  # Check if ball is within target range
                self.args.target_on = True  # Set target tracking flag
                self.send_command(f"P_F,dist={forward_distance:.2f}")  # Send forward command with distance
                In = True  # Set flag indicating ball is in target range
                break  # Exit loop after finding target ball
            else:
                In = False  # Set flag indicating ball is not in target range
        sorted_balls = sorted(balls, key=lambda b: b[4])  # Sort balls by box size (larger is closer)
        best_ball = sorted_balls[-1]  # Select the largest (closest) ball as best
        print(f"have right:{self.args.have_right}, have left:{self.args.have_left}, both side:{self.args.both_side}")  # Print ball position flags
        print(f"ball detected: d={d:.2f} m, d_ground={d_ground:.2f} m, x_actual={x_actual:.2f} m, forward_distance={forward_distance:.2f} m, ball_box_size={ball_box_size:.2f} mm")  # Print ball position details
        return best_ball, In  # Return best ball and target status

    def choose_command(self, best_ball):
        # Function to choose a command based on the best ball's position
        _, _, _, forward_distance, _ = best_ball  # Unpack best ball data, only need forward distance
        if self.args.both_side:  # Check if balls are on both sides
            return f"P_R,dist={forward_distance:.2f}"  # Return command to pivot right
        elif self.args.have_left:  # Check if ball is on the left
            return f"P_L,dist={forward_distance:.2f}"  # Return command to pivot left
        elif self.args.have_right:  # Check if ball is on the right
            return f"P_R,dist={forward_distance:.2f}"  # Return command to pivot right

    def process_detections(self, results):
        """
        process yolo detection results to extract balls and obstacles.
        :param results: yolo prediction results
        :return: list of detections with label, center, and bounding box
        """
        detections = []  # initialize empty list for detections
        boxes = results.boxes  # extract bounding boxes from results
        for i in range(len(boxes.cls)):  # iterate over each detected object
            cls = int(boxes.cls[i].item())  # get class id as integer
            conf = boxes.conf[i].item()  # get confidence score
            xyxy = boxes.xyxy[i].cpu().numpy()  # get bounding box coordinates as numpy array
            label = "ball" if cls == 0 else "obstacle"  # assign label based on class id (0 = ball)
            center = self.compute_box_center(xyxy)  # compute center of bounding box
            detections.append((label, center, xyxy))  # add detection to list
        return detections  # return list of detections

    def compute_box_center(self, box):
        """
        compute the center of a bounding box.
        :param box: bounding box coordinates [x1, y1, x2, y2]
        :return: tuple (center_x, center_y)
        """
        x1, y1, x2, y2 = box  # unpack bounding box coordinates
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0  # return center coordinates

def main():
    # main function to parse arguments and run the controller
    args = parse_args()  # parse command-line arguments

    # create controller instance with parsed arguments
    controller = yolocarcontroller(
        args=args,
        model_path=args.model,
        serial_port=args.serial,
        baud_rate=args.baud_rate,
        conf_thres=args.conf_thres,
        iou_thres=args.iou_thres
    )

    # start the main camera loop
    controller.run_camera_loop()

if __name__ == "__main__":
    main()  # run main function if script is executed directly