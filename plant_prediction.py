# The steps implemented in the object detection sample code: 
# 1. for an image of width and height being (w, h) pixels, resize image to (w', h'), where w/h = w'/h' and w' x h' = 262144
# 2. resize network input size to (w', h')
# 3. pass the image to network and do inference
# (4. if inference speed is too slow for you, try to make w' x h' smaller, which is defined with DEFAULT_INPUT_SIZE (in object_detection.py or ObjectDetection.cs))
import sys
import threading
from tflite_runtime.interpreter import Interpreter
import numpy as np
from PIL import Image
from object_detection import ObjectDetection
# from motor_motions import Motors
import cv2
import helper
import time
# to send numbers to Arduino
import serial

# model file name and label file name
MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'

ARM_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_3433332383235121B092-if00"  # Robotic Arm Arduino
BAUD_RATE = 115200

# define centers of detected object as a global variable, which contains the angle of servos
centers = []

class TFLiteObjectDetection(ObjectDetection):
    """Object Detection class for TensorFlow Lite"""
    def __init__(self, model_filename, labels):
        super(TFLiteObjectDetection, self).__init__(labels)
        self.interpreter = Interpreter(model_path=model_filename)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]['index']
        self.output_index = self.interpreter.get_output_details()[0]['index']

    def predict(self, preprocessed_image):
        inputs = np.array(preprocessed_image, dtype=np.float32)[np.newaxis, :, :, (2, 1, 0)]  # RGB -> BGR and add 1 dimension.

        # Resize input tensor and re-allocate the tensors.
        self.interpreter.resize_tensor_input(self.input_index, inputs.shape)
        self.interpreter.allocate_tensors()
        
        self.interpreter.set_tensor(self.input_index, inputs)
        self.interpreter.invoke()
        return self.interpreter.get_tensor(self.output_index)[0]         
        
def main(port, baud_rate, image_filename=''):
    global centers
    global sending_actions
    sending_actions = False
    
    # Load labels
    with open(LABELS_FILENAME, 'r') as f:
        labels = [l.strip() for l in f.readlines()]

    od_model = TFLiteObjectDetection(MODEL_FILENAME, labels)
    
    video_object = cv2.VideoCapture(0)
    
    previous = time.time()
    delta = 0
    found = False
    start = True
    t_0 = 0
    # the time interval between detections (s)
    t_wait = 2
    # the time image shows on the screen (ms)
    # if an object is detected, the display would pause at that frame for t_show ms before continuing
    t_show = 1000
    
    # intialize motors
    # motor = Motors()
    
    #try:
    frameNumber = 0
    while(start):
        if found and time.time() - t_0 > t_show/1000:
            found = False
            previous = time.time()
        while(not found):
            # Get the current time, increase delta and update the previous variable
            current = time.time()
            delta += current - previous
            previous = current
            
            ret,frame = video_object.read()
            
            if ret:
                assert not isinstance(frame,type(None)), 'frame not foud'
            else:
                break
            
            frameNumber += 1
            print("frame: {0:d}".format(frameNumber))
            print("delta: {0:04f}".format(delta))
            
            # Update orientation based on EXIF tags, if the file has orientation info.
            image = helper.update_orientation(frame)

            # Convert to OpenCV format
            #image = helper.convert_to_opencv(image)
                
            # If the image has either w or h greater than 1600 we resize it down respecting
            # aspect ratio such that the largest dimension is 1600
            
            image = helper.resize_down_to_1600_max_dim(image)
            
            # We next get the largest center square
            h, w = image.shape[:2]
            min_dim = min(w,h)
            max_square_image = helper.crop_center(image, min_dim, min_dim)
            
            # Resize that square down to 512x512
            augmented_image = helper.resize_to_512_square(max_square_image)
            
            # Check if t_wait (or some other value) seconds passed
            if delta > t_wait and not sending_actions:
                # Operations on image
                #image = Image.open(image_filename)
                
                # Find objects in an image
                predictions = od_model.predict_image(Image.fromarray(augmented_image))
                # all objects found are saved in "predictions", which is a list of dict
                # [{"probability": xx, "tag": xx, "boundingBox": xx, ...}, {}, ...]
                # probability is the probability of the detection being correct.
                # it will be used to select detections if there are two many objects detected.
                font = cv2.FONT_HERSHEY_SIMPLEX
                
                # Looping through number of predictions
                for pred in predictions:
                    if pred['probability'] >= 0.5: # CHANGED PRObAbILITY TO 50
                        found = True
                        # Draw rectangle for each bounding box based on left, top pixel + width and height
                        topleft = (int(pred['boundingBox']['left'] * augmented_image.shape[0]), int(pred['boundingBox']['top'] * augmented_image.shape[1]))
                        bottomright = (int(topleft[0] + pred['boundingBox']['width'] * augmented_image.shape[0]), int(topleft[1] + pred['boundingBox']['height'] * augmented_image.shape[0]))
                        # print(topleft)
                        # print(bottomright)
                        
                        ######################################
                        # to save the object location
                        # compute the center of boxes
                        x = (topleft[0] + bottomright[0]) // 2
                        y = (topleft[1] + bottomright[1]) // 2
                        # print(f"center: ({x:d}, {y:d})")
                        
                        centers.append((x, y))
                        ######################################
                        
                        # text to put
                        text=f"{pred['tagName']} | {round(pred['probability'] * 100, 2)}%" 
                        
                        # draw rectangle and text on img
                        cv2.rectangle(augmented_image, topleft, bottomright, (255, 0 ,0), 2)
                        cv2.putText(augmented_image, text, topleft, font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                
                        # motor controls
                        # motor.motor_control(motor.check_species(pred['tagName']))
                        # start = False
                    
                if found:
                    cv2.imshow('Frames',augmented_image)
                    cv2.waitKey(t_show)  
                    t_0 = time.time()
                    #command_arduino(port, baud_rate)
                    send_tag(f"{pred['tagName']}")
                # Reset the time counter
                delta = 0
                
            
            # show the frame on the newly created image window
            if not found:
                cv2.imshow('Frames',augmented_image)
                cv2.waitKey(1)
    
        
    #except:
       # print("Video has ended..")
"""
Below are the new functions for RP-Arduino communications.
"""

def inverse_kinematics(x, y):
    """
    Calculate the inverse kinematics for a robotic arm to reach the detected object, based on the coordinate of the object.
    The current code returns a list of four random angles.
    Students need to implement a real inverse kinematics calculation to determine the conversion between x, y and angles.
    
    Args
    x, y -- coordinates of the center of the detected object.
    
    Returns
    angles -- 4 angles of the servo motors that specifies a gesture of the robotic arm.
    """
    
    # Your code goes hrere
    angles = (np.random.rand(4) * 60 + 60).astype(int)
    
    return angles

# Function to send a string to the Arduino
def send_tag(tag):
    """
    Write an integer to the serial buffer to communicate with Arduino.
    """
    global sending_actions
    arduino = connect_to_arduino(port, baud_rate)
    sending_actions = True
    arduino.write(f"{tag}\n".encode())  # Send the number followed by a newline
    print(f"Sent: {tag}")
    
    # Wait for the Arduino's response
    while True:
        response = arduino.readline().decode("utf-8").strip()
        if response == "DONE":
            print("Arduino completed the action.")
            sending_actions = False
            break

# Function to send a number to the Arduino
def send_number(number):
    """
    Write an integer to the serial buffer to communicate with Arduino.
    """
    global sending_actions
    sending_actions = True
    arduino.write(f"{number}\n".encode())  # Send the number followed by a newline
    print(f"Sent: {number}")
    
    # Wait for the Arduino's response
    while True:
        response = arduino.readline().decode("utf-8").strip()
        if response == "DONE":
            print("Arduino completed the motion.")
            sending_actions = False
            break
        
def connect_to_arduino(port, baud_rate, timeout=1):
    """
    Attempt to connect to the Arduino repeatedly until successful.
    
    Parameters:
    port (str): The serial port to connect to (e.g., 'COM3' or '/dev/ttyUSB0').
    baud_rate (int): The baud rate for the serial communication.
    timeout (int): The timeout for the serial connection in seconds.
    
    Returns:
    serial.Serial: The connected serial object.
    """
    while True:
        try:
            # Attempt to create a serial connection
            arduino = serial.Serial(port, baud_rate, timeout=timeout)
            print(f"Connected to Arduino on port {port} at {baud_rate} baud.")
            return arduino
        except serial.SerialException as e:
            # If connection fails, print the error and retry after a short delay
            print(f"Failed to connect to Arduino: {e}")
            print("Retrying in 2 seconds...")
            time.sleep(2)

def command_arduino(port, baud_rate):
    """
    Listen for (x, y) coordinates, convert them to angles using inverse kinematics,
    and send the angles to the Arduino.
    """
    global centers
    arduino = connect_to_arduino(port, baud_rate)
    while True:
        if len(centers) > 0:
            while centers:
                # pop from the beginning
                center = centers.pop(0)
                print(f"Found object at ({center[0]}, {center[1]})")
                angles = inverse_kinematics(*center)
                print("angles: " + ", ".join(map(str, angles)))
                # send angles to Arduino
                for angle in angles:
                    send_number(angle)    
        else:
            time.sleep(1)

if __name__ == '__main__':
    video_path = "output.mp4"
    port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_3433332383235121B092-if00"  # Robotic Arm Arduino
    baud_rate = 115200
    #main_thread = threading.Thread(target=main, args=(video_path,))
    #command_arduino_thread = threading.Thread(target=command_arduino, args=(port, baud_rate))
    #main_thread.start()
    #command_arduino_thread.start()
    main(port, baud_rate, video_path)
    
    
