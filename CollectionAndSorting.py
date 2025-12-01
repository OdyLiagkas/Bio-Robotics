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

import os
import math
import signal
import csv

# model file name and label file name
MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'

NAV_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_343313230363514032C1-if00"  # Navigation Arduino
ARM_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_3433332383235121B092-if00"  # Robotic Arm Arduino
BAUD_RATE = 115200
ARM_BAUD = 115200

# define centers of detected object as a global variable, which contains the angle of servos
centers = []
PIXELS_TO_MM = 0.3704
MM_TO_DEGREES = 0.3
CENTER_X, CENTER_Y = 256, 256

# ====== Global Variables ======
pose = [0, 0]
yaw = 0
is_centered = False
exit_event = threading.Event()

# ====== Initialize Serial Ports ======
try:
    arduino_nav = serial.Serial(NAV_SERIAL, BAUD_RATE, timeout=1)
    print("Navigation Arduino connected")
except Exception as e:
    print(f"Navigation Arduino connection failed: {str(e)}")
    arduino_nav = None

try:
    arduino_arm = serial.Serial(ARM_SERIAL, ARM_BAUD, timeout=1)
    print("Robotic Arm Arduino connected")
    time.sleep(2)
    arduino_arm.write(b"ZERO\n")
    wait_success = False
    timeout = time.time() + 2.0
    while time.time() < timeout:
        if arduino_arm.in_waiting:
            line = arduino_arm.readline().decode().strip()
            if line == "BASELINE_RESET":
                wait_success = True
                break
    if wait_success:
        print("Ard_Arm connected")
    else:
        print("Ard_Arm is still problem")
except Exception as e:
    print(f"Robotic Arm Arduino connection failed: {str(e)}")
    arduino_arm = None

# ====== Helper Functions ======
def wait_for(ser, token, timeout=15.0):
    """Blocks until a line equal to the token is read from the serial port."""
    if not ser: return False
    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line == token:
                return True
    return False

def normalize_angle(angle):
    """Normalizes an angle to the -180 to 180 range."""
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle

# ====== Arduino Communication Functions ======
def query_yaw_from_nav():
    """Sends YAW to the Navigation Arduino and returns its angle."""
    global arduino_nav
    if not arduino_nav: return 0
    try:
        arduino_nav.write(b"YAW\n")
        timeout = time.time() + 1.0
        while time.time() < timeout:
            if arduino_nav.in_waiting:
                line = arduino_nav.readline().decode().strip()
                if line.startswith("IMU_YAW="):
                    return normalize_angle(int(line.split('=')[1]))
        print("Warning: Querying IMU angle timed out.")
        return 0
    except Exception as e:
        print(f"Warning: Error querying IMU angle: {str(e)}")
        return 0

def send_nav(cmd, delay=2):
    """Sends a command to the navigation Arduino, waits for 'DONE', and updates robot pose."""
    global pose, yaw, current_yaw
    if not arduino_nav:
        print(f"Warning: Navigation Arduino not connected, cannot send command: {cmd}")
        try_reconnect_serial()
        return
    try:
        yaw_before = None
        if cmd.startswith(("L", "R", "ML", "MR")) and len(cmd) > 1:
            yaw_before = query_yaw_from_nav()
            print(f"Angle before rotation: {yaw_before:.1f} degrees")
        arduino_nav.write(f"{cmd}\n".encode())
        print(f"Sent to Nav: {cmd}")
        if not wait_for(arduino_nav, "DONE", timeout=15.0):
            print(f"Warning: Timed out waiting for 'DONE' from Nav Arduino: {cmd}")
        time.sleep(delay)
        current_yaw = query_yaw_from_nav()
        if cmd.startswith(("F", "B")) and len(cmd) > 1:
            dist = int(cmd[1:]) * (1 if cmd[0] == 'F' else -1)
            theta = math.radians(yaw)
            pose[0] += dist * math.cos(theta)
            pose[1] += dist * math.sin(theta)
        elif yaw_before is not None:
            yaw_after = current_yaw
            delta = normalize_angle(yaw_after - yaw_before)
            yaw = normalize_angle(yaw + delta)
            print(f"Updated global yaw: {yaw:.1f} degrees")
    except Exception as e:
        print(f"Warning: Error sending nav command '{cmd}': {e}")
        try_reconnect_serial()

def send_arm(cmd, delay=2):
    """Sends a command to the robotic arm and waits for a 'DONE' confirmation."""
    global arduino_arm
    if not arduino_arm:
        print(f"Warning: Robotic Arm Arduino not connected, cannot send command: {cmd}")
        try_reconnect_arm()
        return
    try:
        arduino_arm.write(f"{cmd}\n".encode())
        print(f"Sent to Arm: {cmd}")
        if not wait_for(arduino_arm, "DONE", timeout=15.0):
            print(f"Warning: Timed out waiting for 'DONE' from Arm Arduino: {cmd}")
        time.sleep(delay)
    except Exception as e:
        print(f"Warning: Error sending arm command '{cmd}': {e}")
        try_reconnect_arm()
#=================================================================================================================================
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
def detect_and_align_target():
    """Detects a target, displays it on screen, and sends alignment commands."""
    global is_centered
    print("Starting target detection and alignment...")
    is_centered = False
    
    with open(LABELS_FILENAME, 'r') as f:
        labels = [l.strip() for l in f.readlines()]
    detector = TFLiteObjectDetection(MODEL_FILENAME, labels)
    
    video = cv2.VideoCapture(0)
    if not video.isOpened():
        print("Error: Cannot open camera.")
        return False
    
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(1)
    
    print("Searching for targets...")
    try:
        while not is_centered and not exit_event.is_set():
            ret, frame = video.read()
            if not ret:
                print("Error: Cannot read video frame.")
                continue
            
            image = helper.update_orientation(frame)
            image = helper.resize_down_to_1600_max_dim(image)
            h, w = image.shape[:2]
            min_dim = min(w, h)
            square_img = helper.crop_center(image, min_dim, min_dim)
            input_img = cv2.resize(square_img, (512, 512))
            
            predictions = detector.predict_image(Image.fromarray(input_img))
            
            best_pred = None
            max_prob = 0.3
            for pred in predictions:
                if pred['probability'] > max_prob:
                    max_prob = pred['probability']
                    best_pred = pred

            # --- REVISED: Added visualization logic ---
            draw_crosshair(input_img, CENTER_X, CENTER_Y) # Draw screen center
            font = cv2.FONT_HERSHEY_SIMPLEX

            if best_pred:
                topleft = (int(best_pred['boundingBox']['left'] * 512), int(best_pred['boundingBox']['top'] * 512))
                bottomright = (int(topleft[0] + best_pred['boundingBox']['width'] * 512), int(topleft[1] + best_pred['boundingBox']['height'] * 512))
                
                target_cx = (topleft[0] + bottomright[0]) // 2
                target_cy = (topleft[1] + bottomright[1]) // 2
                
                # Draw bounding box and target center
                cv2.rectangle(input_img, topleft, bottomright, (0, 0, 255), 2)
                cv2.circle(input_img, (target_cx, target_cy), 5, (0, 0, 255), -1)

                dx_px = target_cx - CENTER_X
                dy_px = CENTER_Y - target_cy
                
                dx_mm = dx_px * PIXELS_TO_MM
                dy_mm = dy_px * PIXELS_TO_MM
                
                # Display offset text on the image
                label = f"dx={dx_mm:.1f}mm, dy={dy_mm:.1f}mm"
                cv2.putText(input_img, label, (topleft[0], topleft[1] - 10), font, 0.6, (255, 255, 255), 2)
                
                print(f"Target offset: x={dx_mm:.1f} mm, y={dy_mm:.1f} mm")
                
                if abs(dx_mm) > 30:
                    direction = 'MR' if dx_mm > 0 else 'ML'
                    angle = int(abs(dx_mm) * MM_TO_DEGREES)
                    if angle > 0: send_nav(f"{direction}{angle}")
                elif dy_mm > 15:
                    send_nav(f"F{int(abs(dy_mm))}")
                elif dy_mm < -15:
                    send_nav(f"B{int(abs(dy_mm))}")
                else:
                    is_centered = True
                    print("Success: Target is centered. Preparing to grab.")
            
            # Display the annotated image in a window
            cv2.imshow("Target Alignment View", input_img)
            
            # This is crucial for the window to update. It also allows you to quit by pressing 'q'.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Manual exit from alignment.")
                exit_event.set()
                break
            # --- END REVISED LOGIC ---
            
            if is_centered:
                time.sleep(0.5) # Pause briefly on the centered image
                break
            
    except Exception as e:
        print(f"Error: Target detection error: {str(e)}")
        return False
    finally:
        video.release()
        cv2.destroyAllWindows() # Close the video window
        
    return is_centered

def PlantPredict(orientation):
    global centers
    global sending_actions
    sending_actions = False
    numGood=0
    numBad=0
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
                #font = cv2.FONT_HERSHEY_SIMPLEX
                
                # Looping through number of predictions
                for pred in predictions:
                    if pred['probability'] >= 0.70: # CHANGED PRObAbILITY TO 75
                        send_nav("STOP")
                        found = True
                        # Draw rectangle for each bounding box based on left, top pixel + width and height
                        topleft = (int(pred['boundingBox']['left'] * augmented_image.shape[0]), int(pred['boundingBox']['top'] * augmented_image.shape[1]))
                        bottomright = (int(topleft[0] + pred['boundingBox']['width'] * augmented_image.shape[0]), int(topleft[1] + pred['boundingBox']['height'] * augmented_image.shape[0]))
                        # print(topleft)
                        # print(bottomright)
                        prediction = str(pred['tagName'])
                        ######################################
                        # to save the object location
                        # compute the center of boxes
                        #x = (topleft[0] + bottomright[0]) // 2
                        #y = (topleft[1] + bottomright[1]) // 2
                        # print(f"center: ({x:d}, {y:d})")
                        
                        #centers.append((x, y))
                        ######################################
                        
                        # text to put
                        #text=f"{pred['tagName']} | {round(pred['probability'] * 100, 2)}%" 
                        
                        # draw rectangle and text on img
                        #cv2.rectangle(augmented_image, topleft, bottomright, (255, 0 ,0), 2)
                        #cv2.putText(augmented_image, text, topleft, font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                
                        # motor controls
                        # motor.motor_control(motor.check_species(pred['tagName']))
                        start = False
                        break
                    
                if found:
                    #cv2.imshow('Frames',augmented_image)
                    #cv2.waitKey(t_show)  
                    t_0 = time.time()
                    #command_arduino(port, baud_rate)
                    print(f"{pred['tagName']}")
                    
                    print(prediction)
                    
                    if prediction == "Good":  #LEFT BIN (G2)    # ADD NAVIGATION TO MAKE DISTINCTION BETWEEN THREE PLANTS
                        send_nav(f"B{middle_plant}")
                        send_arm("HGRABG")
                        if orientation:
                            send_arm("SKYR")
                        else:
                            send_arm("SKYL")
                        numGood=1


                    elif prediction == "Bad":  # LEFT BIN (G1)
                        send_nav(f"B{middle_plant}")
                        send_arm("HGRABB")
                        if orientation:
                            send_arm("SKYR")
                        else:
                            send_arm("SKYL")
                        numBad=1
                    
                    elif prediction == "Empty":
                        if orientation:
                            send_arm("SKRET")
                            send_arm("SEEDR")
                            send_arm("SD")            #TELL THE NAVIGATION TO MOVE AS WELL
                            send_arm("G1")
                            send_arm("SKYR")
                        else:
                            send_arm("SKRET")
                            send_arm("SEEDL")
                            send_arm("SD")
                            send_arm("G1")
                            send_arm("SKYL")

                    elif prediction == "Good Good":#TELL THE NAVIGATION TO MOVE AS WELL
                        send_arm("HGRABG")
                        numGood =2
                        if orientation:
                            send_arm("SKYR")
                        else:
                            send_arm("SKYL")
                    elif prediction == "Bad Bad":#TELL THE NAVIGATION TO MOVE AS WELL ( TO GRAB "RIGHT" PLANT ALWAYS DUE TO MODEL INACCURACY)
                        send_arm("HGRABB")
                        numBad =2
                        if orientation:
                            send_arm("SKYR")
                        else:
                            send_arm("SKYL")
                    elif prediction == "Good Bad":
                        if orientation:
                            send_nav(f"B{backward_plant}")
                            send_arm("HGRABB")
                        else:
                            send_arm("HGRABB")
                        if orientation:
                            send_arm("SKYR")
                        else:
                            send_arm("SKYL")
                        numGood=1
                        numBad=1
                    elif prediction == "Bad Good":
                        if orientation:
                            send_nav(f"B{backward_plant}")
                        send_arm("HGRABB")
                        if orientation:
                            send_arm("SKYR")
                        else:
                            send_arm("SKYL")
                        numGood=1
                        numBad=1
                    #send_tag(prediction)  # ADD CONDITIONS!!!

                    
                # Reset the time counter
                delta = 0
                
            
            # show the frame on the newly created image window
            '''if not found:
                cv2.imshow('Frames',augmented_image)
                cv2.waitKey(1)'''
                
    
        
    print("Video has ended.")
    return numGood, numBad
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
    print("SENDING TAG TO ARDUINO")
    global sending_actions
    arduino = connect_to_arduino(port, baud_rate)
    sending_actions = True
    response = arduino.readline().decode("utf-8").rstrip()
    #print(response)
    if(tag=="Good"):
        arduino.write(f"Good\n".encode())
    elif(tag=="Bad"):
        arduino.write(f"Bad\n".encode())
    elif(tag == "GS"):
        arduino.write(f"GS\n".encode())
    elif(tag == "BS"):
        arduino.write(f"BS\n".encode())
    #arduino.write(f"{tag}\n".encode())  # Send the number followed by a newline
    print(f"Sent: {tag}")
    arduino.flush() 
    # Wait for the Arduino's response
    while True:

        #arduino.write(f"Good\n".encode())
        response = arduino.readline().decode("utf-8").rstrip()
        print(response)
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
        print(response)
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

# ====== Computer Vision Functions ======
def warmup_camera():
    """Initializes and warms up the camera to ensure stable frames."""
    print("Warming up camera...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot open camera.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(5):
        cap.read()
        time.sleep(0.1)
    cap.release()
    print("Camera ready.")

def draw_crosshair(image, x, y, size=20, color=(0, 255, 0), thickness=2):
    """Draws a crosshair on the image for alignment visualization."""
    cv2.line(image, (x - size, y), (x + size, y), color, thickness)
    cv2.line(image, (x, y - size), (x, y + size), color, thickness)

def get_usb_path():
    media_dir = f"/media/pi"

    if not os.path.exists(media_dir):
        return None

    # Any folder inside /media/pi is a mounted USB device
    for item in os.listdir(media_dir):
        path = os.path.join(media_dir, item)
        if os.path.ismount(path):
            return path               # return first USB found

    return None

if __name__ == '__main__':
    video_path = "output.mp4"
    port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_3433332383235121B092-if00"  # Robotic Arm Arduino
    baud_rate = 115200
    #main_thread = threading.Thread(target=main, args=(video_path,))
    #command_arduino_thread = threading.Thread(target=command_arduino, args=(port, baud_rate))
    #main_thread.start()
    #command_arduino_thread.start()
    def signal_handler(sig, frame):
        print("\nProgram interrupted, cleaning up...")
        if arduino_nav: arduino_nav.close()
        if arduino_arm: arduino_arm.close()
        print("Program exited.")
        os._exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    forward_plant = 0
    middle_plant = 20
    backward_plant = 40

    try:
        if not arduino_nav or not arduino_arm:
            print("Error: One or both Arduinos not connected. Program cannot continue.")
            os._exit(1)
        
        yaw = query_yaw_from_nav()
        print(f"Initial IMU angle: {yaw} degrees")
        
                                                                                             ##### ADD USB CODE!
        warmup_camera()
        BACKWARD = False # == RIGHT!!         REMOVE?
        usb_path = get_usb_path()
        filename =f"{usb_path}/Mapping_TEAM4.txt"

        Arows = []
        Brows = []
        send_arm("G1")
        
        send_arm("SKYL")
        for i in range(8):
            print(f"\n======= Starting Forward Repetition {i+1} =======")
            send_nav("START")
            ABase_name = f"Base A{i+1}"
            Healthy, Unhealthy = PlantPredict(BACKWARD)  # FIRST FORWARD/LEFT THEN BACKWARD RIGHT
            Arows.append([ABase_name,Healthy,Unhealthy])
        
        send_arm("SKYR")
        BACKWARD = True
        send_nav("PISW")
        for i in range(8):
            print(f"\n======= Starting Backward Repetition {i+1} =======")
            send_nav("START")
            BBase_name = f"Base B{i+1}"
            Healthy, Unhealthy = PlantPredict(BACKWARD)  # FIRST LEFT THEN RIGHT
            Brows.append([BBase_name,Healthy,Unhealthy])

        send_nav("B50")
        send_arm("BB")
        media_dir = f"/media/pi"
    # Write UTF-8 CSV
        with open(filename, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
        
            # Header
            writer.writerow(["Base Number", "Healthy", "Unhealthy"])
        
            # Data rows
            for base_name, healthy, unhealthy in Arows:
                writer.writerow([base_name, healthy, unhealthy])
            for base_name, healthy, unhealthy in Brows:
                writer.writerow([base_name, healthy, unhealthy])


    except Exception as e:
        print(f"Error: Program error: {str(e)}")
    finally:
        if arduino_nav: arduino_nav.close()
        if arduino_arm: arduino_arm.close()
        print("Program finished.")


    
    
    
