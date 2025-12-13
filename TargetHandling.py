import serial
import time
import os
import cv2
import math
import numpy as np
import threading
import tflite_runtime.interpreter as tf
from PIL import Image

#from object_detection import ObjectDetection
import helper
import signal

# ====== Constants ======
NAV_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_343313230363514032C1-if00"  # Navigation Arduino
ARM_SERIAL = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_3433332383235121B092-if00"  # Robotic Arm Arduino
BAUD_RATE = 115200
ARM_BAUD = 115200

# ====== Model & Vision Parameters ======
MODEL_FILENAME = 'model.tflite'
LABELS_FILENAME = 'labels.txt'
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
    """Sends a command to the robotic arm and waits for a 'DONE' confirmation."""
    global arduino_nav
    if not arduino_nav:
        print(f"Warning: Robotic Nav Arduino not connected, cannot send command: {cmd}")
        try_reconnect_nav()
        return
    try:
        arduino_nav.write(f"{cmd}\n".encode())
        print(f"Sent to Nav: {cmd}")
        if not wait_for(arduino_nav, "DONE", timeout=15.0):
            print(f"Warning: Timed out waiting for 'DONE' from Nav Arduino: {cmd}")
        time.sleep(delay)
    except Exception as e:
        print(f"Warning: Error sending nav command '{cmd}': {e}")
        try_reconnect_nav()

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
    '''

class TFLiteObjectDetection(ObjectDetection):
    """Wrapper for TFLite object detection model."""
    def __init__(self, model_filename, labels):
        super().__init__(labels)
        self.interpreter = tf.Interpreter(model_path=model_filename)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]['index']
        self.output_index = self.interpreter.get_output_details()[0]['index']

    def predict(self, image):
        inputs = np.array(image, dtype=np.float32)[np.newaxis, :, :, (0, 1, 2)]
        self.interpreter.resize_tensor_input(self.input_index, inputs.shape)
        self.interpreter.allocate_tensors()
        self.interpreter.set_tensor(self.input_index, inputs)
        self.interpreter.invoke()
        return self.interpreter.get_tensor(self.output_index)[0]'''

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

# ====== Main Task Flow ======
def run_target_routine():
    """Main routine to find, align with, and pick up a target."""
    global is_centered, exit_event
    is_centered = False
    exit_event.clear()
    
    try:
        print("Step 1: Move to the target area")
        send_arm("G0")
        
        print("Step 2: Begin visual alignment")
        if not detect_and_align_target():
            print("Visual alignment failed.")
            return False
            
        print("Step 3: Grab the target")
        send_arm("G1")
                
        send_nav('B10')
        
        print("Task complete.")
        return True
        
    except Exception as e:
        print(f"Task execution error: {str(e)}")
        return False

# ====== Main Application Entry Point ======
if __name__ == "__main__":
    def signal_handler(sig, frame):
        print("\nProgram interrupted, cleaning up...")
        if arduino_nav: arduino_nav.close()
        if arduino_arm: arduino_arm.close()
        print("Program exited.")
        os._exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        if not arduino_nav or not arduino_arm:
            print("Error: One or both Arduinos not connected. Program cannot continue.")
            os._exit(1)
        
        yaw = query_yaw_from_nav()
        print(f"Initial IMU angle: {yaw} degrees")
        
        send_nav("M220")    
        #warmup_camera()
        for i in range(4):
            print(f"\n======= Starting Target Repetition {i+1} =======")
            
            
            send_arm("GRABR")
            send_arm("GRABL")
            #time.sleep(8)
            send_arm("G1")
            #time.sleep(10)
            send_nav("M220")
            send_nav("B220")
            send_arm("G0")
            #time.sleep(2)
        send_nav("B220")
        send_arm("BB")
        #warmup_camera()
        for i in range(4):
            print(f"\n======= Starting Target Repetition {i+1} =======")
            
            send_nav("M220")
            send_arm("GRABR")
            send_arm("GRABL")
            #time.sleep(8)
            send_arm("G1")
            #time.sleep(10)
            send_nav("M220")
            send_nav("B440")
            send_arm("G0")
            send_arm("BB")
            #time.sleep(2)

        
    except Exception as e:
        print(f"Error: Program error: {str(e)}")
    finally:
        if arduino_nav: arduino_nav.close()
        if arduino_arm: arduino_arm.close()
        print("Program finished.")

