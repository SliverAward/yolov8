import cv2
import RPi.GPIO as GPIO
import time
from yolov8 import YOLOv8
import threading


#BCN PIN
# GPIO 핀 모드 설정
GPIO.setmode(GPIO.BCM)

# 초음파 센서 핀 번호
trig = 23
echo = 24

# LED 핀 번호
led = 12 
GPIO.setup(led, GPIO.OUT)
# 초음파 센서 GPIO 설정
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)


# Initialize the webcam
cap = cv2.VideoCapture(0)

# Initialize YOLOv8 object detector
model_path = "/home/esp/ONNX/best.onnx"
yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)

cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)

distance = 0

def measure_sonic(trig,echo):
    global distance

    # 초음파 센서로 300mm 이내에 객체가 있는지 체크
    GPIO.output(trig, False)
    time.sleep(0.01)

    GPIO.output(trig, True)
    time.sleep(0.0001)
    GPIO.output(trig, False)

    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()
    
    check_time = pulse_end - pulse_start
    distance = check_time * 17000
    distance = round(distance, 2)
    print("Distance : %.1f" %distance)
    time.sleep(0.01)
    return distance

def sonic_thread():
    while cap.isOpened():
        global distance
        distance = measure_sonic(trig,echo)
        time.sleep(0.1)  # 100ms마다 거리 측정을 반복합니다. 이 값을 조절하십시오.
sonic_thread = threading.Thread(target=sonic_thread)
sonic_thread.start()

while cap.isOpened():
    
    # Read frame from the video
    ret, frame = cap.read()

    if not ret:
        break

    boxes, scores, class_ids = yolov8_detector(frame)

    combined_img = yolov8_detector.draw_detections(frame)
    cv2.imshow("Detected Objects", combined_img)
    cv2.resizeWindow("Detected Objects", [416,416])

    #person, vehicle 동시에 탐지되었는지 체크
    if 1 in class_ids and 2 in class_ids:
        print("detection...")
        if distance < 30:
            # LED 켜기
            print("ooooooooooooooooooooon")
            GPIO.output(led, GPIO.HIGH)
            
        elif distance < 20:
            print("20cm ddddddddddddddd")
        elif distance < 10:
            print("10cm ddddddddddddd")
    else:
        # LED 끄기
        GPIO.output(led, GPIO.LOW)
    # Press key q to stop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
sonic_thread.join()
