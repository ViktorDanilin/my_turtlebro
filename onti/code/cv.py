import cv2
import time
import rospy
import serial

ser = serial.Serial('/dev/ttyUSB1', 19200)

cap = cv2.VideoCapture(2)
print('OK')
print(cap.isOpened())
for i in range(20):
    ret, frame = cap.read()
cap.release()
gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
print(gray_image)
cv2.imwrite('test.jpg', gray_image)
with open('test.jpg', 'rb') as file:
    img = file.read()
    pack_size = 1024
    pack_num = int(len(img) / pack_size) + 1
    for i in range(0, pack_num):
        serial_pack = img[i * pack_size:(i + 1) * pack_size]
        send_bytes = ser.write(serial_pack)
        print(send_bytes)
        time.sleep(0.5)
    time.sleep(4)
    ser.write('S')

