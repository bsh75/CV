# capture.py
import cv2
device_index = 0
cap = cv2.VideoCapture(device_index+cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)


framecount=600
frame_buf=[]
while True: #record indefinitely (until user presses q), replace with "while True"
    stream_ret, frame = cap.read()
    if stream_ret:
        cv2.imshow("image", frame)
        if cv2.waitKey(1) == ord('q'):
            break
        frame_buf.append(frame)
        
cv2.destroyAllWindows()

"""#Below is example code to save images to "folder" (Use appropriate directory syntax.)
num=0
while len(frame_buf)>0:
    cv2.imwrite(f'{folder}/cap_{num}.tiff', frame_buf.pop(0))
    num += 1
"""