import numpy as np
import cv2 as cv
cap = cv.VideoCapture(0)
if not cap.isOpened():
 print("Cannot open camera")
 exit()
cap.set(cv.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,720)
frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
frame_size = (frame_width,frame_height)
output = cv.VideoWriter('output_video_from_file.mp4', cv.VideoWriter_fourcc(*'h264'), 20, frame_size)
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if ret:
        output.write(frame)
        cv.imshow('frame', frame)
        # Our operations on the frame come here
    #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # Display the resulting frame
    
    if cv.waitKey(1) == ord('q'):
        
        break
# When everything done, release the capture
output.release()
cap.release()
cv.destroyAllWindows()