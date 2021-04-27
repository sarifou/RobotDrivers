#!/usr/bin/env/ python

import cv2

video_cap = cv2.VideoCapture(0)

while(True):

	ret, frame = video_cap.read()

	cv2.imshow("Frame", frame)

	if cv2.waitKey(10) & 0xFF == ord('q') :
		break

video_cap.release()
cv2.destroyAllWindows()
