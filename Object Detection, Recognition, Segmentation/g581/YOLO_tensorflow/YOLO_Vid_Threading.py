# import the necessary packages
from threading import Thread
import cv2
import time
import datetime
#import tensorflow as tf
import os, sys
import numpy as np
import YOLO_small_tf

yolo = YOLO_small_tf.YOLO_TF()

class WebcamVideoStream:
	def __init__(self, src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()

		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return

			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()

	def read(self):
		# return the frame most recently read
		return self.frame

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


class FPS:
	def __init__(self):
		# store the start time, end time, and total number of frames
		# that were examined between the start and end intervals
		self._start = None
		self._end = None
		self._numFrames = 0

	def start(self):
		# start the timer
		self._start = datetime.datetime.now()
		return self

	def stop(self):
		# stop the timer
		self._end = datetime.datetime.now()

	def update(self):
		# increment the total number of frames examined during the
		# start and end intervals
		self._numFrames += 1

	def elapsed(self):
		# return the total number of seconds between the start and
		# end interval
		return (self._end - self._start).total_seconds()

	def fps(self):
		# compute the (approximate) frames per second
		return self._numFrames / self.elapsed()


vs = WebcamVideoStream(src=0).start()
time.sleep(2)
i=0
fps = FPS().start()
 


# loop over some frames...this time using the threaded stream
while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	#cv2.imshow("Frame", frame)
	key=cv2.waitKey(1)
	if i%5==0:
		
		
		cv2.imwrite('/home/the_z0mbie/g581/test_img_0.jpg',frame)
		#image_path="/home/the_z0mbie/g581/test_img_0.jpg"
		yolo.disp_console = (True)
		yolo.imshow = (True)
		yolo.tofile_img = ('output_test.jpg')
		yolo.tofile_txt = ('output_test.txt')
		yolo.filewrite_img = (True)
		yolo.filewrite_txt = (True)

		yolo.detect_from_file('/home/the_z0mbie/g581/test_img_0.jpg')
		yolo.detect_from_cvmat('/home/the_z0mbie/g581/YOLO_tensorflow/weights/YOLO_small.ckpt')
	
	i+=1
	if key & 0xFF==ord('q'):
		break
	fps.update()
 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
#sess.close()