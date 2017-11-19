import YOLO_small_tf
import cv2
yolo = YOLO_small_tf.YOLO_TF()

yolo.disp_console = (True)
yolo.imshow = (True)
#yolo.tofile_img = ('output_test.jpg')
#yolo.tofile_txt = ('output_test.txt')
#yolo.filewrite_img = (True)
#yolo.filewrite_txt = (True)

yolo.detect_from_file('square-dining-tables.jpg')
yolo.detect_from_cvmat('/home/the_z0mbie/g581/YOLO_tensorflow/weights/YOLO_small.ckpt')
cv2.waitKey(0)