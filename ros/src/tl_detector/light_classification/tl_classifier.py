from styx_msgs.msg import TrafficLight
#import matplotlib
#matplotlib.use('Agg')

#import matplotlib.pyplot as plt
import numpy as np

import cv2
import tensorflow as tf
import rospy
import traceback

import json

#SSD fails beccause of tensorflow version on CARLA
#SSD_GRAPH_FILE = 'light_classification/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'
SSD_GRAPH_FILE = 'light_classification/faster_rcnn_resnet101_coco_11_06_2017/frozen_inference_graph.pb'


#https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md

def log(s):
	with open('logfile.txt','a') as f:
		f.write(s+'\n')

with open('logfile.txt','wb') as f:
	f.write('new log \n')

log('tf version: %s'%str(tf.__version__) )

def load_graph(graph_file):
	"""Loads a frozen inference graph"""
	graph = tf.Graph()
	with graph.as_default():
		od_graph_def = tf.GraphDef()
		with tf.gfile.GFile(graph_file, 'rb') as fid:
			serialized_graph = fid.read()
			od_graph_def.ParseFromString(serialized_graph)
			tf.import_graph_def(od_graph_def, name='')
	return graph


detection_graph = load_graph(SSD_GRAPH_FILE)
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')


sess = tf.Session(graph=detection_graph)

log('session open')

def detect(image, sess):
	image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
	(boxes, scores, classes) = sess.run([detection_boxes, detection_scores, detection_classes], feed_dict={image_tensor: image_np})
	
	boxes = np.squeeze(boxes)
	scores = np.squeeze(scores)
	classes = np.squeeze(classes)
	
	return boxes, scores, classes

def filter_boxes(min_score, boxes, scores, classes):
	"""Return boxes with a confidence >= `min_score`"""
	n = len(classes)
	idxs = []
	for i in range(n):
		if scores[i] >= min_score and classes[i]==10:
			idxs.append(i)
	
	filtered_boxes = boxes[idxs, ...]
	filtered_scores = scores[idxs, ...]
	filtered_classes = classes[idxs, ...]
	return filtered_boxes, filtered_scores, filtered_classes

def to_image_coords(boxes, height, width):
	"""
	The original box coordinate output is normalized, i.e [0, 1].
	
	This converts it back to the original coordinate based on the image
	size.
	"""
	box_coords = np.zeros_like(boxes)
	box_coords[:, 0] = boxes[:, 0] * height
	box_coords[:, 1] = boxes[:, 1] * width
	box_coords[:, 2] = boxes[:, 2] * height
	box_coords[:, 3] = boxes[:, 3] * width
	
	return box_coords

def draw_boxes(img, bboxes, color=(0, 0, 255), thick=3):
	'''draws bounding boxes'''
	# Make a copy of the image
	imcopy = np.copy(img)
	# Iterate through the bounding boxes
	for bbox in bboxes:
		# Draw a rectangle given bbox coordinates
		cv2.rectangle(imcopy, (bbox[1], bbox[0]),(bbox[3], bbox[2]), color, thick)
	# Return the image copy with boxes drawn
	return imcopy

def sub_images(im,box_coords):
	'''grab a rectangular area from a larger image'''
	images=[]
	for b in box_coords:
		x,y,w,h = int(np.round(b[1])),int(np.round(b[0])),int(np.round(b[3]-b[1])),int(np.round(b[2]-b[0]))
		images.append(im[y:y+h, x:x+w])
	return images

def classify_sub(sub):
	'''classify an image that contains ONLY a traffic light (we assume)
	simple R,G, and yellow-ish thresholding at the assumed positions: Green at the bottom, red at the top
	'''
	h,w, _ = sub.shape
	h3 = int(h/3)
	light_color=[]

	#maybe green? check the lower third of the image's green channel
	i=0
	a= h-i*h3
	b = h-(i+1)*h3
	ret,thresh = cv2.threshold(sub[b:a,:,1],190,255,cv2.THRESH_BINARY)
	#plt.imshow(thresh,cmap='gray')

	light_color.append(thresh.mean())

	#maybe yellow? check the middle
	#plt.figure()

	i=1
	a= h-i*h3
	b = h-(i+1)*h3
	lower = np.uint8([180, 180,   0])
	upper = np.uint8([255, 255, 255])
	thresh = cv2.inRange(sub[b:a,:,:], lower, upper)
	#plt.imshow(thresh,cmap='gray')

	light_color.append(thresh.mean())

	#maybe red? check the top part of the image for red
	#plt.figure()
	i=2
	a= h-i*h3
	b = h-(i+1)*h3
	ret,thresh = cv2.threshold(sub[b:a,:,0],127,255,cv2.THRESH_BINARY)
	#plt.imshow(thresh,cmap='gray')

	light_color.append(thresh.mean())

	#print(light_color)
	light_states={0:'GREEN',1:'YELLOW',2:'RED'}
	
	if np.max(light_color) < 5:
		return 'RED'
	
	return light_states[np.argmax(light_color)]

def classify_all(im,sess):
	'''do everything'''
	boxes, scores, classes = detect(im,sess)
	boxes, scores, classes = filter_boxes(.85, boxes, scores, classes)
	h, w, _ = im.shape
	box_coords = to_image_coords(boxes, h, w)
	subs = sub_images(im, box_coords)
	
	#plt.clf()
	#plt.imshow(draw_boxes(im,box_coords))
	#plt.show()

	return [classify_sub(sub) for sub in subs]


class TLClassifier(object):
	def __init__(self):
		log('classifier created')

	def get_classification(self, image):
		"""Determines the color of the traffic light in the image

		Args:
			image (cv::Mat): image containing the traffic light

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		#cv2.imwrite('test.png',image) #works!
		try:
			lights = classify_all(image,sess)
			log(json.dumps(lights))

			for l in lights:
				if l in  ['RED']: #treat yellow lights are red to be on the safe side
					return TrafficLight.RED
		except Exception as e:
			log(traceback.format_exc())

		return TrafficLight.UNKNOWN
