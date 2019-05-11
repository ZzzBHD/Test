#!/usr/bin/env python
# coding: utf-8

import rospy
rospy.init_node('kinect_py_socket', anonymous=True)
import cv2
import numpy as np

from std_msgs.msg import Int32
from std_msgs.msg import Float64
import tensorflow as tf
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


def socket_callback(socket_flag):
    if socket_flag.data==1:
#        rospy.loginfo("I heard %s", socket_flag.data)
        image=cv2.imread('/home/cyberc3/socket/4.png')
#        detecotr.detect(image)
        img_large=cv2.resize(image,(800,500),interpolation=cv2.INTER_AREA)
        cv2.imshow('t',img_large)
        cv2.waitKey(1)
    else:
        cv2.destroyAllWindows()
        rospy.signal_shutdown("close")


class TOD(object):
    def __init__(self):
        self.PATH_TO_CKPT = '/home/cyberc3/Python/sublime_ws/test/frozen_inference_graph.pb'
        self.PATH_TO_LABELS = '/home/cyberc3/Python/sublime_ws/test/mscoco_label_map.pbtxt'
        self.NUM_CLASSES = 1
        self.detection_graph = self._load_model()
        self.category_index = self._load_label_map()
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def _load_model(self):
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
               serialized_graph = fid.read()
               od_graph_def.ParseFromString(serialized_graph)
               tf.import_graph_def(od_graph_def, name='')
        return detection_graph

    def _load_label_map(self):
        label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map,
        max_num_classes=self.NUM_CLASSES,
        use_display_name=True)
        category_index = label_map_util.create_category_index(categories)
        return category_index

    def detect(self, image):
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
#                image_np_expanded = np.expand_dims(image, axis=0)

                # Actual detection.
                (boxes, scores, classes, num_detections) = sess.run(
                    [self.boxes, self.scores, self.classes, self.num_detections],
                    feed_dict={self.image_tensor: image_np_expanded})
                # Visualization of the results of a detection.
                vis_util.visualize_boxes_and_labels_on_image_array(
                    image,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    self.category_index,
                    use_normalized_coordinates=True,
                    line_thickness=8)


        # cv2.namedWindow("detection", cv2.WINDOW_NORMAL)
#        img_large=cv2.resize(image,(800,500),interpolation=cv2.INTER_AREA)
#        cv2.imshow("detection", img_large)
        # cv2.waitKey(0)

def socket():

    rospy.Subscriber("kinect_socket_flag",Float64,socket_callback)
    print("Waiting...")
    rospy.spin()




if __name__ == '__main__':
    try:
#        detecotr = TOD()
        socket()
    except rospy.ROSInterruptException:
        pass
