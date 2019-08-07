#!/usr/bin/env python
#-*- coding: utf-8 -*-
'''kinect_ssd ROS Node'''
import time
import collections
# import rospy
import cv2
import numpy as np
import tensorflow as tf
# from std_msgs.msg import String
# from std_msgs.msg import Int32
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

fps=10
width=800
height=500
frame_number=639
test=cv2.VideoWriter('test2.avi',cv2.VideoWriter_fourcc('M','J','P','G'),fps,(width,height))
# def Init():
#     rospy.init_node('kinect_ssd', anonymous=True)
#     global py_pub_x 
#     global py_pub_box
#     global py_pub_center_x
#     global py_pub_center_y
#     py_pub_x= rospy.Publisher('kinect_bounding_x', Int32, queue_size=10)
#     py_pub_box = rospy.Publisher('kinect_bounding_box', Int32, queue_size=10)
#     py_pub_center_x=rospy.Publisher('kinect_bounding_center_x', Int32, queue_size=10)
#     py_pub_center_y=rospy.Publisher('kinect_bounding_center_y', Int32, queue_size=10)

class TOD(object):
    def __init__(self):
        self.PATH_TO_CKPT = 'frozen_inference_graph.pb'
        self.PATH_TO_LABELS = 'mscoco_label_map.pbtxt'
        self.NUM_CLASSES = 1
        self.detection_graph = self._load_model()
        self.category_index = self._load_label_map()
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        box_to_color_map = collections.defaultdict(str)
        self.box_to_keypoints_map = collections.defaultdict(list)

    def _load_model(self):
        detection_graph = tf.Graph()
        print ("load_model\n")
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

    def detect(self):
        # image = cv2.imread('/media/cyber-z/D/darknet/data/kite.jpg')
        max_boxes_to_draw=20
        min_score_thresh=.5
        mid_x=100#初始画面中值
        mid_cache=200
        # keypoints=None
        with self.detection_graph.as_default():
            config = tf.ConfigProto(allow_soft_placement=True)
            config.gpu_options.per_process_gpu_memory_fraction = 0.2
            config.gpu_options.allow_growth = False
            # init_op = tf.global_variables_initializer()
            with tf.Session(config=config) as sess:
            # with tf.Session(config=tf.ConfigProto(gpu_options=gpu_options)) as sess:
                k=1
                # while cv2.waitKey(30)!=27:
                while k<=frame_number:
                    #image = cv2.imread('/media/cyber-z/D/darknet/data/kite.jpg')
                    #image_in = cv2.imread('/media/cyber-z/D/Test_img/3.png')
                    image_in=cv2.imread(filename='/home/cyberc3/img_k/'+str(k)+'.jpg')
                    # image_in = cv2.imread('/home/cyberc3/Kinect-test/Test_img/test1.png')
                    min_dif=100
                    target_box=[]
                    person_boxsize=[]
                    person_box=[]
                    person_num=0
                    if image_in is None:
                        print("Read error")
                    else:
                    # image=cv2.resize(image_in,(800,500),interpolation=cv2.INTER_AREA)
                # Expand nsions since the model expects images to have shape: [1, None, None, 3]
                        image=cv2.resize(image_in,(424,512),interpolation=cv2.INTER_AREA)
                        img_height = image.shape[0]
                        img_width = image.shape[1]
                        # print img_height,img_width
                        image_np_expanded = np.expand_dims(image, axis=0)
                        star=time.time()
                    # Actual detection.
                        (boxes, scores, classes, num_detections) = sess.run(
                        [self.boxes, self.scores, self.classes, self.num_detections],
                        feed_dict={self.image_tensor: image_np_expanded})
                        # en=time.time()
                        # print("cost %s s" %(round((en-star),3)))
                    # Visualization of the results of a detection.
                        if not max_boxes_to_draw:
                            max_boxes_to_draw = np.squeeze(boxes).shape[0]
                        for i in range(min(max_boxes_to_draw, np.squeeze(boxes).shape[0])):
                            if np.squeeze(scores) is None or np.squeeze(scores)[i] > min_score_thresh:
                                # print np.squeeze(boxes)[i][1],np.squeeze(boxes)[i][0],np.squeeze(boxes)[i][3],np.squeeze(boxes)[i][2]
                                # print("get person!")
                                if np.squeeze(classes)[i] == 1:
                                    xmin = int(np.squeeze(boxes)[i][1] * img_width)
                                    ymin = int(np.squeeze(boxes)[i][0] * img_height)
                                    xmax = int(np.squeeze(boxes)[i][3] * img_width)
                                    ymax = int(np.squeeze(boxes)[i][2] * img_height)
                                    xmid = xmin+(xmax-xmin)/2
                                    ymid = ymin+(ymax-ymin)/2
                                    dif_x = abs(xmid - mid_x)
                                    person_num+=1
                                    if dif_x <= 15:  # 先判断不跳变
                                        if dif_x <= min_dif:
                                            target_box = [xmin, ymin, xmax, ymax]
                                            min_dif = dif_x
                                            mid_cache=xmid
                                    person_box=[xmin,ymin,xmax,ymax]
                                    person_boxsize.append((xmax - xmin) * (ymax - ymin))
                                    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (255, 178, 50), 2)
                                    cv2.putText(image,str((xmax - xmin) * (ymax - ymin)),(xmin,ymin),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255, 178, 50),2)
                                    # print xmin, ymin, xmax, ymax
                                    box = tuple(np.squeeze(boxes)[i].tolist())
                                # if keypoints is not None:
                                #     self.box_to_keypoints_map[box].extend(keypoints[i])

                        # if len(person_boxsize) is not 0:
                        #     print max(person_boxsize)
                        #     py_pub_x.publish(mid_cache)
                        #     py_pub_box.publish(max(person_boxsize))
                        #     py_pub_center_x.publish(xmid)
                        #     py_pub_center_y.publish(ymid)
                        #     mid_x=mid_cache

                        # if person_num==1:
                        #     cv2.rectangle(image, (person_box[0], person_box[1]), (person_box[2], person_box[3]),
                        #               (0, 0, 255), 4)
                        #     cv2.putText(image,str(max(person_boxsize)),(person_box[0], person_box[1]),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0, 0, 255),2)
                        # elif len(target_box) is not 0:
                        #     cv2.rectangle(image, (target_box[0], target_box[1]), (target_box[2], target_box[3]),
                        #               (0, 0, 255), 4)
                        #     cv2.putText(image,str(max(person_boxsize)),(person_box[0], person_box[1]),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0, 0, 255),2)
                        # else:
                        #     mid_x=200
                        
                        # print mid_cache
                        en = time.time()
                        # print("cost %s s" % (round((en - star), 3)))
                        # vis_util.visualize_boxes_and_labels_on_image_array(
                        #     image,
                        #     np.squeeze(boxes),
                        #     np.squeeze(classes).astype(np.int32),
                        #     np.squeeze(scores),
                        #     self.category_index,
                        #     use_normalized_coordinates=True,
                        #     line_thickness=8)
                        img_large=cv2.resize(image,(800,500),interpolation=cv2.INTER_AREA)
                        # cv2.imshow("detection", img_large)

                        test.write(img_large)
                    print 'The',k,'frame is done!','\n'
                    k=k+1
                        # cv2.imshow("detection", img_large)
                test.release()

# def ssd_callback(data):
#     '''kinect_ssd Callback Function'''
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

# def listener():
#     rospy.spin()

# def talker():
#     '''kinect_ssd Publisher'''
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('kinect_ssd', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

if __name__ == '__main__':
    # Init()
    detecotr = TOD()
    detecotr.detect()