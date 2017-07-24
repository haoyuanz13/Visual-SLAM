#!/usr/bin/env python
# How to read a rosbag file
# References: http://wiki.ros.org/rosbag/Code%20API
# ref: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber

# If you have ROS-Bags and you want to extract data from it to work later on, you can do this:
# rostopic echo "topic_name" -b "bag_name" -p > "file.csv"


# Recording a subset of the data
# rosbag record -O subset /turtle1/cmd_vel /turtle1/pose

# Examining and playing the bag file
# rosbag info <your bagfile>
import rospy
import cv2
import rosbag
import  time
import os
from cv_bridge import CvBridge, CvBridgeError
import cPickle as pkl
def main():
    path = 'images'
    if not os.path.exists(path):
        os.makedirs(path)
    rospy.init_node('publish_rosbag')
    rospy.loginfo('Okie import')
    bag_file = '/home/tynguyen/sample.bag'
    bag = rosbag.Bag(bag_file)
    rospy.loginfo('Okie read bag')
    t0 = 0
    bridge = CvBridge()
    r = rospy.Rate(20)  # 10hz
    i = 0
    rgb_data = []
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    out_video_ = cv2.VideoWriter('perch_video' +'.mp4',fourcc,10,(640,480))
    
    # , '/scarab/camera/rgb/image_raw/camera_info'
    for topic, msg, t in bag.read_messages(topics=['/scarab/camera/rgb/image_raw/compressed']):
        data_t = {}

        print('-i =', i)
        # print '--t {0}, topic {1}, msg {2}'.format(t,topic, msg)
        # print '--time:',(msg.header.stamp - t0)*1.0/10.0**6 # deltatime in millisecond
        # print 'Time:',msg.header.stamp.toSec()
        print('t:',msg.header.stamp.to_time())
        data_t['t'] = [msg.header.stamp.to_time()]

        cv_img = bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        data_t['rgb'] = cv_img
        cv2.imshow('video',cv_img)
        file_name = os.path.join(path,str(i) + '.jpg')
        cv2.imwrite(file_name,cv_img)
        out_video_.write(cv_img)
        i += 1
        # if i == 6:
        #     break
        rgb_data.append(data_t)
        print('size of image:',cv_img.shape)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        r.sleep()

    if not os.path.exists('data'):
        os.makedirs('data')
    with open('data/RGB_1.pkl','wb') as f:
        pkl.dump(rgb_data,f,pkl.HIGHEST_PROTOCOL)
        print('--Complete writing file')

    bag.close()
    cv2.destroyAllWindows()
    out_video_.release()

if __name__ == "__main__":
    rospy.loginfo('start')
    main()