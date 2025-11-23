#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import cv2
import mediapipe as mp
import math
import sys
from numpy.lib.type_check import imag

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands


class HandNode(Node):
    def __init__(self):
        super().__init__('hand_node')
        self.pub = self.create_publisher(String, 'finger_num', 10)


def main(args=None):
  rclpy.init(args=args)
  node = HandNode()

  cap = cv2.VideoCapture(0)
  with mp_hands.Hands(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    
    while True: #not rospy.is_shutdown():
      finger_num = None

      success, image = cap.read()
      if not success:
        print("Ignoring empty camera frame.")
        continue

      image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
      image.flags.writeable = False
      results = hands.process(image)
      image_height, image_width, _ = image.shape
      image.flags.writeable = True
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
          #各指の第一関節の座標を変数に格納
          for index, landmark in enumerate(hand_landmarks.landmark):
              landmark_x = min(int(landmark.x * image_width), image_width - 1)
              landmark_y = min(int(landmark.y * image_height), image_height - 1)

              #必要な手の関節座標を変数に格納
              if index == 0:
                  cx0,cy0 = landmark_x, landmark_y
              if index == 4:
                  cx4,cy4 = landmark_x, landmark_y
              if index == 8:
                  cx8,cy8 = landmark_x, landmark_y
             # if index == 9:
              #    cx9,cy9 = landmark_x, landmark_y
              if index == 12:
                  cx12,cy12 = landmark_x, landmark_y
              if index == 16:
                  cx16,cy16 = landmark_x, landmark_y
              if index == 20:
                  cx20,cy20 = landmark_x, landmark_y

          #取得した座標より各指の中間点を表示
          #gap1x,gap1y = (cx0+cx9)/2, (cy0+cy9)/2
          #gap1x = int(gap1x)
          #gap1y = int(gap1y)
          #cv2.circle(image, (gap1x,gap1y), 5, (0, 255, 0), 2)
          #gap1x,gap1y = gap1x-image_width/2,-(gap1y-image_height/2)

          
          #取得した座標より各指の手首との距離を計算
          finger_thump  = int(math.sqrt((cx4-cx0)**2))
          finger_index  = int(math.sqrt((cx8-cx0)**2  + (cy8-cy0)**2) )
          finger_middle = int(math.sqrt((cx12-cx0)**2 + (cy12-cy0)**2))
          finger_ring   = int(math.sqrt((cx16-cx0)**2 + (cy16-cy0)**2))
          finger_little = int(math.sqrt((cx20-cx0)**2 + (cy20-cy0)**2))

          threshold = [100,200,200,200,200]
          finger_open = ["close"] * 5
          fingers = [finger_thump, finger_index, finger_middle, finger_ring, finger_little]

          for i,fing,thres in zip(range(5),fingers,threshold):
            if fing > thres:
                 finger_open[i] = "open"
            else:
                 finger_open[i] = "close"
          if finger_open[1] == "open" and all(f == "close" for f in [finger_open[0], finger_open[2], finger_open[3], finger_open[4]]):
                finger_num = 1
          
          
          #カメラ画像に値を表示させたい場合は
          cv2.putText(image,"finger_thump:"+str(finger_thump)+":"+str(finger_open[0]),(10,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"finger_index:"+str(finger_index)+":"+str(finger_open[1]),(10,60),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"finger_middle:"+str(finger_middle)+":"+str(finger_open[2]),(10,90),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"finger_ring:"+str(finger_ring)+":"+str(finger_open[3]),(10,120),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"finger_little:"+str(finger_little)+":"+str(finger_open[4]),(10,150),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"threshold:"+str(threshold),(10,180),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)#閾値
          cv2.putText(image,"number:"+str(finger_num),(10,210),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)#指数字

          mp_drawing.draw_landmarks(
              image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
      cv2.imshow('MediaPipe Hands', image)

      # publish
      msg = String()
      msg.data = str(finger_num)
      node.pub.publish(msg)
      node.get_logger().info(f"Publish: {msg.data}")
      
      if cv2.waitKey(5) & 0xFF == 27:
        break
    
if __name__ == '__main__':
    try:
      main()
    except Exception as e:# rospy.ROSInterruptException: 
      print("Error:", e)
      pass  
