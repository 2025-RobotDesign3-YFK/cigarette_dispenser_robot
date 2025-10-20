#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import cv2
import mediapipe as mp
import rospy
import math
import sys
from geometry_msgs.msg import Point
from numpy.lib.type_check import imag
from robotdesign3_2021_1.msg import CustomArray

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def main():
  rospy.init_node("MediaPipe_node")
  hand_position = rospy.Publisher('hand_topic', CustomArray, queue_size=10)
  r = rospy.Rate(10) # 10hz
  cap = cv2.VideoCapture(4)
  array_points = CustomArray()
  array_points.points= [0]
  with mp_hands.Hands(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    
    while not rospy.is_shutdown():
      
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


          #取得した座標より各指の距離を表示
          gap1x,gap1y = cx0+cx4, cy0+cy4
          gap2x,gap2y = cx0+cx8, cy0+cy8
          gap3x,gap3y = cx0+cx12, cy0+cy12
          gap4x,gap4y = cx0+cx16, cy0+cy16
          gap5x,gap5y = cx0+cx20, cy0+cy20

          gap1x = int(gap1x)
          gap1y = int(gap1y)

          gap2x = int(gap2x)
          gap2y = int(gap2y)

          gap3x = int(gap3x)
          gap3y = int(gap3y)

          gap4x = int(gap4x)
          gap4y = int(gap4y)

          gap5x = int(gap5x)
          gap5y = int(gap5y)

          cv2.circle(image, (gap1x,gap1y), 5, (0, 255, 0), 2)
          cv2.circle(image, (gap2x,gap2y), 5, (0, 255, 0), 2)
          cv2.circle(image, (gap3x,gap3y), 5, (0, 255, 0), 2)
          cv2.circle(image, (gap4x,gap4y), 5, (0, 255, 0), 2)
          cv2.circle(image, (gap5x,gap5y), 5, (0, 255, 0), 2)
          
          gap1x,gap1y = gap1x-image_width/2,-(gap1y-image_height/2)
          gap2x,gap2y = gap2x-image_width/2,-(gap2y-image_height/2)
          gap3x,gap3y = gap3x-image_width/2,-(gap3y-image_height/2)
          gap4x,gap4y = gap4x-image_width/2,-(gap4y-image_height/2)
          gap5x,gap5y = gap5x-image_width/2,-(gap5y-image_height/2)


          array_points.points = []

          point_1 = Point()
          point_2 = Point()
          point_3 = Point()
          point_4 = Point()
          point_5 = Point()

          point_1.x = gap1x/3000
          point_1.y = gap1y/3000

          point_2.x = gap2x/3000
          point_2.y = gap2y/3000

          point_3.x = gap3x/3000
          point_3.y = gap3y/3000

          point_4.x = gap4x/3000
          point_4.y = gap4y/3000

          point_5.x = gap5x/3000
          point_5.y = gap5y/3000

          array_points.points.append(point_1)
          array_points.points.append(point_2)
          array_points.points.append(point_3)
          array_points.points.append(point_4)
          array_points.points.append(point_5)

          hand_position.publish(array_points)
          print(array_points.points[0].x)
          
          #カメラ画像に値を表示させたい場合は
          cv2.putText(image,"gap1x:"+str(gap1x)+"gap1y:"+str(gap1y),(10,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"gap2x:"+str(gap2x)+"gap1y:"+str(gap2y),(10,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"gap3x:"+str(gap3x)+"gap1y:"+str(gap3y),(10,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"gap4x:"+str(gap4x)+"gap1y:"+str(gap4y),(10,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
          cv2.putText(image,"gap5x:"+str(gap5x)+"gap1y:"+str(gap5y),(10,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)


          mp_drawing.draw_landmarks(
              image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
      cv2.imshow('MediaPipe Hands', image)
      
      r.sleep()
      if cv2.waitKey(5) & 0xFF == 27:
        break
    
if __name__ == '__main__':
    try:
      main()
    except rospy.ROSInterruptException: 
      pass  
