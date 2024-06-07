#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from nav_msgs.msg import Path
import tf
from math import sqrt
from geometry_msgs.msg import PoseStamped
# from path_reader import pathReader
from turtlesim.msg import Pose
from pyproj import Proj
from std_msgs.msg import Float32MultiArray

from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from autonomous_msgs import PoseMsg

# global_path 와 turtle의 status_msg를 이용해
# 현재 waypoint와 local_path를 생성
prev_waypoint = 0
index = 0


def find_local_path(ref_path,status_msg):
    out_path=Path()
    current_x=status_msg.x #UTM.x
    current_y=status_msg.y
    current_waypoint=0
    global prev_waypoint
    global index
    
    min_dis=float('inf')

    # 가장 가까운 waypoint(current waypoint) 찾기
    for i in range(len(ref_path.poses)):
        dx = current_x - ref_path.poses[i].pose.position.x
        dy = current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
    
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i #현재 위치에서 가장 가까운 waypoint index




  

    
    prev_waypoint = current_waypoint
    index=current_waypoint
    # 현재 waypoint 부터 최대 10개의 waypoint를 local_path에 추가
    if current_waypoint+20 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+20

    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint):
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=0
        tmp_pose.header.seq = index# index저장함
        out_path.poses.append(tmp_pose)

    
    # print(out_path.poses[i].header.seq)
    print("index",index)
    # print("prev",prev_waypoint)
    return out_path,current_waypoint

class pathReader:
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        self.path_folder_name = "path"
        full_file_name=self.file_path+"/"+self.path_folder_name+"/"+file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()
        out_path.header.frame_id='/map'
        # 파일 한줄 --> waypoint 한개
        line=openFile.readlines()
        index = 0
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=0
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=0
            read_pose.header.seq = index
            out_path.poses.append(read_pose)
            index+=1

        openFile.close()
        # print(out_path)
        return out_path


# class turtle_listener():
#     def __init__(self):
#         rospy.Subscriber('/utm_point',PoseStamped, self.statusCB)
#         self.status_msg=Pose()

#     def statusCB(self,data): ## turtle Status Subscriber
#         self.status_msg=data
#         br = tf.TransformBroadcaster()
#         br.sendTransform((self.status_msg.x, self.status_msg.y, 0),
#                         tf.transformations.quaternion_from_euler(0,0, self.status_msg.theta),
#                         rospy.Time.now(),
#                         "Ego",
#                         "map")
        
class UTMConverter:
    def __init__(self, zone=52):

        self.gps_sub = rospy.Subscriber("/gps", PoseMsg, self.navsat_callback)
        self.x, self.y = 0,0
        #데이터 가져오는 부분
        self.proj_UTM = Proj(proj='utm',zone=52,ellps='WGS84',preserve_units=False)
        self.is_gps = False

    
    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude

        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True

        self.convertLL2UTM()

        br = tf.TransformBroadcaster()
        br.sendTransform((-self.x, -self.y, 0.),
                        tf.transformations.quaternion_from_euler(0,0,0.),
                        rospy.Time.now(),
                        "map",
                        "base_link" )
        utm_msg = Float32MultiArray()

        utm_msg.data = [self.x, self.y]
        # print(utm_msg.data)
        # print('lat : ',self.lat)
        # print('lon : ',self.lon)


    def convertLL2UTM(self):

        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o
        # print('[',self.x,',',self.y,']')

if __name__ == '__main__' :
    try:
        rospy.init_node('local_path_finder', anonymous=True)
        path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        # tl=turtle_listener()
        gp = UTMConverter()
        # 전역 경로 로드
        p_r=pathReader("planning")
        global_path = p_r.read_txt("new_way.txt")

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            # 지역 경로 생성
            if gp.is_gps == True:
                local_path,current_waypoint = find_local_path(global_path, gp)
                local_path_pub.publish(local_path)
                path_pub.publish(global_path)
            if not gp.is_gps:
                pass
            # local_path,current_waypoint = find_local_path(global_path, gp)
            # local_path_pub.publish(local_path)
            # path_pub.publish(global_path)
            gp.is_gps = False
            rate.sleep()

    except rospy.ROSInterruptException:
        pass



  # if 899<prev_waypoint<1118:
    #     current_waypoint = prev_waypoint + 1

    # else:
    #     for i in range(len(ref_path.poses)):
    #         dx = current_x - ref_path.poses[i].pose.position.x
    #         dy = current_y - ref_path.poses[i].pose.position.y
    #         dis=sqrt(dx*dx + dy*dy)
    #         if dis < min_dis :
    #             min_dis=dis
    #             current_waypoint=i #현재 위치에서 가장 가까운 waypoint index
        

            
            
        #     if 899<i<1118:
        #         if prev_waypoint+10<i:
        #             current_waypoint = prev_waypoint + 1
        #             i = current_waypoint
        #         elif prev_waypoint-10>i:
        #             current_waypoint = prev_waypoint
        #             i= current_waypoint

        #     else:
        #         current_waypoint=i #가장 가까운 웨이포인트가 나옴
        #     prev_waypoint = current_waypoint
            # print('i : ', i, 'crr : ',current_waypoint)
        # if dis < min_dis :
        #     min_dis=dis
        #     current_waypoint=i
        #     print(i)
