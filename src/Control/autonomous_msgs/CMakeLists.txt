cmake_minimum_required(VERSION 3.0.2)
project(autonomous_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

 add_message_files(
   DIRECTORY msg
   FILES
   CtrlCmd.msg
   EgoVehicleStatus.msg
   GPSMessage.msg
   ObjectStatus.msg
   ObjectStatusList.msg
   PolyfitLaneData.msg
   PolyfitLaneDataArray.msg
   PoseMsg.msg
 )

#  add_service_files(
#    DIRECTORY srv
#    FILES
#    MoraiScenarioLoadSrv.srv
#  )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS 
  std_msgs
  geometry_msgs
  message_runtime
)
