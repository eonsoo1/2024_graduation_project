# 2024_graduation_project
S/W Platform for Team 1 of graduation project

# Clone
### 1. Clone repository
```
 git clone https://github.com/eonsoo1/2024_graduation_project.git
```
### 2. Build
```
catkin_make
source devel/setup.bash
```
# Launch
### 1. usb port 인식 및 권한 부여 
```
1) usb port 인식 확인(코드에서 GPS = ACM0, IMU = ACM1로 사용 중)
ls /dev/tty*
2) port 권한 부여
sudo /dev/tty*
```
### 2. IMU launch 파일 실행(설프 디렉토리 내)
```
roslaunch myahrs_driver myahrs_driver.launch
```

### 3. GPS launch 파일 실행(설프 디렉토리 내)
```
roslaunch ntrip_ros ntrip_ros.launch
```
결과창

### 4. EKF node 실행(설프 > shellscript 디렉토리 내)
```
sh launch.sh
```
