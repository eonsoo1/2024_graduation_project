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
결과창
![Screenshot from 2024-06-03 13-56-30](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/ba1360ed-79fd-4496-96ef-293a3bfd345c)


### 3. GPS launch 파일 실행(설프 디렉토리 내)
```
roslaunch ntrip_ros ntrip_ros.launch
```
결과창
1) 와이파이와 연결이 잘 안됐을 때
![Screenshot from 2024-06-03 13-57-58](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/1ccea63e-5fee-4464-a6d0-7410fc63f778)
2) 정상적으로 실행 됐을 때
![Screenshot from 2024-06-03 13-57-25](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/d0450c3e-4d23-4adb-a6c2-d977cd7df767)

### 4. EKF node 실행(설프 > shellscript 디렉토리 내)
```
sh launch.sh
```
결과창
![Screenshot from 2024-06-03 14-02-03](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/90144be2-85cb-4893-b012-390ad646981f)

