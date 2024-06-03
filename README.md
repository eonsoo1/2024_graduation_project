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
## 1. usb port 인식 및 권한 부여 
```
1) usb port 인식 확인(코드에서 GPS = ACM0, IMU = ACM1로 사용 중)
ls /dev/tty*
2) port 권한 부여
sudo /dev/tty*
```
## 2. IMU launch 파일 실행(설프 디렉토리 내)
```
roslaunch myahrs_driver myahrs_driver.launch
```
### 결과창
1) port명이 ACM1이 아닐 때
![Screenshot from 2024-06-03 14-03-18](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/e2be8f86-9930-4209-ab3a-7cfbb3bdcf6e)
해결방법 : Localization/myahrs_driver/launch/myahrs_driver.launch device명 port에 맞게 수정 or IMU port를 먼저 연결

3) 정상적으로 실행 되었을 때
![Screenshot from 2024-06-03 13-56-30](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/ba1360ed-79fd-4496-96ef-293a3bfd345c)


## 3. GPS launch 파일 실행(설프 디렉토리 내)
```
roslaunch ntrip_ros ntrip_ros.launch
```
### 결과창
1) 와이파이와 연결이 잘 안됐을 때
![Screenshot from 2024-06-03 13-57-58](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/1ccea63e-5fee-4464-a6d0-7410fc63f778)
해결방법 : 와이파이 재연결 후(휴대폰 핫스팟 가능), launch파일 재실행(여러 번 반복해야 됨) 
2) port명이 ACM0이 아닐 때
![Screenshot from 2024-06-03 14-04-32](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/b29190e7-3879-49f0-a4f0-3766f63023c5)
해결방법 : Localization/ublox_f9p/ublox_gps/config/zed-f9p.yaml 에서 device명 port에 맞게 수정 or GPS port를 먼저 연결
    
4) 정상적으로 실행 됐을 때
![Screenshot from 2024-06-03 13-57-25](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/d0450c3e-4d23-4adb-a6c2-d977cd7df767)

## 4. EKF node 실행(설프 > shellscript 디렉토리 내)
```
sh launch.sh
```
### 결과창
![Screenshot from 2024-06-03 14-02-03](https://github.com/eonsoo1/2024_graduation_project/assets/138430886/90144be2-85cb-4893-b012-390ad646981f)

