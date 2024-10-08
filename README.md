# ROS2_class_code
ROS2 실습코드 

### 코드를 다운받고 실행하는 방법
- 기본 workspace 폴더는 ros2_ws이다
- 아래에 src폴더에 소스코드가 들어가 있다.
- workspace 에서 colcon build --package-select <package-name>을 입력하면 해당 package만 빌드하게 된다

      colcon build --package-select imu_publisher

- workspace 에서 colcon build 을 입력하면 전체 패키지 빌드하게 된다

      colcon build

- /install, /build, /log 폴더 등이 생성이 되며
  
      source install/setup.bash

 - 실행을 하면 패키지의 node를 실행 할수 있다.

### imu_publisher 실행시 주의 사항 
--------------------------------------------------------------------------------------------------------------
#### 아두이노보드가 virtual box 에 연결되지 않는 문제 발생  --> 해결됨
  - 저의 경우 virtualbox에 아두이노 보드를 연결할 경우 virtual box 자체가 다운되는 문제가 반복해서 발생
  - 위의 문제 해결됨 : 원인 파악은 안되었으나 virtual box 장치의 usb 에 체크하고
  - 아두이노보드를 usb3.0 포트에 꽂으니 시리얼 통신 연결됨
  - usb 2.0 포트에 꽂을 경우 안되는 문제 발생함.
  - USB2.0 포트와 USB3.0 포트에 따라서 인식이 안되는 문제 발생하였고 교육용을로 배포한 노트북에서는 문제 없을것 같음.

  #### 해결 방법 --> 위의 문제 해결됨, 아두이노 보드 사용가능, ESP34 보드도 사용가능
  - 아두이노 보드 대신 ESP32 보드를 사용하여 IMU6050 연결
  - IMU6050 펌웨어 업로드를 윈도우에 있는 아두이노 IDE 를 사용하여 업로드하고
  - 제가 제공한 imu_publisher.py 파일로 실행함
  - rviz2를 실행시키고 fram_id 는 imu_frame으로 설정

  #### 실행 순서
   ##### ros2 run  실행
    ros2 run imu_publisher imu_publisher
    
   ##### 다른 터미널에서 rviz2 실행
    rviz2
  
   - Global Options 에서 Fixed Frame 을 imu_frame 으로 입력
   - Displays 밑에 탭에서 add  하여   By topic를 선택
   - /imu, /data, imu 항목을 선택후 ok누름
   - 그러면 imu 탭이 추가되고 이곳에서 imu 상태를 볼수 있는 좌표와 화살표가 나타나게됨
   - imu6050의 움직임에 따라 화살표가 움직이게 됨

## OPENCV 설치 문제
#### 교재에서 sudo apt install ros-humble-opecv 패키지가 설치가 안되는 문제 발생 (패키지명 변경됨)
- sudo apt install ros-humble-vision-opencv로 변경됨

      sudo apt install ros-humble-vision-opencv
  
- apt-cache로 opencv 관련 패키지 검색후 사용

      apt-cache search opencv
   
