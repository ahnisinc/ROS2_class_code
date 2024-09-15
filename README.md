# ROS2_class_code
ROS2 실습코드 

## imu_publisher 실행시 주의 사항 
### 아두이노보드가 virtual box 에 연결되지 않는 문제 발생
  - 저의 경우 virtualbox에 아두이노 보드를 연결할 경우 virtual box 자체가 다운되는 문제가 반복해서 발생

  ### 해결 방법 
  - 아두이노 보드 대신 ESP32 보드를 사용하여 IMU6050 연결
  - IMU6050 펌웨어 업로드를 윈도우에 있는 아두이노 IDE 를 사용하여 업로드하고
  - 제가 제공한 imu_publisher.py 파일로 실행함
  - rviz2를 실행시키고 fram_id 는 imu_frame으로 설정

  ### 실행 순서
   #### ros2 run  실행
      $ ros2 run imu_publisher imu_publisher
    
   #### 다른 터미널에서 rviz2 실행
      $ rviz2
      -Global Options 에서 Fixed Frame 을 imu_frame 으로 입력
      -Displays 밑에 탭에서 add  하여   By topic를 선택
      -/imu, /data, imu 항목을 선택후 ok누름
      -그러면 imu 탭이 추가되고 이곳에서 imu 상태를 볼수 잇는 좌표와 화살표가 나타나게됨
      -imu6050의 움직임에 따라 화살표가 움직이게 됨

    
