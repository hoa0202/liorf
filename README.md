# New Feature
------------------- Update Date: 2022-11-20 -------------------
- This version has removed the feature extraction module, making it easier to adapt to different lidars;
  
- Support 'robosense' lidar and Mulran datasets, make the following changes in "*.yaml":
  - sensor: "robosense" or sensor: "mulran"

- Support 6-axis IMU, make the following changes in "*.yaml":
  - imuType: 0 # 0: 6-axis IMU, 1: 9-axis IMU

- Support low frequency IMU（50HZ、100HZ）, make the following changes in "*.yaml":
  - imuRate: 500

------------------- Update Date: 2022-12-13 -------------------
- Re-derivation the LM optimization, don't need coordinate transformation.

------------------- Update Date: 2022-12-24 -------------------
- Modified gps factor, no longer depending on the 'robot_localization' package, and make it easier to adapt to different gnss device(RTK/GPS).

- The gps factor is modified to make it easier to adapt to gnss devices with different frequencies(10HZ~500HZ).

------------------- Update Date: 2023-02-11 -------------------
- Add scancontext loop closure detection;

- Support M2DGR dataset.

------------------- Update Date: 2024-04-29 -------------------
- Add [liorf ROS2](https://github.com/YJZLuckyBoy/liorf/tree/liorf-ros2) version (Foxy、Galactic).

Blog：[LIO-SAM：配置环境、安装测试、适配自己采集数据集](https://blog.csdn.net/qq_42938987/article/details/108434290)

Video：[基于LIO-SAM框架SLAM算法开发系列视频](https://space.bilibili.com/327643131/channel/collectiondetail?sid=945184&ctype=0)

## Dependency
- [gtsam](https://gtsam.org/get_started/)(Georgia Tech Smoothing and Mapping library)
  ```
    sudo add-apt-repository ppa:borglab/gtsam-release-4.0
    sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```
- Others
  ```
    sudo apt install libgeographic-dev
  ```

## Install
1. Use the following commands to download and compile the package.
  ```
    mkdir -p ~/liorf-ros2/src && cd ~/liorf-ros2/src
    git clone https://github.com/YJZLuckyBoy/liorf.git
    cd liorf && git checkout liorf-ros2
    cd ../../
    colcon build
  ```

## Run the package
1. Run the launch file
  ```
    source install/setup.bash
    ros2 launch liorf run_lio_sam_default.launch.py
  ```

2. Play existing bag files. Example data in ROS2 format can be downloaded here ([lio-sam-dataset with ros2 format](https://drive.google.com/drive/folders/1n2AZC7GPpUMoW0K4nFGOI6vVjQcuxPru?usp=sharing))
  ```
    ros2 bag play casual_walk/
  ```

## For fusion gps factor
- Make sure your gnss topic type is 'sensor_msgs::msg::NavSatFix';

- Modify 'gpsTopic' paramter in '*.yaml' with yourself gnss topic;
  ```
    gpsTopic: "gps/fix"    # GPS topic
  ```
- If you want to use liorf with integrated gps factor in kitti dataset, you can use the modified python script in "config/doc/kitti2bag" to obtain high-frequency gps data(Rate: 100HZ, Topic: '/gps/fix/correct'). About how to use "[kitti2bag.py](https://github.com/YJZLuckyBoy/liorf/blob/main/config/doc/kitti2bag/kitti2bag.py)", please refer to [doc/kitti2bag](https://github.com/TixiaoShan/LIO-SAM/tree/master/config/doc/kitti2bag). 
This will generate a BAG package in ROS1 format, You need to convert it to the BAG package in ROS2 format, Please refer to the conversion method [ros2 to ros1](https://ternaris.gitlab.io/rosbags/index.html).

<!-- - For more details, please check the demo video: [基于LIO-SAM框架SLAM算法开发（六）：建图之快速适配多雷达及GNSS设备](https://www.bilibili.com/video/BV1ZD4y177ut/?spm_id_from=333.999.0.0&vd_source=fb7f82fee1e57e882c6174174ad2fa11) -->

## Mapping
  <!-- 1. lio-sam dataset
  <p align='center'>
      <img src="./demo/lio_sam_livox_data.gif" alt="drawing" width="800" height = "400"/>
  </p>

  2. M2DGR dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/M2DGR/gate_01.png" alt="drawing" width="800" height = "400"/>
  </p>

  3. kitti-05 dataset
  <p align='center'>
      <img src="./demo/kitti.gif" alt="drawing" width="800" height = "400"/>
  </p>

  4. ubran_hongkong dataset
  <p align='center'>
      <img src="./demo/ubran_hongkong.gif" alt="drawing" width="800" height = "400"/>
  </p>

  5. MulRan dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/mulran/mulran_00.png" alt="drawing" width="400"/>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/mulran/mulran_01.png" alt="drawing" width="400"/>
  </p>

  6. Multiple Lidar
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/multiple_lidar/multiple_lidar_00.png" alt="drawing" width="400"/>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/multiple_lidar/multiple_lidar_01.png" alt="drawing" width="400"/>
  </p>

  6. r3live dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/r3live/r3live_data_00.png" alt="drawing" width="266"/>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/r3live/r3live_data_01.png" alt="drawing" width="266"/>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/liorf/r3live/r3live_data_02.png" alt="drawing" width="266"/>
  </p> -->

## Performance
  <!-- 1. MulRan
  <p align='center'>
      <img src="./demo/mulran_traj.png" alt="drawing" width="800" height = "400"/>
  </p>

  2. Kitti 01
  <p align='center'>
      <img src="./demo/kitti01_trajec.png" alt="drawing" width="800" height = "400"/>
  </p>
  <p align='center'>
      <img src="./demo/kitti01_ape.png"    alt="drawing" width="400"/>
      <img src="./demo/kitti01_rpe.png"    alt="drawing" width="400"/>
  </p>

  3. Kitti 05
  <p align='center'>
      <img src="./demo/kitti05_trajec.png" alt="drawing" width="800" height = "400"/>
  </p>
  <p align='center'>
      <img src="./demo/kitti05_ape.png"    alt="drawing" width="400"/>
      <img src="./demo/kitti05_rpe.png"    alt="drawing" width="400"/>
  </p> -->

## Acknowledgments
  Thanks for [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), [FAST_LIO2](https://github.com/hku-mars/FAST_LIO), [UrbanNavDataset](https://github.com/weisongwen/UrbanNavDataset), [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) and [MulRanDataset](https://sites.google.com/view/mulran-pr/?pli=1).

# LIORF - LiDAR SLAM with Memory Optimization

## SQLite 기반 슬라이딩 윈도우 최적화

본 프로젝트에 메모리 최적화를 위한 SQLite 데이터베이스 기반 슬라이딩 윈도우 방식이 추가되었습니다. 이 방식은 장기간 SLAM 운용 시 메모리 사용량을 효율적으로 관리하면서도 맵 정보의 품질을 유지합니다.

### 주요 기능

- **슬라이딩 윈도우 방식**: 현재 위치 주변의 키프레임만 메모리에 유지하고 나머지는 데이터베이스에 저장
- **자동 메모리 관리**: 설정된 메모리 제한에 도달하면 오래된 키프레임을 자동으로 언로드
- **공간 쿼리 최적화**: R-tree 인덱스를 활용한 효율적인 위치 기반 키프레임 검색
- **일관성 유지**: 트랜잭션을 통한 데이터 일관성 보장
- **포인트 클라우드 직렬화**: 효율적인 포인트 클라우드 저장 및 로드

### 사용 방법

데이터베이스 모드는 launch 파일 또는 파라미터를 통해 활성화할 수 있습니다:

```bash
# 런치 파일을 통한 실행
ros2 launch liorf run_lio_sam_livox_mid360.launch.py
```

또는 파라미터를 직접 설정:

```bash
ros2 run liorf liorf_mapOptmization --ros-args -p use_database_mode:=true -p db_path:=/path/to/your_map.db
```

### 주요 파라미터

- `use_database_mode`: 데이터베이스 모드 활성화 여부 (기본값: false)
- `db_path`: 데이터베이스 파일 경로 (기본값: "slam_map.db")
- `active_keyframes_window_size`: 메모리에 유지할 활성 키프레임 수 (기본값: 100)
- `spatial_query_radius`: 공간 쿼리 반경 (미터, 기본값: 30.0)
- `memory_check_interval_ms`: 메모리 사용량 확인 주기 (밀리초, 기본값: 5000)
- `memory_limit_mb`: 메모리 사용 제한 (MB, 기본값: 2000)

### 동작 원리

1. 새 키프레임이 추가될 때마다 데이터베이스에 저장
2. 현재 위치 주변의 키프레임만 메모리에 로드 (슬라이딩 윈도우)
3. 루프 클로저 등 필요 시 데이터베이스에서 관련 키프레임 로드
4. 메모리 사용량이 임계값을 초과하면 오래된 키프레임 해제

### 필요 의존성

- SQLite3: `sudo apt-get install libsqlite3-dev`

### 성능 팁

- 빠른 SSD에 데이터베이스 파일 저장 권장
- 활성 윈도우 크기를 환경과 하드웨어에 맞게 조절
- 더 빠른 로딩을 위해 포인트 클라우드 해상도 조절 가능
