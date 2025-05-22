#include "utility.h"
#include "mapDatabase.h"
#include "liorf/msg/cloud_info.hpp"
#include "liorf/srv/save_map.hpp"
// <!-- liorf_yjz_lucky_boy -->
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "Scancontext.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <numeric> // for std::accumulate
#include <set>     // for std::set
#include "costmap.h" // 코스트맵 생성기 클래스 헤더 추가
#include "loopclosure.h" // Loop Closure 클래스 헤더 추가

#include "mapOptimization.h"

// 헤더가 이미 필요한 모든 선언을 포함하므로 중복 선언 제거
// using namespace gtsam;
// using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
// using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
// using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
// using symbol_shorthand::G; // GPS pose

mapOptimization::mapOptimization(const rclcpp::NodeOptions & options) : ParamServer("liorf_mapOptimization", options)
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        subCloud = create_subscription<liorf::msg::CloudInfo>("liorf/deskew/cloud_info", QosPolicy(history_policy, reliability_policy),
                    std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        subGPS = create_subscription<sensor_msgs::msg::NavSatFix>(gpsTopic, QosPolicy(history_policy, reliability_policy),
                    std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));

        pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/trajectory", QosPolicy(history_policy, reliability_policy));
        pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/map_global", QosPolicy(history_policy, reliability_policy));
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/odometry", QosPolicy(history_policy, reliability_policy));
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/odometry_incremental", QosPolicy(history_policy, reliability_policy));
        pubPath = create_publisher<nav_msgs::msg::Path>("liorf/mapping/path", QosPolicy(history_policy, reliability_policy));
        pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/map_local", QosPolicy(history_policy, reliability_policy));
        pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/cloud_registered", QosPolicy(history_policy, reliability_policy));
        pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("liorf/mapping/cloud_registered_raw", QosPolicy(history_policy, reliability_policy));
        pubSLAMInfo = create_publisher<liorf::msg::CloudInfo>("liorf/mapping/slam_info", QosPolicy(history_policy, reliability_policy));
        pubGpsOdom = create_publisher<nav_msgs::msg::Odometry>("liorf/mapping/gps_odom", QosPolicy(history_policy, reliability_policy));

        srvSaveMap = create_service<liorf::srv::SaveMap>("liorf/save_map", 
                        std::bind(&mapOptimization::saveMapService, this, std::placeholders::_1, std::placeholders::_2 ));

        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);
        downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        allocateMemory();
        
        // 코스트맵 발행자 생성
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/liorf/costmap", 10);
        
        // 코스트맵 생성기 초기화
        costmap_generator_ = std::make_unique<CostmapGenerator>(this);
        
        // 파라미터 설정
        double costmap_resolution = this->declare_parameter<double>("costmap_resolution", 0.1);
        double costmap_width = this->declare_parameter<double>("costmap_width", 20.0);     // 너비 축소
        double costmap_height = this->declare_parameter<double>("costmap_height", 20.0);   // 높이 축소
        double min_height_threshold = this->declare_parameter<double>("min_height_threshold", 0.15);
        double max_height_threshold = this->declare_parameter<double>("max_height_threshold", 1.1);
        int obstacle_threshold = this->declare_parameter<int>("obstacle_threshold", 2);
        int point_threshold = this->declare_parameter<int>("point_threshold", 1);
        double height_diff_threshold = this->declare_parameter<double>("height_diff_threshold", 0.01);
        std::string base_frame_id = this->declare_parameter<std::string>("base_frame_id", "base_link");
        bool auto_resize_map = this->declare_parameter<bool>("auto_resize_map", true);
        
        // 코스트맵 생성기에 파라미터 전달
        costmap_generator_->setParameters(
            costmap_resolution, costmap_width, costmap_height,
            min_height_threshold, max_height_threshold,
            obstacle_threshold, point_threshold, height_diff_threshold,
            base_frame_id, auto_resize_map
        );

    // Loop Closure 모듈 초기화 - 명시적으로 파라미터 전달
    double historyKeyframeSearchRadius = 10.0;
    int historyKeyframeSearchNum = 25;
    double historyKeyframeSearchTimeDiff = 30.0;
    double historyKeyframeFitnessScore = 0.3;
    
    // ParamServer에서 이미 선언된 파라미터를 가져옵니다
    this->get_parameter_or("historyKeyframeSearchRadius", historyKeyframeSearchRadius, historyKeyframeSearchRadius);
    this->get_parameter_or("historyKeyframeSearchNum", historyKeyframeSearchNum, historyKeyframeSearchNum);
    this->get_parameter_or("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, historyKeyframeSearchTimeDiff);
    this->get_parameter_or("historyKeyframeFitnessScore", historyKeyframeFitnessScore, historyKeyframeFitnessScore);
    
    RCLCPP_INFO(this->get_logger(), "Initializing Loop Closure with parameters: radius=%f, num=%d, timeDiff=%f, fitnessScore=%f",
        historyKeyframeSearchRadius, historyKeyframeSearchNum, historyKeyframeSearchTimeDiff, historyKeyframeFitnessScore);
    
    loop_closure_ = std::make_unique<LoopClosure>(
        this, 
        historyKeyframeSearchRadius,
        historyKeyframeSearchNum,
        historyKeyframeSearchTimeDiff,
        historyKeyframeFitnessScore
    );
        
        // 맵 저장 기능 초기화
        initializeMapSaver();
        
        RCLCPP_INFO(this->get_logger(), "Map Saver initialized with path: %s", map_save_path_.c_str());
}

void mapOptimization::allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

void mapOptimization::laserCloudInfoHandler(const liorf::msg::CloudInfo::SharedPtr msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.sec + msgIn->header.stamp.nanosec * 1e-9;

        // extract info and feature cloud
        cloudInfo = *msgIn;
        
        // 메모리 효율성을 위해 클라우드 복사 전 clear 수행
        if (laserCloudSurfLast) laserCloudSurfLast->clear();
        
        pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudSurfLast);

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval) {
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();

            // 주변 키프레임 추출 - 함수 복원
            extractSurroundingKeyFrames();

            // 포인트 클라우드 다운샘플링으로 메모리 사용량 및 계산량 감소
            downsampleCurrentScan();

            scan2MapOptimization();

            saveKeyFramesAndFactor();

            correctPoses();

            // 메모리 관리 기능 활성화 - 안전한 방식으로 메모리 관리
            // 10개 키프레임마다 메모리 정리 실행 (가독성 향상)
            static int frame_counter = 0;
            frame_counter++;
            if (cloudKeyPoses3D->size() > max_keyframes_in_memory_ && frame_counter >= 10) {
            clearOldFrames();
                frame_counter = 0;
            }

            publishOdometry();

            publishFrames();
        }
    
    // 코스트맵 생성기에 데이터 전달
    if (costmap_generator_ && !surfCloudKeyFrames.empty() && cloudKeyPoses3D && cloudKeyPoses6D) {
        // PCL의 points 멤버를 std::vector<PointTypePose>로 변환
        std::vector<PointTypePose> keyPoses6DVector(cloudKeyPoses6D->points.begin(), cloudKeyPoses6D->points.end());
        
        // 코스트맵 데이터 업데이트
        costmap_generator_->processClouds(cloudKeyPoses3D, surfCloudKeyFrames, keyPoses6DVector);
    }
    
    // Loop Closure 모듈에 데이터 전달
    if (loop_closure_ && !surfCloudKeyFrames.empty() && cloudKeyPoses3D && cloudKeyPoses6D) {
        loop_closure_->setInputData(cloudKeyPoses3D, cloudKeyPoses6D, surfCloudKeyFrames, timeLaserInfoCur);
    }
}

void mapOptimization::gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr gpsMsg)
    {
        if (gpsMsg->status.status != 0)
            return;

        Eigen::Vector3d trans_local_;
        static bool first_gps = false;
        if (!first_gps) {
            first_gps = true;
            gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        }

        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        nav_msgs::msg::Odometry gps_odom;
        gps_odom.header = gpsMsg->header;
        gps_odom.header.frame_id = "map";
        gps_odom.pose.pose.position.x = trans_local_[0]; //gps odom _ publish 방향 
        gps_odom.pose.pose.position.y = trans_local_[1];
        gps_odom.pose.pose.position.z = trans_local_[2];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, 0.0);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        gps_odom.pose.pose.orientation = quat_msg;
        pubGpsOdom->publish(gps_odom);
        gpsQueue.push_back(gps_odom);
    }

void mapOptimization::pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

    // pcl::getTransformation 대신 Eigen 사용
    Eigen::Affine3f transCur = Eigen::Affine3f::Identity();
    transCur.translation() << transformIn->x, transformIn->y, transformIn->z;
    
    Eigen::AngleAxisf rotX(transformIn->roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotY(transformIn->pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotZ(transformIn->yaw, Eigen::Vector3f::UnitZ());
    transCur.rotate(rotZ * rotY * rotX);
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

pcl::PointCloud<PointType>::Ptr mapOptimization::transformPointCloudWithLidarOffset(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

    // 기존 변환 행렬을 Eigen 함수를 사용하여 생성
    Eigen::Affine3f transCur = Eigen::Affine3f::Identity();
    transCur.translation() << transformIn->x, transformIn->y, transformIn->z;
    
    Eigen::AngleAxisf rotX(transformIn->roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotY(transformIn->pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotZ(transformIn->yaw, Eigen::Vector3f::UnitZ());
    transCur.rotate(rotZ * rotY * rotX);
        
        // 기본 LiDAR 오프셋 값 (TF를 찾지 못할 경우 사용)
        float lidarOffsetX = 0.23;
        float lidarOffsetY = 0.0;
        float lidarOffsetZ = 0.805;
        
        Eigen::Affine3f lidarOffset = Eigen::Affine3f::Identity();
        
        try {
            // TF에서 동적으로 base_link와 lidar_frame 사이의 변환 가져오기
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_->lookupTransform(baselinkFrame, lidarFrame, tf2::TimePointZero);
            
            // TransformStamped를 Eigen::Affine3f로 변환
            Eigen::Vector3f translation(
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z
            );
            
            Eigen::Quaternionf rotation(
                transformStamped.transform.rotation.w,
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z
            );
            
            lidarOffset.translation() = translation;
            lidarOffset.linear() = rotation.toRotationMatrix();
            
            // Quaternion에서 Roll, Pitch, Yaw 추출
            double roll, pitch, yaw;
            tf2::Quaternion tf2Quat(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w
            );
            tf2::Matrix3x3(tf2Quat).getRPY(roll, pitch, yaw);
            
            // 디버그 메시지 (필요시 주석 해제)
            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(), 
            //     *this->get_clock(),
            //     5000, // 5초마다 출력
            //     "TF에서 LiDAR 오프셋 가져옴: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f",
            //     translation.x(), translation.y(), translation.z(),
            //     roll, pitch, yaw
            // );
        }
    catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform from %s to %s: %s", baselinkFrame.c_str(), lidarFrame.c_str(), ex.what());
        
        // 기본값으로 변환 행렬 생성
        lidarOffset.translation() << lidarOffsetX, lidarOffsetY, lidarOffsetZ;
        }
        
        // 최종 변환 = 기존변환 * LiDAR오프셋
        Eigen::Affine3f finalTransform = transCur * lidarOffset;
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = finalTransform(0,0) * pointFrom.x + finalTransform(0,1) * pointFrom.y + finalTransform(0,2) * pointFrom.z + finalTransform(0,3);
            cloudOut->points[i].y = finalTransform(1,0) * pointFrom.x + finalTransform(1,1) * pointFrom.y + finalTransform(1,2) * pointFrom.z + finalTransform(1,3);
            cloudOut->points[i].z = finalTransform(2,0) * pointFrom.x + finalTransform(2,1) * pointFrom.y + finalTransform(2,2) * pointFrom.z + finalTransform(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

gtsam::Pose3 mapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

gtsam::Pose3 mapOptimization::trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

Eigen::Affine3f mapOptimization::pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

Eigen::Affine3f mapOptimization::trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

PointTypePose mapOptimization::trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

bool mapOptimization::saveMapService(const std::shared_ptr<liorf::srv::SaveMap::Request> req,
                                std::shared_ptr<liorf::srv::SaveMap::Response> res)
    {
      string saveMapDirectory;

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files ..." << endl;
      if(req->destination.empty()) {
          // yaml 파일에서 설정한 경로 사용
          saveMapDirectory = map_save_path_ + "/PCD";
      } else {
          // 사용자 지정 경로 사용 - 절대 경로인지 확인
          if (req->destination[0] == '/') {
              saveMapDirectory = req->destination;
          } else {
              saveMapDirectory = map_save_path_ + "/" + req->destination;
          }
      }
      cout << "Save destination: " << saveMapDirectory << endl;
      
      // create directory and remove old files;
      int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
      unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
      
      // save key frame transformations
      pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);

      // extract global point cloud map
      pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
      for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
          // LiDAR 오프셋을 고려한 변환 사용
          *globalSurfCloud   += *transformPointCloudWithLidarOffset(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
          cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
      }

      if(req->resolution != 0)
      {
        cout << "\n\nSave resolution: " << req->resolution << endl;
        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req->resolution, req->resolution, req->resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
      }
      else
      {

        // save surf cloud
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
      }

      // save global point cloud map
      *globalMapCloud += *globalSurfCloud;

      int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
      res->success = ret == 0;

      downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files completed\n" << endl;

      return true;
    }

void mapOptimization::visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(0.2);
        while (rclcpp::ok()){
            rate.sleep();
            publishGlobalMap();
        }

        if (savePCD == false)
            return;

        std::shared_ptr<liorf::srv::SaveMap::Request> req = std::make_unique<liorf::srv::SaveMap::Request>();
        std::shared_ptr<liorf::srv::SaveMap::Response> res = std::make_unique<liorf::srv::SaveMap::Response>();

        if(!saveMapService(req, res)){
            cout << "Fail to save map" << endl;
        }
    }

void mapOptimization::publishGlobalMap()
    {
        if (pubLaserCloudSurround->get_subscription_count() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        
        // 최소 10개의 키프레임 표시 보장
        if (globalMapKeyPosesDS->size() < 10 && cloudKeyPoses3D->size() > 10) {
            globalMapKeyPosesDS->clear();
            // 최근 10개 키프레임 추가
            for (int i = std::max(0, (int)cloudKeyPoses3D->size() - 10); i < cloudKeyPoses3D->size(); ++i) {
                globalMapKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            }
        }
        
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            // 오프셋을 고려한 변환 함수 사용
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }

void mapOptimization::scan2MapOptimization()
{
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudSurfLastDSNum > 30)
        {
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            RCLCPP_WARN(get_logger(), "Not enough features! Only %d planar features available.", laserCloudSurfLastDSNum);
        }
    }

void mapOptimization::transformUpdate()
    {
        if (cloudInfo.imuavailable == true && imuType)
        {
            if (std::abs(cloudInfo.imupitchinit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf2::Quaternion imuQuaternion;
                tf2::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

float mapOptimization::constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

bool mapOptimization::saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

void mapOptimization::addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

void mapOptimization::addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (common_lib_->pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (ROS_TIME(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (ROS_TIME(gpsQueue.front().header.stamp) > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (common_lib_->pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

void mapOptimization::addLoopFactor()
{
    if (loop_closure_ == nullptr || !loop_closure_->isLoopClosed())
        return;

    const auto& loopIndexQueue = loop_closure_->getLoopIndexQueue();
    const auto& loopPoseQueue = loop_closure_->getLoopPoseQueue();
    const auto& loopNoiseQueue = loop_closure_->getLoopNoiseQueue();
    
    // 루프 큐가 비어있으면 반환
    if (loopIndexQueue.empty())
        return;

    // 그래프에 루프 제약 추가
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        auto noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    // Loop Closure 완료 후 플래그 설정
    aLoopIsClosed = true;
}

void mapOptimization::saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        // 중요: intensity는 항상 배열 인덱스와 일치하도록 설정
        // 이렇게 하면 intensity와 벡터 인덱스가 항상 같아 인덱스 변환 문제가 발생하지 않음
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // 현재 배열 크기가 다음 인덱스
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // 동일한 인덱스 사용
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);
        
        // KdTree 업데이트
        if (cloudKeyPoses3D->size() > 0) {
            try {
                kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
                kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[키프레임] KdTree 업데이트 중 오류 발생: %s", e.what());
            }
        }

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // The following code is copy from sc-lio-sam
        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        
        // Loop Closure 모듈에 데이터 전달 - SC Manager 관련 작업도 Loop Closure 내부에서 처리
        if (loop_closure_ && !surfCloudKeyFrames.empty() && cloudKeyPoses3D && cloudKeyPoses6D) {
            // 현재 스캔 데이터도 함께 전달
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *thisRawCloudKeyFrame);
            
            // 루프 클로저에 데이터 전달 - Scan Context 생성 작업은 Loop Closure 내부에서 수행
            loop_closure_->setInputData(cloudKeyPoses3D, cloudKeyPoses6D, surfCloudKeyFrames, timeLaserInfoCur, thisRawCloudKeyFrame);
        }

        // save path for visualization
        updatePath(thisPose6D);
        
        // 실제 벡터 인덱스 - intensity 값은 항상 벡터 인덱스와 동일함
        int keyframeId = static_cast<int>(thisPose3D.intensity);
        int keyframeIdx = keyframeId; // 인덱스 = ID
        
        // 파일로 키프레임 저장 (일정 간격으로)
        if (use_keyframe_database_ && map_database_ && (keyframeId % save_interval_ == 0)) {
            // 로그 출력 제한 - 10개마다 상세 로그 출력
            static int save_counter = 0;
            save_counter++;
            bool log_details = (save_counter % 10 == 0);
            
            if (log_details) {
                RCLCPP_INFO(this->get_logger(), 
                           "┌─────────────────── 키프레임 저장 정보 ─────────────────┐");
                RCLCPP_INFO(this->get_logger(), 
                           "│ 키프레임 ID: %3d | 위치: %6.2f, %6.2f, %6.2f       │", 
                           keyframeId, thisPose6D.x, thisPose6D.y, thisPose6D.z);
            }
            
            // intensity 값과 벡터 인덱스가 항상 같으므로 안전함
            if (keyframeIdx >= 0 && keyframeIdx < static_cast<int>(surfCloudKeyFrames.size())) {       
                if (map_database_->saveKeyframe(keyframeId, thisPose6D, thisPose6D.time, surfCloudKeyFrames[keyframeIdx])) {
                    last_saved_keyframe_id_ = keyframeId;
                    
                    if (log_details) {
                        // 포인트 수 계산
                        int points_count = surfCloudKeyFrames[keyframeIdx]->size();
                        
                        RCLCPP_INFO(this->get_logger(), 
                                   "│ 저장 결과: 성공 | 포인트 수: %5d | 총 저장 수: %3d  │", 
                                   points_count, last_saved_keyframe_id_ + 1);
                        RCLCPP_INFO(this->get_logger(), 
                                   "└────────────────────────────────────────────────┘");
                    }
                } else if (log_details) {
                    RCLCPP_WARN(this->get_logger(), 
                               "│ 저장 결과: 실패 | DB 오류 발생                      │");
                    RCLCPP_INFO(this->get_logger(), 
                               "└────────────────────────────────────────────────┘");
                }
            } else if (log_details) {
                RCLCPP_ERROR(this->get_logger(), 
                            "│ 저장 실패: 인덱스 오류 (ID:%d, Idx:%d, Size:%zu)     │", 
                            keyframeId, keyframeIdx, surfCloudKeyFrames.size());
                RCLCPP_INFO(this->get_logger(), 
                           "└────────────────────────────────────────────────┘");
            }
        } else {
            // 저장되지 않는 이유 로깅은 제거 (불필요한 로그 감소)
        }
    }

void mapOptimization::correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

void mapOptimization::updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

void mapOptimization::publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        // Ref: http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        pubLaserOdometryGlobal->publish(laserOdometryROS);
        
        // Publish TF
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
        tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = "base_link";
        br->sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuavailable == true && imuType)
            {
                if (std::abs(cloudInfo.imupitchinit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf2::Quaternion imuQuaternion;
                    tf2::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imurollinit, 0, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imupitchinit, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);
    }

void mapOptimization::publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo->get_subscription_count() != 0)
        {
            // if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            // {
            //     liorf::msg::CloudInfo slamInfo;
            //     slamInfo.header.stamp = timeLaserInfoStamp;
            //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            //     *cloudOut += *laserCloudSurfLastDS;
            //     slamInfo.key_frame_cloud = publishCloud(rclcpp::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
            //     slamInfo.key_frame_poses = publishCloud(rclcpp::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
            //     pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
            //     *localMapOut += *laserCloudSurfFromMapDS;
            //     slamInfo.key_frame_map = publishCloud(rclcpp::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
            //     pubSLAMInfo->publish(slamInfo);
            //     lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            // }
        }
    }

    // 맵 크기 자동 조정 함수
void mapOptimization::updateMapSize(const pcl::PointCloud<PointType>::Ptr& cloud)
{
    // CostmapGenerator 내부에서 맵 크기와 경계 관리
    if (costmap_generator_) {
        costmap_generator_->updateMapSize(cloud);
    }
}

// 메모리 관리를 위한 새로운 함수: 오래된 키프레임 관리
void mapOptimization::clearOldFrames()
{
    // 안전을 위해 너무 많은 키프레임이 있는 경우만 처리
    if (cloudKeyPoses3D->size() <= max_keyframes_in_memory_) {
        return;
    }

    try {
        // 메모리 정리 실행 카운터 (정적 변수로 선언하여 함수 호출 간 유지)
        static int clean_counter = 0;
        clean_counter++;
        
        // 총 키프레임 수와 유지할 키프레임 수 확인
        int total_frames = cloudKeyPoses3D->size();
        int frames_to_keep = max_keyframes_in_memory_;
        int frames_to_remove = total_frames - frames_to_keep;
        
        // 현재 프로세스 메모리 사용량 확인 (Linux 시스템)
        long memory_usage_kb = 0;
        FILE* file = fopen("/proc/self/status", "r");
        if (file) {
            char line[128];
            while (fgets(line, 128, file) != NULL) {
                if (strncmp(line, "VmRSS:", 6) == 0) {
                    sscanf(line, "VmRSS: %ld", &memory_usage_kb);
                    break;
                }
            }
            fclose(file);
        }
        
        // 메모리 사용 정보 출력 (5회마다 또는 메모리 정리 시에만)
        bool should_log_verbose = (clean_counter % 5 == 0);
        
        if (should_log_verbose) {
            RCLCPP_INFO(this->get_logger(), 
                       "┌─────────────────── 메모리 관리 정보 ────────────────────┐");
            RCLCPP_INFO(this->get_logger(), 
                       "│ 키프레임: %4d개 | 메모리 제한: %4d개 | 제거 대상: %4d개 │", 
                       total_frames, frames_to_keep, frames_to_remove);
            RCLCPP_INFO(this->get_logger(), 
                       "│ 맵 컨테이너: %4zu개 | 메모리 사용량: %6.1f MB           │", 
                       laserCloudMapContainer.size(), memory_usage_kb / 1024.0);
        }
        
        // 가장 오래된 키프레임 ID 확인 (정리 대상)
        std::vector<int> old_frame_ids;
        for (int i = 0; i < frames_to_remove; i++) {
            if (i < total_frames) {
                old_frame_ids.push_back(static_cast<int>(cloudKeyPoses3D->points[i].intensity));
            }
        }
        
        // 맵 컨테이너에서 오래된 항목 제거
        int cleared_container = 0;
        for (int id : old_frame_ids) {
            if (laserCloudMapContainer.find(id) != laserCloudMapContainer.end()) {
                laserCloudMapContainer.erase(id);
                cleared_container++;
            }
        }
        
        // 맵 캐시를 30% 정도만 유지 (과도한 메모리 사용 방지)
        if (laserCloudMapContainer.size() > frames_to_keep * 1.3) {
            int excess_cache = laserCloudMapContainer.size() - frames_to_keep;
            int items_to_remove = std::min(excess_cache, frames_to_remove);
            
            // ID 기준으로 정렬된 맵 복사본 생성
            std::vector<int> cache_ids;
            for (auto& item : laserCloudMapContainer) {
                cache_ids.push_back(item.first);
            }
            
            // 정렬
            std::sort(cache_ids.begin(), cache_ids.end());
            
            // 가장 오래된 캐시 항목 제거
            int extra_cleared = 0;
            for (int i = 0; i < items_to_remove && i < static_cast<int>(cache_ids.size()); i++) {
                laserCloudMapContainer.erase(cache_ids[i]);
                extra_cleared++;
            }
            
            if (extra_cleared > 0 && should_log_verbose) {
                RCLCPP_INFO(this->get_logger(), "│ 추가 캐시 정리: %4d개 항목 제거                        │", extra_cleared);
                cleared_container += extra_cleared;
            }
        }
        
        // 메모리 정리 결과 출력
        if (should_log_verbose) {
            RCLCPP_INFO(this->get_logger(), 
                       "│ 정리 결과: %4d개 항목 제거됨 | 남은 항목: %4zu개         │", 
                       cleared_container, laserCloudMapContainer.size());
            RCLCPP_INFO(this->get_logger(), 
                       "└────────────────────────────────────────────────────────┘");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[메모리관리] 메모리 정리 중 오류 발생: %s", e.what());
    }
}

void mapOptimization::pruneMemoryKeyframes()
{
    // 메모리 관리를 완전히 비활성화 - 문제 해결을 위한 임시 조치
    if (true) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), 
                             *this->get_clock(), 
                             10000, // 10초마다 출력
                             "[메모리관리] 메모리 관리 비활성화됨 - 현재 키프레임: %zu개", cloudKeyPoses3D->size());
        return;
    }
    
    // 이하는 사용하지 않음
    if (cloudKeyPoses3D->size() <= max_keyframes_in_memory_) {
        return;
    }
    
    std::string memory_mode = "batch_clear";
    if (this->has_parameter("memory_management_mode")) {
        this->get_parameter("memory_management_mode", memory_mode);
    }
    
    // 모드에 따라 처리
    if (memory_mode == "sliding_window") {
        // 비활성화 - 오류 발생
        RCLCPP_WARN_THROTTLE(this->get_logger(), 
                             *this->get_clock(), 
                             10000, // 10초마다 출력
                             "[메모리관리] 슬라이딩 윈도우 모드 비활성화됨");
    } else {
        // 배치 모드 - 30%의 오래된 키프레임 제거
        int count = cloudKeyPoses3D->size();
        if (count > max_keyframes_in_memory_) {
            RCLCPP_INFO(this->get_logger(), "[메모리관리] 배치 정리 시작: 총 %d개 키프레임", count);
            // 추가 작업 비활성화
        }
    }
}

void mapOptimization::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    
    // 주변 키프레임 추출 결과를 저장할 변수들 초기화
    if (laserCloudSurfFromMap) laserCloudSurfFromMap->clear();
    
    // 빈 포인트 클라우드인 경우 처리
    if (cloudKeyPoses3D->size() == 0) {
        RCLCPP_WARN(this->get_logger(), "[맵생성] 키프레임이 없습니다");
        return;
    }
    
    // extract all the nearby key poses and downsample them
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // 안전 방식: 기본적으로 최근 10개의 키프레임 사용
    int numPoses = cloudKeyPoses3D->size();
    int recentPosesToUse = std::min(10, numPoses);
    
    for (int i = numPoses - recentPosesToUse; i < numPoses; i++) {
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[i]);
    }
    
    // KdTree 검색 시도 (실패 시 최근 키프레임만 사용)
    try {
        if (numPoses > 10) { // KdTree 사용이 의미있을 때만 시도
            // KdTree를 최신 데이터로 업데이트
            kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        
            // 현재 위치 주변 키프레임 검색
            if (kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis) > 0) {
                // 각 점을 추가
                for (int i = 0; i < (int)pointSearchInd.size(); ++i) {
                    int idx = pointSearchInd[i];
                    if (idx >= 0 && idx < static_cast<int>(cloudKeyPoses3D->size())) {
                        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[idx]);
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[맵생성] KdTree 검색 실패, 최근 키프레임만 사용: %s", e.what());
        // 이미 최근 키프레임은 추가되어 있으므로 추가 작업 불필요
    }

    // 다운샘플링 수행 (안전하게 시도)
    try {
        if (surroundingKeyPoses->size() > 10) {
            downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
            downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        } else {
            // 다운샘플링 불필요하면 그대로 복사
            *surroundingKeyPosesDS = *surroundingKeyPoses;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[맵생성] 다운샘플링 실패, 원본 사용: %s", e.what());
        // 안전하게 원본 사용
        *surroundingKeyPosesDS = *surroundingKeyPoses;
    }
    
    // 안전 확인: 다운샘플링 결과가 비어있으면 최근 키프레임 사용
    if (surroundingKeyPosesDS->empty() && numPoses > 0) {
        RCLCPP_WARN(this->get_logger(), "[맵생성] 다운샘플링 결과 비어있음, 최신 키프레임 사용");
        surroundingKeyPosesDS->push_back(cloudKeyPoses3D->back());
    }

    // 이후 extractCloud 함수 내용과 유사하게 변환
    laserCloudSurfFromMap->clear(); 
    
    // 메모리에 있는 키프레임 데이터 합치기
    int addedClouds = 0;
    for (int i = 0; i < (int)surroundingKeyPosesDS->size(); ++i)
    {
        // 키프레임 ID 얻기
        PointType& point = surroundingKeyPosesDS->points[i];
        int keyframeId = static_cast<int>(point.intensity);
        
        // ID가 유효한지 확인
        if (keyframeId < 0 || keyframeId >= static_cast<int>(surfCloudKeyFrames.size())) {
            continue;
        }
        
        // laserCloudMapContainer에 변환된 포인트 클라우드 저장 (메모리 관리용)
        if (laserCloudMapContainer.find(keyframeId) == laserCloudMapContainer.end()) {
            // 변환된 클라우드가 없으면 새로 생성
            pcl::PointCloud<PointType> laserCloudCornerTemp;
            pcl::PointCloud<PointType> laserCloudSurfTemp;
            
            try {
                laserCloudSurfTemp = *transformPointCloud(
                    surfCloudKeyFrames[keyframeId], 
                    &cloudKeyPoses6D->points[keyframeId]
                );
                // 맵 컨테이너에 저장
                laserCloudMapContainer[keyframeId] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
                
                // 맵에 추가
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                addedClouds++;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[맵생성] 포인트 클라우드 변환 중 오류 발생: %s (ID: %d)", 
                            e.what(), keyframeId);
            }
        } else {
            // 이미 변환된 클라우드가 있으면 재사용
            *laserCloudSurfFromMap += laserCloudMapContainer[keyframeId].second;
            addedClouds++;
        }
    }
    
    if (addedClouds == 0) {
        RCLCPP_WARN(this->get_logger(), "[맵생성] 추가된 클라우드가 없습니다!");
        return;
    }

    // 다운샘플링으로 포인트 클라우드 크기 줄이기
    try {
        downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[맵생성] 최종 다운샘플링 실패: %s", e.what());
        // 실패 시 그대로 사용
        *laserCloudSurfFromMapDS = *laserCloudSurfFromMap;
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    }
    
    RCLCPP_DEBUG(this->get_logger(), "[맵생성] 최종 맵 포인트 수: %d, 맵 캐시 크기: %zu", 
                laserCloudSurfFromMapDSNum, laserCloudMapContainer.size());
}

void mapOptimization::updateInitialGuess()
{
    // save current transformation before any processing
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    static Eigen::Affine3f lastImuTransformation;
    // initialization
    if (cloudKeyPoses3D->points.empty())
    {
        transformTobeMapped[0] = cloudInfo.imurollinit;
        transformTobeMapped[1] = cloudInfo.imupitchinit;
        transformTobeMapped[2] = cloudInfo.imuyawinit;

        if (!useImuHeadingInitialization)
            transformTobeMapped[2] = 0;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
        return;
    }

    // use imu pre-integration estimation for pose guess
    static bool lastImuPreTransAvailable = false;
    static Eigen::Affine3f lastImuPreTransformation;
    if (cloudInfo.odomavailable == true)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialguessx,    cloudInfo.initialguessy,     cloudInfo.initialguessz, 
                                                        cloudInfo.initialguessroll, cloudInfo.initialguesspitch, cloudInfo.initialguessyaw);
        if (lastImuPreTransAvailable == false)
        {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        } else {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuPreTransformation = transBack;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
            return;
        }
    }

    // use imu incremental estimation for pose guess (only rotation)
    if (cloudInfo.imuavailable == true && imuType)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                    transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imurollinit, cloudInfo.imupitchinit, cloudInfo.imuyawinit); // save imu before return;
        return;
    }
}

void mapOptimization::extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int id = pointSearchInd[i];
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }

    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    for(auto& pt : surroundingKeyPosesDS->points)
    {
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }

    // also extract some latest key frames in case the robot rotates in one position
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses-1; i >= 0; --i)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    extractCloud(surroundingKeyPosesDS);
}

void mapOptimization::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudSurfFromMap->clear(); 
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        if (common_lib_->pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
        {
            // transformed cloud available
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        } else {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp;
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudSurfFromMap   += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
        
    }

    // Downsample the surrounding surf key frames (or map)
    downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

    // clear map cache if too large
    if (laserCloudMapContainer.size() > 1000)
        laserCloudMapContainer.clear();
}

void mapOptimization::updatePointAssociateToMap()
{
    transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
}

void mapOptimization::surfOptimization()
{
    updatePointAssociateToMap();

    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel); 
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                         pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                         pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                        + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

void mapOptimization::combineOptimizationCoeffs()
{
    // combine surf coeffs
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
        if (laserCloudOriSurfFlag[i] == true){
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
        }
    }
    // reset flag for next iteration
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool mapOptimization::LMOptimization(int iterCount)
{
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(transformTobeMapped[2]);
    float crx = cos(transformTobeMapped[2]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50) {
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++) {
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].x;
        pointOri.y = laserCloudOri->points[i].y;
        pointOri.z = laserCloudOri->points[i].z;
        // lidar -> camera
        coeff.x = coeffSel->points[i].x;
        coeff.y = coeffSel->points[i].y;
        coeff.z = coeffSel->points[i].z;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
    /*     float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
             */

        float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
                  + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

        float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
                  + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
                  + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

        float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
                  + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
                  + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;
          
        // camera -> lidar
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arx;
        matA.at<float>(i, 3) = coeff.x;
        matA.at<float>(i, 4) = coeff.y;
        matA.at<float>(i, 5) = coeff.z;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {

        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(
                       pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                       pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                       pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
                       pow(matX.at<float>(3, 0) * 100, 2) +
                       pow(matX.at<float>(4, 0) * 100, 2) +
                       pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
        return true; // converged
    }
    return false; // keep optimizing
}

void mapOptimization::initializeDatabase()
{
    // 데이터베이스 객체 생성
    map_database_ = std::make_unique<MapDatabase>(this, db_path_, cloud_storage_path_);
    
    // 데이터베이스 초기화
    if (!map_database_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize map database. Using in-memory mode only.");
        return;
    }
    
    // 기존 포즈 데이터 로드
    std::vector<PointTypePose> poses;
    int loaded_poses = map_database_->loadAllKeyframePoses(poses);
    
    if (loaded_poses > 0) {
        RCLCPP_INFO(this->get_logger(), "Loaded %d keyframe poses from database", loaded_poses);
        
        // 최근 일부 키프레임만 메모리에 로드
        int poses_to_load = std::min(loaded_poses, max_keyframes_in_memory_);
        std::vector<int> recent_ids;
        map_database_->getRecentKeyframeIds(poses_to_load, recent_ids);
        
        for (int id : recent_ids) {
            PointTypePose pose;
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
            
            if (map_database_->loadKeyframe(id, pose, cloud)) {
                // 키프레임 데이터 메모리에 추가
                cloudKeyPoses3D->push_back(PointType{pose.x, pose.y, pose.z, pose.intensity});
                cloudKeyPoses6D->push_back(pose);
                surfCloudKeyFrames.push_back(cloud);
                last_saved_keyframe_id_ = id;
            }
        }
        
        // 경로 업데이트
        for (const auto& pose : poses) {
            updatePath(pose);
        }
    }
}

// saveKeyframeToDisk 함수는 제거되고 대신 MapDatabase::saveKeyframe 사용

bool mapOptimization::loadKeyframeFromDB(int keyframe_id, PointTypePose& pose, pcl::PointCloud<PointType>::Ptr& cloud)
{
    if (!map_database_) {
        return false;
    }
    
    return map_database_->loadKeyframe(keyframe_id, pose, cloud);
}

// 마지막 loadSurroundingKeyframesFromDB 함수 정의 사용

void mapOptimization::loadSurroundingKeyframesFromDB()
{
    if (!map_database_ || cloudKeyPoses3D->empty()) {
        return;
    }
    
    // 현재 위치 주변의 키프레임 검색
    const auto& currentPose = cloudKeyPoses3D->back();
    std::vector<int> surrounding_ids;
    
    int found = map_database_->querySurroundingKeyframes(
        currentPose.x, 
        currentPose.y, 
        currentPose.z, 
        surroundingKeyframeSearchRadius,
        surrounding_ids
    );
    
    if (found == 0) {
        return;
    }
    
    // 메모리에 없는 키프레임 로드
    for (int id : surrounding_ids) {
        // 이미 메모리에 있는지 확인
        bool already_in_memory = false;
        for (const auto& pose : cloudKeyPoses6D->points) {
            if (static_cast<int>(pose.intensity) == id) {
                already_in_memory = true;
                break;
            }
        }
        
        // 메모리에 없으면 로드
        if (!already_in_memory) {
            PointTypePose pose;
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
            
            if (loadKeyframeFromDisk(id, pose, cloud)) {
                // laserCloudMapContainer에 추가
                pcl::PointCloud<PointType> laserCloudCornerTemp;
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(cloud, &pose);
                laserCloudMapContainer[id] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }
    }
}

// main 함수는 별도 파일(src/main.cpp)로 분리되었습니다.
void mapOptimization::initializeMapSaver()
{
    // 맵 저장 경로 설정
    if (!this->has_parameter("map_save_path")) {
        map_save_path_ = this->declare_parameter<std::string>("map_save_path", "/liorf_ws2/src/liorf/maps");
    } else {
        this->get_parameter("map_save_path", map_save_path_);
    }
    
    // 포인트 클라우드 저장 경로 설정
    if (!this->has_parameter("cloud_storage_path")) {
        cloud_storage_path_ = this->declare_parameter<std::string>("cloud_storage_path", map_save_path_ + "/clouds");
    } else {
        this->get_parameter("cloud_storage_path", cloud_storage_path_);
    }
    
    // 파일 저장 관련 파라미터
    if (!this->has_parameter("save_keyframe_interval")) {
        save_interval_ = this->declare_parameter<int>("save_keyframe_interval", 5);
    } else {
        this->get_parameter("save_keyframe_interval", save_interval_);
    }
    
    if (!this->has_parameter("max_keyframes_in_memory")) {
        max_keyframes_in_memory_ = this->declare_parameter<int>("max_keyframes_in_memory", 100);
    } else {
        this->get_parameter("max_keyframes_in_memory", max_keyframes_in_memory_);
    }
    
    if (!this->has_parameter("use_keyframe_database")) {
        use_keyframe_database_ = this->declare_parameter<bool>("use_keyframe_database", true);
    } else {
        this->get_parameter("use_keyframe_database", use_keyframe_database_);
    }
    
    // 메모리 관리 모드 설정
    std::string memory_mode = "sliding_window";
    if (this->has_parameter("memory_management_mode")) {
        this->get_parameter("memory_management_mode", memory_mode);
    } else {
        this->declare_parameter<std::string>("memory_management_mode", "sliding_window");
        memory_mode = "sliding_window";
    }
    
    // 맵 초기화 옵션 설정
    bool clear_map = true;
    if (this->has_parameter("clear_map_on_start")) {
        this->get_parameter("clear_map_on_start", clear_map);
    } else {
        this->declare_parameter<bool>("clear_map_on_start", true);
        clear_map = true;
    }
    
    RCLCPP_INFO(this->get_logger(), "메모리 관리 모드: %s, 맵 초기화: %s", 
               memory_mode.c_str(), clear_map ? "활성화" : "비활성화");
    
    // 저장 디렉토리가 없으면 생성
    rcpputils::fs::path map_path(map_save_path_);
    rcpputils::fs::path cloud_path(cloud_storage_path_);
    
    try {
        if (!rcpputils::fs::exists(map_path)) {
            rcpputils::fs::create_directories(map_path);
        }
        
        if (!rcpputils::fs::exists(cloud_path)) {
            rcpputils::fs::create_directories(cloud_path);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "디렉토리 생성 실패: %s", e.what());
    }
    
    // MapDatabase 초기화
    map_database_ = std::make_unique<MapDatabase>(this, map_save_path_, cloud_storage_path_);
    
    if (map_database_->initialize()) {
        RCLCPP_INFO(this->get_logger(), "[초기화] MapDatabase 초기화 성공: %s (저장 간격: %d, 최대 메모리 키프레임: %d)", 
                   map_save_path_.c_str(), save_interval_, max_keyframes_in_memory_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "[초기화] MapDatabase 초기화 실패");
    }
    
    RCLCPP_INFO(this->get_logger(), "Map Saver initialized with path: %s", map_save_path_.c_str());
}

bool mapOptimization::loadKeyframeFromDisk(int keyframe_id, PointTypePose& pose, pcl::PointCloud<PointType>::Ptr& cloud)
{
    if (!map_database_) {
        return false;
    }
    
    return map_database_->loadKeyframe(keyframe_id, pose, cloud);
}

void mapOptimization::downsampleCurrentScan()
{
    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}
