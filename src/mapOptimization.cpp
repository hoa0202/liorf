#include "utility.h"
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
#include "costmap.h" // 코스트맵 생성기 클래스 헤더 추가
#include "loopclosure.h" // Loop Closure 클래스 헤더 추가

#include "mapOptimization.h"

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
        
        RCLCPP_INFO(this->get_logger(), "Costmap integration initialized with resolution: %.2f, size: %.1fm x %.1fm, origin: (%.2f, %.2f)",
                   costmap_resolution, costmap_width, costmap_height, -costmap_width/2, -costmap_height/2);

        // 데이터베이스 관련 기능 초기화
        use_database_mode_ = this->declare_parameter<bool>("use_database_mode", false);
        localization_mode_ = this->declare_parameter<bool>("localization_mode", false);  // 로컬라이제이션 모드 파라미터 추가
        active_keyframes_window_size_ = this->declare_parameter<int>("active_keyframes_window_size", 100);
        active_loop_features_window_size_ = this->declare_parameter<int>("active_loop_features_window_size", 100);
        spatial_query_radius_ = this->declare_parameter<double>("spatial_query_radius", 10.0);
        
        if (use_database_mode_) {
            RCLCPP_INFO(this->get_logger(), "데이터베이스 모드 활성화: %s", 
                       localization_mode_ ? "로컬라이제이션 모드" : "매핑 모드");
            
            try {
                // 데이터베이스 경로 및 설정 가져오기
                std::string db_path = this->declare_parameter<std::string>("database_path", "");
                std::string clouds_directory = this->declare_parameter<std::string>("clouds_directory", "");
                bool database_reset_on_start = this->declare_parameter<bool>("database_reset_on_start", true);
                
                // 절대 경로 변환
                if (!db_path.empty() && db_path[0] != '/') {
                    if (db_path.find("~/") == 0) {
                        db_path = std::string(std::getenv("HOME")) + db_path.substr(1);
                    } else {
                        char cwd[1024];
                        if (getcwd(cwd, sizeof(cwd)) != NULL) {
                            db_path = std::string(cwd) + "/" + db_path;
                        }
                    }
                }
                
                if (!clouds_directory.empty() && clouds_directory[0] != '/') {
                    if (clouds_directory.find("~/") == 0) {
                        clouds_directory = std::string(std::getenv("HOME")) + clouds_directory.substr(1);
                    } else {
                        char cwd[1024];
                        if (getcwd(cwd, sizeof(cwd)) != NULL) {
                            clouds_directory = std::string(cwd) + "/" + clouds_directory;
                        }
                    }
                }
                
                RCLCPP_INFO(this->get_logger(), "최종 데이터베이스 경로: %s", db_path.c_str());
                RCLCPP_INFO(this->get_logger(), "최종 클라우드 디렉토리: %s", clouds_directory.c_str());
                RCLCPP_INFO(this->get_logger(), "데이터베이스 초기화 모드: %s", database_reset_on_start ? "완전 초기화" : "연속 매핑");
                RCLCPP_INFO(this->get_logger(), "메모리 키프레임 제한: %d, 메모리 루프 특징점 제한: %d", 
                           active_keyframes_window_size_, active_loop_features_window_size_);
                RCLCPP_INFO(this->get_logger(), "공간 쿼리 반경: %.2f", spatial_query_radius_);
                RCLCPP_INFO(this->get_logger(), "로컬라이제이션 모드: %s", localization_mode_ ? "활성화" : "비활성화");
                
                db_manager_ = std::make_unique<DBManager>(
                    this,
                    db_path,
                    active_keyframes_window_size_,
                    spatial_query_radius_,
                    clouds_directory,
                    database_reset_on_start,
                    localization_mode_  // 로컬라이제이션 모드 전달
                );
                
                // 루프 특징점 메모리 제한 설정
                if (db_manager_) {
                    db_manager_->setMaxMemoryLoopFeatures(active_loop_features_window_size_);
                }
                
                if (!db_manager_->initialize()) {
                    RCLCPP_ERROR(this->get_logger(), "데이터베이스 초기화 실패, 데이터베이스 모드 비활성화");
                    db_manager_.reset();
                    use_database_mode_ = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "데이터베이스 관리자 초기화 성공");
                    db_manager_->startMemoryMonitoring();
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "데이터베이스 초기화 중 예외 발생: %s", e.what());
                db_manager_.reset();
                use_database_mode_ = false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "메모리 기반 모드로 실행 중 (데이터베이스 비활성화)");
        }

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
    
    // 루프 클로저 초기화 - DB 관리자 전달
    loop_closure_ = std::make_unique<LoopClosure>(
        this,
        historyKeyframeSearchRadius,
        historyKeyframeSearchNum,
        historyKeyframeSearchTimeDiff,
        historyKeyframeFitnessScore,
        use_database_mode_ ? db_manager_.get() : nullptr  // DB 관리자 전달
    );
    
    // DB 모드 설정
    if (loop_closure_) {
        loop_closure_->setDBMode(use_database_mode_);
        RCLCPP_INFO(this->get_logger(), "Loop Closure 모듈 DB 모드: %s", use_database_mode_ ? "활성화" : "비활성화");
    }
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

            // 메모리 관리: 너무 오래된 프레임 제거
            clearOldFrames();

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
    if (loop_closure_ && cloudKeyPoses3D && cloudKeyPoses6D) {
        if (use_database_mode_ && db_manager_ && db_manager_->isInitialized()) {
            // DB 모드 - 포즈 정보만 전달
            loop_closure_->setInputDataWithDB(cloudKeyPoses3D, cloudKeyPoses6D, timeLaserInfoCur);
            RCLCPP_DEBUG(this->get_logger(), "Publishing map: Loop closure using DB mode");
        } else if (!surfCloudKeyFrames.empty()) {
            // 기존 메모리 모드
            loop_closure_->setInputData(cloudKeyPoses3D, cloudKeyPoses6D, surfCloudKeyFrames, timeLaserInfoCur);
            RCLCPP_DEBUG(this->get_logger(), "Publishing map: Loop closure using memory-only mode");
        }
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
      if(req->destination.empty()) saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
      else saveMapDirectory = std::getenv("HOME") + req->destination;
      cout << "Save destination: " << saveMapDirectory << endl;
      // create directory and remove old files;
      (void)system((std::string("exec rm -r ") + saveMapDirectory).c_str());
      (void)system((std::string("mkdir -p ") + saveMapDirectory).c_str());
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
            size_t cloud_size = cloudKeyPoses3D->size();
            for (size_t i = (cloud_size > 10) ? (cloud_size - 10) : 0; i < cloud_size; ++i) {
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
            if (thisKeyInd >= 0 && static_cast<size_t>(thisKeyInd) < surfCloudKeyFrames.size()) {
                *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            }
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
    for (size_t i = 0; i < loopIndexQueue.size(); ++i)
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

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

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
        
        // 데이터베이스에 키프레임 및 포즈 저장
        if (use_database_mode_ && db_manager_ && db_manager_->isInitialized()) {
            int keyframe_id = cloudKeyPoses3D->size() - 1;
            PointTypePose pose = cloudKeyPoses6D->points[keyframe_id];
            
            // 현재 프레임의 포인트 클라우드
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudSurfLastDS, *thisKeyFrame);
            
            // 데이터베이스에 저장
            if (!db_manager_->addKeyFrame(keyframe_id, timeLaserInfoCur, pose, thisKeyFrame)) {
                RCLCPP_ERROR(this->get_logger(), "키프레임 ID %d를 데이터베이스에 저장하지 못했습니다", keyframe_id);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "키프레임 ID %d가 데이터베이스에 저장되었습니다", keyframe_id);
            }
            
            // 메모리 관리: 키프레임 수가 지정된 한계를 초과하면 오래된 프레임 정리
            if (static_cast<int>(surfCloudKeyFrames.size()) > active_keyframes_window_size_) {
                clearOldFrames();
            }
            
            // 활성 윈도우 업데이트
            updateActiveWindow(thisPose6D);
        }

        // Loop Closure 모듈에 데이터 전달 - SC Manager 관련 작업도 Loop Closure 내부에서 처리
        if (loop_closure_) {
            if (use_database_mode_ && db_manager_ && db_manager_->isInitialized()) {
                // DB 모드 - 포즈 정보만 전달
                pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
                pcl::fromROSMsg(cloudInfo.cloud_deskewed, *thisRawCloudKeyFrame);
                loop_closure_->setInputDataWithDB(cloudKeyPoses3D, cloudKeyPoses6D, timeLaserInfoCur, thisRawCloudKeyFrame);
                RCLCPP_DEBUG(this->get_logger(), "Loop closure using DB mode");
            } else {
                // 기존 메모리 모드
                pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
                pcl::fromROSMsg(cloudInfo.cloud_deskewed, *thisRawCloudKeyFrame);
                loop_closure_->setInputData(cloudKeyPoses3D, cloudKeyPoses6D, surfCloudKeyFrames, timeLaserInfoCur, thisRawCloudKeyFrame);
                RCLCPP_DEBUG(this->get_logger(), "Loop closure using memory-only mode");
            }
        }

        // save path for visualization
        updatePath(thisPose6D);
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
            tf2::Quaternion quat_tf_inc;
            quat_tf_inc.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg_inc;
            tf2::convert(quat_tf_inc, quat_msg_inc);
            laserOdomIncremental.pose.pose.orientation = quat_msg_inc;
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
        // static int lastSLAMInfoPubSize = -1; // unused variable
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
    // 최대 키프레임 수 제한 (메모리 사용량 조절)
    const int maxKeyframeSize = 1000;  // 1000 -> 800
    
    if (cloudKeyPoses6D->size() > maxKeyframeSize) {
        // 조기 반환으로 불필요한 연산 방지
        if (cloudKeyPoses6D->size() <= maxKeyframeSize + 50) return;
        
        // 맵 컨테이너에서만 가장 오래된 데이터 삭제 (정확도 유지)
        for (int i = 0; i < 50; ++i) {
            int oldestIdx = i;
            laserCloudMapContainer.erase(oldestIdx);
        }
    }
    
    // 데이터베이스 모드일 경우 활성 윈도우 업데이트
    if (use_database_mode_ && db_manager_ && db_manager_->isInitialized() && !cloudKeyPoses6D->empty()) {
        // 현재 위치 기준으로 필요한 키프레임만 메모리에 유지
        PointTypePose currentPose = cloudKeyPoses6D->back();
        db_manager_->updateActiveWindow(currentPose);
        
        // 루프 특징점 활성 윈도우도 업데이트
        if (loop_closure_) {
            loop_closure_->updateActiveLoopFeatureWindow(currentPose);
        }
    }
}

void mapOptimization::updateActiveWindow(const PointTypePose& current_pose) {
    if (!use_database_mode_ || !db_manager_ || !db_manager_->isInitialized()) {
        return;
    }
    
    // 현재 포즈 정보를 사용하여 활성 윈도우 업데이트
    db_manager_->updateActiveWindow(current_pose);
}

void mapOptimization::loadKeyFramesFromDB() {
    if (!use_database_mode_ || !db_manager_ || !db_manager_->isInitialized()) {
        return;
    }
    
    // 현재 위치 근처의 키프레임 로드
    PointTypePose currentPose = trans2PointTypePose(transformTobeMapped);
    std::vector<int> nearbyFrameIds = db_manager_->loadKeyFramesByRadius(
        currentPose, 
        spatial_query_radius_, 
        active_keyframes_window_size_
    );
    
    if (nearbyFrameIds.empty()) {
        RCLCPP_INFO(this->get_logger(), "근처에 로드할 키프레임 없음");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "%lu개의 키프레임 로드됨", nearbyFrameIds.size());
    
    // 로드된 키프레임 메모리에 추가
    for (const auto& id : nearbyFrameIds) {
        // 이미 메모리에 있는지 확인
        bool already_loaded = false;
        for (const auto& point : cloudKeyPoses3D->points) {
            if (static_cast<int>(point.intensity) == id) {
                already_loaded = true;
                break;
            }
        }
        
        if (!already_loaded) {
            pcl::PointCloud<PointType>::Ptr cloud = db_manager_->loadCloud(id);
            if (cloud && cloud->size() > 0) {
                // 메모리에 추가
                surfCloudKeyFrames.push_back(cloud);
                
                // 키프레임 정보 쿼리해서 포즈 추가
                std::string sql = "SELECT x, y, z, roll, pitch, yaw, timestamp FROM keyframes WHERE id = ?;";
                
                sqlite3_stmt* stmt;
                int rc = sqlite3_prepare_v2(db_manager_->getDB(), sql.c_str(), -1, &stmt, nullptr);
                if (rc != SQLITE_OK) {
                    RCLCPP_ERROR(this->get_logger(), "SQL 준비 실패: %s", sqlite3_errmsg(db_manager_->getDB()));
                    continue;
                }
                
                sqlite3_bind_int(stmt, 1, id);
                
                if (sqlite3_step(stmt) == SQLITE_ROW) {
                    double x = sqlite3_column_double(stmt, 0);
                    double y = sqlite3_column_double(stmt, 1);
                    double z = sqlite3_column_double(stmt, 2);
                    double roll = sqlite3_column_double(stmt, 3);
                    double pitch = sqlite3_column_double(stmt, 4);
                    double yaw = sqlite3_column_double(stmt, 5);
                    double timestamp = sqlite3_column_double(stmt, 6);
                    
                    // 포즈 정보 추가
                    PointType pose3D;
                    pose3D.x = x;
                    pose3D.y = y;
                    pose3D.z = z;
                    pose3D.intensity = id;
                    cloudKeyPoses3D->push_back(pose3D);
                    
                    PointTypePose pose6D;
                    pose6D.x = x;
                    pose6D.y = y;
                    pose6D.z = z;
                    pose6D.roll = roll;
                    pose6D.pitch = pitch;
                    pose6D.yaw = yaw;
                    pose6D.time = timestamp;
                    pose6D.intensity = id;
                    cloudKeyPoses6D->push_back(pose6D);
                    
                    RCLCPP_DEBUG(this->get_logger(), "키프레임 ID %d 로드됨", id);
                }
                
                sqlite3_finalize(stmt);
            }
        }
    }
    
    // 주변 키프레임 맵 업데이트
    if (!nearbyFrameIds.empty()) {
        extractSurroundingKeyFrames();
    }
}

void mapOptimization::initializeDBManager() {
    if (!use_database_mode_ || !db_manager_) {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "데이터베이스에서 이전 맵 데이터 로드 중...");
    
    // 데이터베이스에서 전역 맵 로드
    pcl::PointCloud<PointType>::Ptr global_map = db_manager_->loadGlobalMap(mappingSurfLeafSize);
    if (global_map && global_map->size() > 0) {
        // 전역 맵 발행
        sensor_msgs::msg::PointCloud2 globalMapMsg;
        pcl::toROSMsg(*global_map, globalMapMsg);
        globalMapMsg.header.stamp = this->now();
        globalMapMsg.header.frame_id = odometryFrame;
        pubLaserCloudSurround->publish(globalMapMsg);
        
        RCLCPP_INFO(this->get_logger(), "전역 맵 로드 완료: %lu 포인트", global_map->size());
    } else {
        RCLCPP_INFO(this->get_logger(), "로드할 전역 맵 없음, 새 맵 생성");
    }
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
            if (static_cast<size_t>(thisKeyInd) < surfCloudKeyFrames.size()) {
                pcl::PointCloud<PointType> laserCloudCornerTemp;
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
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

void mapOptimization::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() && !localization_mode_) // 로컬라이제이션 모드가 아니면서 키프레임이 없으면 반환
        return;
    
    // 주변 맵 포인트 클라우드 초기화
    if (laserCloudSurfFromMap) laserCloudSurfFromMap->clear();

    // DB 모드가 활성화되었고, DBManager가 정상 작동 중일 때
    if (use_database_mode_ && db_manager_ && db_manager_->isInitialized())
    {
        // =========================================================================
        // === 데이터베이스를 활용한 고속 로컬 맵 생성 (병목 현상 해결의 핵심) ===
        // =========================================================================
        std::set<int> surrounding_keyframe_indices;
        
        // 현재 로봇의 추정 포즈
        PointTypePose current_pose = trans2PointTypePose(transformTobeMapped);

        // 1. DB에서 공간적으로 가까운 키프레임 ID를 빠르게 쿼리 (기존 함수 사용)
        std::vector<int> db_nearby_ids = db_manager_->loadKeyFramesByRadius(current_pose, surroundingKeyframeSearchRadius, active_keyframes_window_size_);
        surrounding_keyframe_indices.insert(db_nearby_ids.begin(), db_nearby_ids.end());

        // 2. 시간적으로 최신 키프레임 추가 (DB에 아직 없거나, 연속성 보장을 위해)
        int numPoses = cloudKeyPoses6D->size();
        for (int i = numPoses - 1; i >= 0 && i >= numPoses - 15; --i) // 최근 15개 프레임 포함
        {
            surrounding_keyframe_indices.insert(static_cast<int>(cloudKeyPoses6D->points[i].intensity));
        }
        
        RCLCPP_DEBUG(this->get_logger(), "로컬 맵 생성: DB에서 %zu개, 메모리에서 최신 프레임 포함하여 총 %zu개의 주변 키프레임 선택",
                     db_nearby_ids.size(), surrounding_keyframe_indices.size());

        // 3. 선택된 키프레임들로 로컬 맵(Submap) 구성
        for (int key_id : surrounding_keyframe_indices)
        {
            PointTypePose pose_6d;
            bool pose_found = false;

            // 먼저 메모리에 있는 포즈에서 찾아봅니다 (가장 빠름).
            for(const auto& p : cloudKeyPoses6D->points) {
                if (static_cast<int>(p.intensity) == key_id) {
                    pose_6d = p;
                    pose_found = true;
                    break;
                }
            }

            // 메모리에서 포즈를 찾지 못했다면 DB에 쿼리해야 합니다.
            if (!pose_found) {
                 // TODO: DBManager에 ID로 포즈를 직접 조회하는 함수 'loadPose(int key_id, PointTypePose& pose_out)'를 구현하는 것이 좋습니다.
                 // 임시로 DB를 직접 쿼리합니다.
                std::string sql = "SELECT x, y, z, roll, pitch, yaw FROM keyframes WHERE id = ?;";
                sqlite3_stmt* stmt;
                if (sqlite3_prepare_v2(db_manager_->getDB(), sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
                    sqlite3_bind_int(stmt, 1, key_id);
                    if (sqlite3_step(stmt) == SQLITE_ROW) {
                        pose_6d.x = sqlite3_column_double(stmt, 0);
                        pose_6d.y = sqlite3_column_double(stmt, 1);
                        pose_6d.z = sqlite3_column_double(stmt, 2);
                        pose_6d.roll = sqlite3_column_double(stmt, 3);
                        pose_6d.pitch = sqlite3_column_double(stmt, 4);
                        pose_6d.yaw = sqlite3_column_double(stmt, 5);
                        pose_found = true;
                    }
                    sqlite3_finalize(stmt);
                }
            }
            
            if (!pose_found) {
                RCLCPP_WARN(this->get_logger(), "Keyframe ID %d의 포즈 정보를 찾을 수 없습니다. 로컬 맵에서 제외됩니다.", key_id);
                continue; // 포즈를 못찾으면 이 키프레임은 스킵
            }
            
            // DBManager의 loadCloud 함수를 사용하여 클라우드 로드
            pcl::PointCloud<PointType>::Ptr cloud_ptr = db_manager_->loadCloud(key_id);

            if (cloud_ptr && !cloud_ptr->empty())
            {
                // 로드한 클라우드를 해당 포즈로 변환하여 로컬 맵에 추가
                *laserCloudSurfFromMap += *transformPointCloud(cloud_ptr, &pose_6d);
            }
        }
    }
    else // DB 모드가 비활성화된 경우 (기존 방식)
    {
        // =========================================================================
        // === 기존 메모리 기반 주변 키프레임 추출 (키프레임이 많아지면 느려짐) ===
        // =========================================================================
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // 병목 지점: cloudKeyPoses3D가 커질수록 이 검색 연산은 매우 느려집니다.
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (size_t i = 0; i < pointSearchInd.size(); ++i)
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

        // 로봇이 한 위치에서 회전하는 경우를 대비해 최신 키프레임 몇 개를 추가
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        // 선택된 키프레임들의 포인트 클라우드를 변환하여 로컬 맵 생성
        for (size_t i = 0; i < surroundingKeyPosesDS->size(); ++i)
        {
            int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
            
            // 메모리 내 캐시 확인
            if (laserCloudMapContainer.count(thisKeyInd)) 
            {
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            } 
            else 
            {
                // 메모리에 있는 surfCloudKeyFrames에서 클라우드를 가져와 변환
                if (thisKeyInd >= 0 && static_cast<size_t>(thisKeyInd) < surfCloudKeyFrames.size()) {
                    pcl::PointCloud<PointType> laserCloudCornerTemp; // 구조 유지를 위한 빈 클라우드
                    pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                    *laserCloudSurfFromMap += laserCloudSurfTemp;
                    laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
                }
            }
        }
    }

    // 최종적으로 구성된 로컬 맵을 다운샘플링하여 최적화에 사용
    downSizeFilterLocalMapSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterLocalMapSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

    // 메모리 관리를 위해 맵 캐시 크기 조절 (메모리 모드에서 주로 유효)
    if (laserCloudMapContainer.size() > 1000)
        laserCloudMapContainer.clear();
}


void mapOptimization::downsampleCurrentScan()
{
    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
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