#include "mapOptimization.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <thread>

int main(int argc, char** argv)
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);
    
    try {
        // 노드 및 실행기 설정
        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        rclcpp::executors::SingleThreadedExecutor exec;
        auto MO = std::make_shared<mapOptimization>(options);
        exec.add_node(MO);
        
        // 스레드 관리를 위한 벡터
        std::vector<std::thread> threads;
        
        // Loop Closure 스레드 시작
        if (MO->loop_closure_) {
            RCLCPP_INFO(MO->get_logger(), "Starting Loop Closure thread...");
            threads.emplace_back(&LoopClosure::loopClosureThread, MO->loop_closure_.get());
        } else {
            RCLCPP_WARN(MO->get_logger(), "Loop Closure module not initialized");
        }
        
        // 맵 시각화 스레드 시작
        threads.emplace_back(&mapOptimization::visualizeGlobalMapThread, MO);
        
        // 코스트맵 스레드 시작 (필요시)
        if (MO->costmap_generator_) {
            MO->costmap_generator_->startCostmapThread();
        }
        
        // 메인 루프 실행
        exec.spin();
        
        // 종료 시 모든 스레드 정리
        for (auto& thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("liorf_mapOptimization"), 
                    "Exception in main thread: %s", e.what());
    }
    
    // ROS 2 종료
    rclcpp::shutdown();
    
    return 0;
} 