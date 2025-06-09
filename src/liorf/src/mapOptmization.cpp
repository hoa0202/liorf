// ... existing code ...
            // Step 1: 초기 추정치 업데이트와 주변 키프레임 추출을 병렬로 수행
            initialGuessFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "초기 추정치 업데이트 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->updateInitialGuess();
                RCLCPP_INFO(this->get_logger(), "초기 추정치 업데이트 스레드 완료");
            });
            
            // Step 2: 주변 키프레임 추출은 초기 추정치가 필요할 수 있으므로 순차적으로 처리
            initialGuessFuture.wait();
            surroundingKeyFramesFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "주변 키프레임 추출 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->extractSurroundingKeyFrames();
                RCLCPP_INFO(this->get_logger(), "주변 키프레임 추출 스레드 완료");
            });
            
            // Step 3: 현재 스캔 다운샘플링은 독립적으로 수행 가능
            downsampleFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "다운샘플링 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->downsampleCurrentScan();
                RCLCPP_INFO(this->get_logger(), "다운샘플링 스레드 완료");
            });
            
            // Step 5: 키프레임 저장 및 포즈 수정은 병렬로 수행 가능
            std::future<void> saveKeyFramesFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "키프레임 저장 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->saveKeyFramesAndFactor();
                RCLCPP_INFO(this->get_logger(), "키프레임 저장 스레드 완료");
            });
            
            saveKeyFramesFuture.wait();
            
            // Step 6: 포즈 수정 및 경로 업데이트
            correctPoses();
            
            // Step 7: 오래된 프레임 정리와 오도메트리/프레임 발행은 병렬로 수행 가능
            std::future<void> clearOldFramesFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "오래된 프레임 정리 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->clearOldFrames();
                RCLCPP_INFO(this->get_logger(), "오래된 프레임 정리 스레드 완료");
            });
            
            std::future<void> publishOdometryFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "오도메트리 발행 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->publishOdometry();
                RCLCPP_INFO(this->get_logger(), "오도메트리 발행 스레드 완료");
            });
            
            std::future<void> publishFramesFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "프레임 발행 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->publishFrames();
                RCLCPP_INFO(this->get_logger(), "프레임 발행 스레드 완료");
            });

            // 코스트맵 데이터 업데이트 - 비동기적으로 실행
            auto costmapFuture = std::async(std::launch::async, [this, keyPoses6DVector]() {
                RCLCPP_INFO(this->get_logger(), "코스트맵 처리 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                this->costmap_generator_->processClouds(this->cloudKeyPoses3D, this->surfCloudKeyFrames, keyPoses6DVector);
                RCLCPP_INFO(this->get_logger(), "코스트맵 처리 스레드 완료");
            });
            // 결과를 사용하지 않지만, 나중에 필요할 경우 사용할 수 있도록 유지합니다.
            (void)costmapFuture;

            auto loopClosureFuture = std::async(std::launch::async, [this]() {
                RCLCPP_INFO(this->get_logger(), "루프 클로저 스레드 시작 (Thread ID: %lu)", (unsigned long)std::this_thread::get_id());
                if (use_database_mode_ && db_manager_ && db_manager_->isInitialized()) {
                    // DB 모드 - 포즈 정보만 전달
                    loop_closure_->setInputDataWithDB(cloudKeyPoses3D, cloudKeyPoses6D, timeLaserInfoCur);
                    RCLCPP_DEBUG(this->get_logger(), "Publishing map: Loop closure using DB mode");
                } else if (!surfCloudKeyFrames.empty()) {
                    // 기존 메모리 모드
                    loop_closure_->setInputData(cloudKeyPoses3D, cloudKeyPoses6D, surfCloudKeyFrames, timeLaserInfoCur);
                    RCLCPP_DEBUG(this->get_logger(), "Publishing map: Loop closure using memory-only mode");
                }
                RCLCPP_INFO(this->get_logger(), "루프 클로저 스레드 완료");
            });
// ... existing code ... 