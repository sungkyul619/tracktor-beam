#include "ArucoTracker.hpp"
#include <sstream>

ArucoTrackerNode::ArucoTrackerNode()
	: Node("aruco_tracker_node")
{
	loadParameters();

	// TODO: params to adjust detector params
	// See: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html


	auto qos = rclcpp::QoS(1).best_effort();
	// Subscribers
	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
			     _image_sub_topic, qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
				    _camera_info_sub_topic, qos, std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));
	

	auto detectorParams = cv::aruco::DetectorParameters();

	// See: https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html
	auto dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);


	_detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detectorParams);


	// Publishers
	_image_pub = create_publisher<sensor_msgs::msg::Image>("/image_proc", qos);
	_target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", qos);
}

void ArucoTrackerNode::loadParameters()
{
	    // 파라미터 선언 및 기본값 설정
    // 기본 토픽명을 여기에 명시하여 사용자가 변경할 수 있도록 합니다.
    declare_parameter<std::string>("image_sub_topic", "/image_raw");
    declare_parameter<std::string>("camera_info_sub_topic", "/camera_info");
    declare_parameter<int>("aruco_id", 0);
    declare_parameter<int>("dictionary", cv::aruco::DICT_4X4_250); // cv::aruco::PredefinedDictionaryType의 enum 값을 사용
    declare_parameter<double>("marker_size", 0.05); // 미터 단위 (예: 5cm)

    // 선언된 파라미터 값 가져오기
    get_parameter("image_sub_topic", _image_sub_topic);
    get_parameter("camera_info_sub_topic", _camera_info_sub_topic);
    get_parameter("aruco_id", _param_aruco_id);
    get_parameter("marker_size", _param_marker_size);
    get_parameter("dictionary", _param_dictionary); // int로 가져옴
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // ROS 이미지 메시지를 OpenCV 이미지(BGR8 형식)로 변환합니다.
        // 이 변환을 통해 OpenCV 함수를 사용하여 이미지 처리를 할 수 있어요.
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // --- ArUco 마커 감지 ---
        // 이미지에서 ArUco 마커를 찾습니다.
        // 'ids': 감지된 마커들의 고유 ID가 저장됩니다.
        // 'corners': 각 마커의 4개 코너(모서리)의 이미지 좌표가 저장됩니다.
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        _detector->detectMarkers(cv_ptr->image, corners, ids);
        
        // 감지된 마커들을 이미지에 그립니다.
        // 각 마커의 테두리와 ID를 시각적으로 표시해줘요.
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

  		// 카메라 내부 파라미터가 있는지 확인한다. 이정보가없으면 정확한 자세추정이 불가능.
		// --- 카메라 보정 정보 확인 및 자세 추정 ---
        // 카메라 내부 파라미터(_camera_matrix)와 왜곡 계수(_dist_coeffs)가 있어야
        // 정확한 3D 자세(위치, 방향) 추정이 가능해요. 
        if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {

            // 마커 코너들의 왜곡을 보정합니다.
            // 카메라 렌즈의 왜곡 때문에 이미지상의 점들이 실제와 다르게 보일 수 있는데,
            // 이를 보정하여 더 정확한 3D 계산을 가능하게 해요. 
            std::vector<std::vector<cv::Point2f>> undistortedCorners;
            for (const auto& corner : corners) {
                std::vector<cv::Point2f> undistortedCorner;
                // 왜곡 보정 함수: `cv::noArray()`는 R, P 매트릭스가 없음을 의미해요.
                cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
                undistortedCorners.push_back(undistortedCorner);
            }

            // 감지된 마커들 중 지정된 ID(_param_aruco_id)를 가진 마커를 찾습니다.
            for (size_t i = 0; i < ids.size(); i++) {
                if (ids[i] != _param_aruco_id) {
                    continue; // 원하는 ID의 마커가 아니면 건너뜁니다.
                }

                // --- 3D 객체 포인트 정의 ---
                // ArUco 마커의 실제 3D 공간에서의 코너 좌표를 정의합니다.
                // 이 좌표는 마커의 중심을 (0,0,0)으로 가정하고, Z축은 0에 있다고 가정해요.
                // _param_marker_size는 마커의 실제 크기를 나타냅니다.
                float half_size = _param_marker_size / 2.0f;
                std::vector<cv::Point3f> objectPoints = {
                    cv::Point3f(-half_size,  half_size, 0),  // 상단 왼쪽
                    cv::Point3f(half_size,  half_size, 0),   // 상단 오른쪽
                    cv::Point3f(half_size, -half_size, 0),   // 하단 오른쪽
                    cv::Point3f(-half_size, -half_size, 0)   // 하단 왼쪽
                };

                // --- PnP(Perspective-n-Point) 알고리즘으로 자세 추정 ---
                // 3D 공간의 점(objectPoints)과 그것에 대응하는 2D 이미지 점(undistortedCorners[i])을 사용하여
                // 카메라 좌표계에서 마커의 3D 자세(회전 및 이동)를 추정합니다.
                // 'rvec': 회전 벡터 (Rodrigues 형식)
                // 'tvec': 이동 벡터 (병진 벡터)
                cv::Vec3d rvec, tvec;
                cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec);
                
				// solvePnP는 마커의 3D 위치와 방향을 계산하고, drawFrameAxes는 그 계산된 위치와 방향을 이미지 위에 시각적으로 보여주는 역할을 합니다.


                // 추정된 마커의 자세를 이미지에 3D 축으로 그려서 시각화합니다.
                cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, _param_marker_size);

                // --- 회전 벡터를 쿼터니언으로 변환 ---
                // ROS 메시지에서는 주로 쿼터니언을 사용하여 회전 정보를 표현해요.
                cv::Mat rot_mat;
                cv::Rodrigues(rvec, rot_mat); // 회전 벡터를 회전 행렬로 변환
                cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize(); // 회전 행렬을 쿼터니언으로 변환 후 정규화

                // --- 마커의 자세(위치 및 방향)를 ROS 메시지로 발행 ---
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = msg->header.stamp; // 메시지 시간은 원본 이미지 시간과 동일하게
                pose_msg.header.frame_id = "camera_frame"; // 이 자세가 어떤 프레임 기준인지 명시
                
                // 이동 벡터 (위치) 설정
                pose_msg.pose.position.x = tvec[0];
                pose_msg.pose.position.y = tvec[1];
                pose_msg.pose.position.z = tvec[2];
                
                // 쿼터니언 (방향) 설정
                pose_msg.pose.orientation.x = quat.x;
                pose_msg.pose.orientation.y = quat.y;
                pose_msg.pose.orientation.z = quat.z;
                pose_msg.pose.orientation.w = quat.w;

                _target_pose_pub->publish(pose_msg); // 발행!

                // 이미지에 추가적인 주석(텍스트 등)을 달아요.
                annotate_image(cv_ptr, tvec);

                // NOTE: 여기에서 `break` 문이 있기 때문에,
                // 여러 ArUco 마커가 감지되더라도 설정된 `_param_aruco_id`를 가진 마커 중
                // 첫 번째로 발견된 마커의 자세만 처리하고 루프를 종료합니다.
                break;
            }

        } else {
            // 카메라 보정 정보가 없는 경우 오류 메시지를 출력합니다.
            RCLCPP_ERROR(get_logger(), "Missing camera calibration");
        }

        // --- 처리된 이미지 발행 ---
        // 항상 처리된 이미지(마커가 그려지고 축이 그려진)를 다시 ROS 메시지로 발행합니다.
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = cv_ptr->image;
        _image_pub->publish(*out_msg.toImageMsg().get());

    } catch (const cv_bridge::Exception& e) {
        // cv_bridge 변환 중 예외(오류)가 발생하면 오류 메시지를 출력합니다.
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	// Always update the camera matrix and distortion coefficients from the new message
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // Use clone to ensure a deep copy
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx
		    _camera_matrix.at<double>(1, 1), // fy
		    _camera_matrix.at<double>(0, 2), // cx
		    _camera_matrix.at<double>(1, 2)  // cy
		   );

	// Check if focal length is zero after update
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camera_info topic.");

		RCLCPP_INFO(get_logger(), "Unsubscribing from camera info topic");
		_camera_info_sub.reset();
	}
}

void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target)
{
	// Annotate the image with the target position and marker size
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "X: "  << target[0] << " Y: " << target[1]  << " Z: " << target[2];
	std::string text_xyz = stream.str();

	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;
	int baseline = 0;
	cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	cv::putText(image->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoTrackerNode>());
	rclcpp::shutdown();
	return 0;
}