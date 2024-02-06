#include <opencv2/opencv.hpp>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace cv;
using namespace std;

int main(){
    VideoCapture cap("/workspaces/isaac_ros-dev/isaac_ros-dev/sample_video/AprilTag_test_video.mp4");
    
    
    if (!cap.isOpened()) {
		cerr << "Camera open failed!" << endl;
		return 0;
	}

    Mat frame, inversed;

    while(true){
        cap >> frame;
        if (frame.empty())
			continue;
        // inversed = ~frame;
        imshow("Frame", frame);

        if(waitKey(10) == 27){
            break;
        }
    }

    destroyAllWindows();

    cout << "1" << endl;
    return 0;
}

