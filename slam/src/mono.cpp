#include <cmath>
#include <unistd.h>

#include "ros/ros.h"
#include "ros/package.h"

#include "opencv2/core/core.hpp"

#include "../submodules/ORB_SLAM3/include/System.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

using namespace cv;

class Slam {
public:
    Slam(ros::NodeHandle &n);

    ~Slam();

    void imageCallback(const sensor_msgs::ImageConstPtr& inputMsg);
  
    ORB_SLAM3::System *system;
    int count;
    int64 start;

    bool isDebug;
    image_transport::Publisher debugPub;
};

Slam::~Slam() {
    delete system;
}

Slam::Slam(ros::NodeHandle &n) {
    const std::string vocPath = ros::package::getPath("orb_slam3") + "/submodules/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    const std::string settingsPath = ros::package::getPath("orb_slam3") + "/settings.txt";
    system = new ORB_SLAM3::System(vocPath, settingsPath, ORB_SLAM3::System::MONOCULAR, true);

    ROS_INFO_STREAM("SLAM node: loaded vocabulary " + vocPath);
    ROS_INFO_STREAM("SLAM node: using settings " + settingsPath);

    count = 0;
    start = getTickCount();

    isDebug = false;
    ros::param::get("~debug", isDebug);
    if (isDebug) {
        ROS_INFO_STREAM("Debug mode");
    } else {
        ROS_INFO_STREAM("Regular mode");
    }

    if (isDebug) {
        image_transport::ImageTransport it(n);
        debugPub = it.advertise("slam/debug_image", 1);
    }
}

void Slam::imageCallback(const sensor_msgs::ImageConstPtr& inputMsg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(inputMsg, "bgr8");
        system -> TrackMonocular(cv_ptr -> image, cv_ptr -> header.stamp.toSec());

        if (isDebug) {
            Mat debugImage = system -> DrawFrame();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debugImage).toImageMsg();
            debugPub.publish(msg);
        }

        if (count > 0 and count % 300 == 0) {
            double time = (getTickCount() - start) / getTickFrequency();
            ROS_INFO("SLAM node: FPS   %f", count / time);
        }
        count++;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", inputMsg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam_mono");
    ros::NodeHandle n;

    Slam slam = Slam(n);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe(
        "/main_camera/image_raw", 1, &Slam::imageCallback, &slam);
    ros::spin();

    return 0;
}
