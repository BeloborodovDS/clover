#include <cmath>
#include <unistd.h>

#include "ros/ros.h"
#include "ros/package.h"
#include "human_detection/DetectionBox.h"
#include "human_detection/DetectionList.h"

#include "opencv2/core/core.hpp"

#include "ncs_wrapper/vino_wrapper.hpp"
#include "../submodules/sort-cpp/sort-c++/SORTtracker.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#define TRACKING_MAX_AGE     1
#define TRACKING_MIN_HITS    5
#define TRACKING_MIN_IOU     0.05
#define TRACKING_NUM_COLORS  10

using namespace cv;
using namespace human_detection;

/* function to parse SSD detector output
 * @param predictions: output buffer of SSD net 
 * @param numPred: maximum number of SSD predictions (from net config)
 * @param thresh: detection threshold
 * @param probs, boxes: resulting confidences and bounding boxes
 */
void getDetectionBoxes(const float* predictions, int numPred, float thresh,
                         vector<float>& probs, vector<Rect_<float>>& boxes)
{
    float score = 0, cls = 0, id = 0;
    
    //predictions holds numPred*7 values
    //data format: image_id, detection_class, detection_confidence, box_normed_x, box_normed_y, box_normed_w, box_normed_h
    for (int i = 0; i < numPred; i++) {
        score = predictions[i*7+2];
        cls = predictions[i*7+1];
        id = predictions[i*7];
        if (id>=0 && score>thresh && cls<=1) {
            probs.push_back(score);
            boxes.push_back(Rect_<float>(
                predictions[i*7+3], 
                predictions[i*7+4],
                (predictions[i*7+5] - predictions[i*7+3]), 
                (predictions[i*7+6] - predictions[i*7+4])
            ));
        }
    }
}

class Detection {
public:
    Detection(ros::NodeHandle &n);

    ~Detection();

    void imageCallback(const sensor_msgs::ImageConstPtr& inputMsg);
  
    ros::Publisher pub;
    NCSWrapper ncs;
    SORTtracker tracker;
    bool firstDetections;
    int H;
    int W;
    int count;
    int64 start;
    Mat dataMat;

    Scalar_<int> *colors;
    bool isDebug;
    image_transport::Publisher debugPub;
};

Detection::Detection(ros::NodeHandle &n) {
    pub = n.advertise<DetectionList>("detection/human_detection", 1);
    
    ncs = NCSWrapper(false);
    std::string modelPath = ros::package::getPath("human_detection") + "/data/human_vino";
    if (!ncs.load_file(modelPath)) {
        ROS_ERROR_STREAM("Detection node: failed to load graph file " + modelPath);
    }
    H = ncs.netInputHeight;
    W = ncs.netInputWidth;
    ROS_INFO_STREAM("Detection node: loaded graph file " + modelPath);

    tracker = SORTtracker(TRACKING_MAX_AGE, TRACKING_MIN_HITS, TRACKING_MIN_IOU);
    bool firstDetections = true;  //flag used to init tracker

    count = 0;
    start = getTickCount();
    dataMat = Mat(H, W, CV_8UC3);

    isDebug = false;
    ros::param::get("~debug", isDebug);
    if (isDebug) {
        ROS_INFO_STREAM("Debug mode");
    } else {
        ROS_INFO_STREAM("Regular mode");
    }

    if (isDebug) {
        RNG rng(0xFFFFFFFF);
        colors = new Scalar_<int>[TRACKING_NUM_COLORS];
        for (int i = 0; i < TRACKING_NUM_COLORS; i++) {
            rng.fill(colors[i], RNG::UNIFORM, 0, 256);
        }
        image_transport::ImageTransport it(n);
        debugPub = it.advertise("detection/debug_image", 1);
    }
}

Detection::~Detection() {
    if (isDebug) {
        delete[] colors;
    }
}

void Detection::imageCallback(const sensor_msgs::ImageConstPtr& inputMsg) {
    try {
        float* ncsOutput = NULL;
        DetectionList outputMsg;
        vector<Rect_<float>> detVector;
        vector<float> probVector;
        vector<TrackingBox> trackVector;

        Mat image = cv_bridge::toCvShare(inputMsg, "bgr8") -> image;
        resize(image, dataMat, Size(W, H), 0, 0, INTER_NEAREST);

        //load image into NCS
        if (!ncs.load_tensor_nowait(dataMat)) {
            ROS_ERROR("Detection node: an error occured in NCS (load_tensor_nowait)");
            return;
        }
        // get result from NCS (blocking)
        if(!ncs.get_result(ncsOutput)) {
            ROS_ERROR("Detection node: an error occured in NCS (get_result)");
            return;
        }
        // decode detections from NCS
        detVector.clear();
        probVector.clear();
        getDetectionBoxes(ncsOutput, ncs.maxNumDetectedFaces, 0.2, probVector, detVector);

        // if first detections ever: init tracker if possible
        if (detVector.size() > 0 && firstDetections) {
            tracker.init(detVector);
            firstDetections = false;
        }
        // if tracker initialized, track
        if (!firstDetections) {
            trackVector.clear();
            tracker.step(detVector, trackVector);
        }

        for (int i = 0; i < trackVector.size(); i++) {
            outputMsg.count = count;
            DetectionBox box;
            box.x = trackVector[i].box.x;
            box.y = trackVector[i].box.y;
            box.width = trackVector[i].box.width;
            box.height = trackVector[i].box.height;
            outputMsg.detections.push_back(box);
        }

        if (trackVector.size() > 0) {
            pub.publish(outputMsg);
        }

        if (isDebug) {
            for (int i = 0; i < trackVector.size(); i++) {
                Scalar_<int> intcol = colors[trackVector[i].id % TRACKING_NUM_COLORS];
                Scalar col = Scalar(intcol[0], intcol[1], intcol[2]);
                Rect_<float> box = trackVector[i].box;
                Rect_<float> scaledBox = Rect_<float>(
                    box.x * W, box.y * H, box.width * W, box.height * H);
                rectangle(dataMat, scaledBox, col, 3);
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dataMat).toImageMsg();
            debugPub.publish(msg);
        }

        if (count > 0 and count % 300 == 0) {
            double time = (getTickCount() - start) / getTickFrequency();
            ROS_INFO("Detection node: FPS   %f", count / time);
        }
        count++;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", inputMsg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "human_detection");
    ros::NodeHandle n;

    Detection detection = Detection(n);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe(
        "/main_camera/image_raw", 1, &Detection::imageCallback, &detection);
    ros::spin();

    return 0;
}
