#ifndef SNIPER_CAM_PUBLISHER_H
#define SNIPER_CAM_PUBLISHER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


namespace sniper_cam
{
class Sniper_cam

{

protected:



    // private parameters
    struct params_s {

        int camera_number;
        int frame_rate;


    };



public:

    Sniper_cam();
    ~Sniper_cam();

    // public function to call in main loop. This function will do most of the work
    void publish_image();

private:

    // private node handler
    ros::NodeHandle nh;

    // private image transport handle
    image_transport::ImageTransport it;

    // private image transport publsiher to advertise
    image_transport::Publisher pub;

    // private variables
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    struct params_s params;
    uint16_t counter;



};
}










#endif //SNIPER_CAM_PUBLISHER_H
