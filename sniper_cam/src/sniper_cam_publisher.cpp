#include <opencv2/highgui/highgui.hpp>
#include <sstream>

#include "sniper_cam_publisher.h"


// This node is meant to access the images from a local camera
// using openCV and publish them as a ros message. The image transport
// functionality allows the node to publish compressed images to minimize
// the bandwidth of the transmitted images.



namespace sniper_cam
{
Sniper_cam::Sniper_cam():
    it(image_transport::ImageTransport(nh))
{

    // variables to use in functions
    // these are all private and cannot not be accessed
    // outside the class.

    // publsiher
    pub = it.advertise("sniper_cam/image", 1);

    // private node handler
    nh;

    // variable for the frame matrix
    frame;

    // variable for msg
    msg;

    counter = 0;

    // parameters
    nh.param<int>("CAMERA_NUMBER", params.camera_number, 0);
    nh.param<int>("FRAME_RATE", params.frame_rate, 30);

}

// destructor function. Nesessary for all c++ classes
Sniper_cam::~Sniper_cam()
{

}


// void function that does all that needs to be done to publish
// the images
void Sniper_cam::publish_image(){


    // get access to local camera using open cv. We might want to change the
    // input to this function to a ros parameter, but since we only plan on
    // using this node for the sniper camera we can assume the sniper camera will be
    // camera 0.
    cv::VideoCapture cap(params.camera_number);

    // check if the camera has been accessed
    if(!cap.isOpened()) return;

    // set the loop rate to 15. The input to this should also be a ros parameter
    // For the competition, this should be set to 1Hz.
    ros::Rate loop_rate(params.frame_rate);


    // while the ros handler is running ok.
    while(nh.ok()){


        cap >> frame;
        // check if grabbed frame is actually full with some content
        if(!frame.empty()){

            //resize the image from 1080p 576p (~half size)
            cv::resize(frame,frame,cv::Size(1024,576)); //16:9 aspect ratio

            // convert msg to a ros msg
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

            // publish messages
            if (counter == 0)
            {
                pub.publish(msg);
                counter = params.frame_rate;

            }

            counter--;


            // wait for kill
            //cv::waitKey(1);
        }
        // spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }


}
}

int main(int argc, char** argv)
{


    ROS_INFO_STREAM("Sniper cam publisher");
    ros::init(argc, argv, "sniper_cam_publisher");
    sniper_cam::Sniper_cam sc;
    sc.publish_image();

}
