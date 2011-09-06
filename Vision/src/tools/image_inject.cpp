//---------------------------------------------------------
// Auth: Chris Bessent <cmbq76>
// File: image_inject.cpp
// Date: 2/8/11
//
// Desc: Program that will load a local image file using 
//       OpenCV and publish to the specified topic
//       with the specified frequency (or 1.0Hz).  Used
//       mainly for testing image processing nodes.
//---------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

std::string img_path;
std::string topic;
double frequency = 1.0;

void parseArgs( std::vector<std::string>& args )
{
    for( int i = 0; i < args.size(); i++ )
    {
        if( args[i].find("image=")!=std::string::npos )
        {
            args[i].erase(0,6);
            img_path = args[i];
        }
        if( args[i].find("topic=")!=std::string::npos )
        {
            args[i].erase(0,6);
            topic = args[i];
        }
        if( args[i].find("hz=")!=std::string::npos )
        {
            args[i].erase(0,3);
            frequency = strtod(args[i].c_str(), NULL);
        }
    }
}

int main(int argc, char** argv )
{
    std::vector<std::string> args_out;
    ros::removeROSArgs( argc, argv, args_out );
    parseArgs( args_out );
    if( topic == "" || img_path == "" || frequency == 1.0 )
    {
        ROS_WARN("image_inject: arguments have not been remapped! Typical command-line usage:\n"
                 "\t$ ./image_inject image=<image file> topic=<image topic> hz=<frequency>" );
    }

    ros::init(argc, argv, "image_inject", ros::init_options::AnonymousName );
    ros::NodeHandle n;

    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>(topic, 1);

    sensor_msgs::CvBridge bridge_;
    IplImage* img;
    img = cvLoadImage( img_path.c_str() );

    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        img_pub.publish( bridge_.cvToImgMsg(img,"bgr8") );
        loop_rate.sleep();
    }
    return 0;
}
