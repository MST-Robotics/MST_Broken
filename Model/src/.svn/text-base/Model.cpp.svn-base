/*******************************************************************************
 * File: Model.cpp
 * Auth: Chris Bessent <cmbq76>
 *
 * Desc:
 ******************************************************************************/

/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include "sensor_msgs/Image.h"
#include "mst_common/Raytrace.h"

/***********************************************************
* Other includes
***********************************************************/
#include "Configure.h"

/***********************************************************
* Global variables
***********************************************************/
ros::Publisher      g_output_pub;

int***      g_rayDef    = NULL;
int*        g_numPix    = NULL;
int**       g_rangePts  = NULL;
int         S_RAD       = 1;   //TODO: Change to parameter
int         HIT_CT_TH   = 3;   //TODO: Change to parameter
int         NUM_RAYS    = 181; //TODO: Change to parameter

/***********************************************************
* Function prototypes
***********************************************************/
void initRaytrace( unsigned int, unsigned int );
void computeRaytrace( const sensor_msgs::Image::ConstPtr&, mst_common::Raytrace& );

/***********************************************************
* Message Callbacks
***********************************************************/
void filtImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    static unsigned int old_width  = 0;
    static unsigned int old_height = 0;

    if( (old_width  != msg->width) ||
        (old_height != msg->height)  )
    {
        initRaytrace( msg->width, msg->height );
        old_width = msg->width;
        old_height = msg->height;
    }

    mst_common::Raytrace output_msg;
    computeRaytrace( msg, output_msg );
    g_output_pub.publish( output_msg );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NODE_NAME");
    ros::NodeHandle n;

    ros::Subscriber filt_image_sub = n.subscribe( FILT_IMAGE_TOPIC, 5, filtImageCallback );
    g_output_pub = n.advertise<mst_common::Raytrace>(RAYTRACE_TOPIC, 5);

    ros::spin();
}

void initRaytrace( unsigned int width, unsigned int height )
{
    //------------------------------------------------------
    // Memory Management
    //------------------------------------------------------
    if( g_rayDef != NULL )
        delete[] g_rayDef;
    if( g_numPix != NULL )
        delete[] g_numPix;
    if( g_rangePts != NULL )
        delete[] g_rangePts;

    g_rayDef = new int**[NUM_RAYS];
    for( int i = 0; i < NUM_RAYS; i++ )
    {
        g_rayDef[i] = new int*[width+height];
    }
    for( int i = 0; i < NUM_RAYS; i++ )
    {
        for( unsigned int j = 0; j < width+height; j++ )
        {
            g_rayDef[i][j] = new int[2];
        }
    }

    g_numPix = new int[NUM_RAYS];

    g_rangePts = new int*[NUM_RAYS];
    for( int i = 0; i < NUM_RAYS; i++ )
    {
        g_rangePts[i] = new int[2];
    }

    //------------------------------------------------------
    // Fill arrays
    //------------------------------------------------------
    const unsigned int CAMERA_X = 320; //TODO: Change to parameter
    const unsigned int CAMERA_Y = 30;  //TODO: Change to parameter

    double degInterval=180.0/(NUM_RAYS-1);
    double radInterval=degInterval/57.295779513;

    for(int rayId=0; rayId<NUM_RAYS; rayId++)
    {
        double theta=rayId*radInterval;

        int er=(int)(CAMERA_Y+(600.0*sin(theta)));
        int ec=(int)(CAMERA_X+(600.0*cos(theta)));

        // load first ray pixel

        g_numPix[rayId]=1;
        g_rayDef[rayId][1][0]=CAMERA_Y;
        g_rayDef[rayId][1][1]=CAMERA_X;

        double dr=(er-CAMERA_Y)/600.0; // where does 600 come from?
        double dc=(ec-CAMERA_X)/600.0;

        for(double dist=0.0; dist<600.0; dist+=0.33)
        {
            int r=(int)(CAMERA_Y+(dr*dist));
            int c=(int)(CAMERA_X+(dc*dist));

            if(r>=0 && r <=(height-1) && c >=0 && c <=(width-1)) // keep within bounds of image
            {
                if((r != g_rayDef[rayId][g_numPix[rayId]][0]) || (c != g_rayDef[rayId][g_numPix[rayId]][1]))
                {
                    g_numPix[rayId]++;
                    g_rayDef[rayId][g_numPix[rayId]][0]=r;
                    g_rayDef[rayId][g_numPix[rayId]][1]=c;
                }
            }
        }
    }
}

void computeRaytrace( const sensor_msgs::Image::ConstPtr& msg,
                      mst_common::Raytrace& output_msg )
{
    output_msg.ranges.clear();
    output_msg.num_ranges = NUM_RAYS;

    const int CAMERA_WIDTH  = msg->width;
    const int CAMERA_HEIGHT = msg->height;
    const int CAMERA_X      = 320;
    const int CAMERA_Y      = 30;

    float pix_val;
    int pixId;
    int r,c;
    int lor,hir,loc,hic;
    int hitCount;
    bool hit;
    double dist;

    for(int rayId=0; rayId<NUM_RAYS; rayId++)
    {
        pixId=0;
        hit=false;

        while(!hit && pixId<g_numPix[rayId])
        {
            pixId++;
            r=g_rayDef[rayId][pixId][0];
            c=g_rayDef[rayId][pixId][1];

            lor=r-S_RAD;
            hir=r+S_RAD;
            loc=c-S_RAD;
            hic=c+S_RAD;

            if(lor<0)
            {
                lor=0;
            }
            if(hir>(CAMERA_HEIGHT-1))
            {
                hir=(CAMERA_HEIGHT-1);
            }
            if(loc<0)
            {
                loc=0;
            }
            if(hic>(CAMERA_WIDTH-1))
            {
                hic=(CAMERA_WIDTH-1);
            }

            hitCount=0;
            int k=0;
            for(int sr=lor; sr<=hir; sr++)
            {
                for(int sc=loc; sc<=hic; sc++)
                {
                    k++;
                    //~ unsigned char* ptr = (unsigned char*)(modelimg->imageData+(CAMERA_HEIGHT-1-sr)*modelimg->widthStep+3*sc);
                    pix_val = (float)msg->data[(CAMERA_HEIGHT-1-sr)*CAMERA_WIDTH+sc];

                    if(pix_val>0)
                    {
                        hitCount++;
                    }
                }
            }

            if(hitCount >= HIT_CT_TH)
            {
                hit=true;
                dist=sqrt(((r-CAMERA_Y)*(r-CAMERA_Y)) + ((c-CAMERA_X)*(c-CAMERA_X)))/100.0; // is this scalar correct?
                output_msg.ranges.push_back(dist);

                g_rangePts[rayId][0]=r;
                g_rangePts[rayId][1]=c;
            }
        } // while loop

        if (!hit && pixId==g_numPix[rayId])
        {
            dist=sqrt(((r-CAMERA_Y)*(r-CAMERA_Y)) + ((c-CAMERA_X)*(c-CAMERA_X)))/100.0;
            output_msg.ranges.push_back(dist);

            g_rangePts[rayId][0]=r;
            g_rangePts[rayId][1]=c;
        }
    } // rayId loop

}

