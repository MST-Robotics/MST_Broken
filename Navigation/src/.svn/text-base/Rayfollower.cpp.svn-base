/*******************************************************************************
 * File: Rayfollower.cpp
 * Auth: Chris Bessent <cmbq76>
 *
 * Desc: Conversion of AI made by James (one of them, at least).
 *       Waits until new Raytrace msg is sent and computes
 *       linear and angular velocity to navigate forwards.
 ******************************************************************************/

/***********************************************************
* ROS specific includes
***********************************************************/
#include <ros/ros.h>
#include <ros/callback_queue.h>

/***********************************************************
* Message includes
***********************************************************/
#include "mst_common/Raytrace.h"
#include "mst_common/Velocity.h"

/***********************************************************
* Other includes
***********************************************************/
#include "Rayfollower.h"
#include "Math.h"
#include <stdlib.h>

/***********************************************************
* Global variables
***********************************************************/
ros::Publisher          motion_pub;

mst_common::Velocity    velocity_msg;

/***********************************************************
* Function prototypes
***********************************************************/
void rayfollow( const std::vector<double>, const int );
void smear( double*, const int );
int  find_path( double*, const int );

/***********************************************************
* Message Callbacks
***********************************************************/
void modelCallback( const mst_common::Raytrace::ConstPtr& msg )
{
    rayfollow( msg->ranges, msg->num_ranges );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rayfollower");
    ros::NodeHandle n;

    ros::Subscriber model_sub = n.subscribe( RAYTRACE_TOPIC, 1000, modelCallback );
    motion_pub = n.advertise<mst_common::Velocity>( MOTION_TOPIC, 1000 );

    ros::spin();
}

void rayfollow( const std::vector<double> ranges, const int n )
{
    double* mod_ranges = new double[n];

    for( int i = 0; i < n; ++i )
    {
        mod_ranges[i] = ranges[i];
    }

    smear( mod_ranges, n );
    int chosen_ray = find_path( mod_ranges, n );

    double angular;
    double linear;

    angular = 10 * ( 1 / 10.0 * mod_ranges[n/2] ) * ( chosen_ray - n/2 );
    angular = limit( angular, -TURN_LIMIT, TURN_LIMIT );
    linear = ( TOPSPEED - ((abs(angular) / 180.0 ) * (0.75 )));
    linear = limit( linear, 0.0, 1.0 );
    angular = -deg_to_rad( angular );

    velocity_msg.linear = linear;
    velocity_msg.angular = angular;

    motion_pub.publish( velocity_msg );

    delete[] mod_ranges;
}

void smear( double* ranges, const int n )
{
    double delta_theta;
    double last_theta;
    double last_dist;
    double calc_dist;

    last_theta = 1;
    last_dist = 20;

    for( int i = 1; i < (n-1); ++i )
    {
        delta_theta = i - last_theta;
        delta_theta = deg_to_rad( delta_theta );
        calc_dist = last_dist / cos( delta_theta );

        if( calc_dist <= ranges[i] && SMEAR_MULT >= ( last_dist * tan( delta_theta ) ) )
        {
            ranges[i] = calc_dist;
        }
        else
        {
            last_theta = i;
            last_dist = ranges[i];
        }
    }

    last_theta = n-1;
    last_dist = 20;

    for( int i = (n-1); i >= 1; --i )
    {
        delta_theta = -(i - last_theta);
        delta_theta = deg_to_rad( delta_theta );
        calc_dist = last_dist / cos( delta_theta );

        if( calc_dist <= ranges[i] && SMEAR_MULT >= ( last_dist * tan( delta_theta ) ) )
        {
            ranges[i] = calc_dist;
        }
        else
        {
            last_theta = i;
            last_dist = ranges[i];
        }
    }
} /* smear() */

int find_path( double* ranges, const int n )
{
    double ray          = 0;
    double max_ray      = 0;
    int    max_theta    = 0;
    double multiplier   = 0;
    double last         = 0;
    double diff         = 0;

    int left_bndry = (int)(LEFT_BNDRY_LIMIT * n);
    int rght_bndry = (int)(RGHT_BNDRY_LIMIT * n);

    for( int i = left_bndry; i <= rght_bndry; ++i )
    {
        diff = last - ranges[i];
        last = ranges[i];
        diff = abs( diff );
        limit( diff, 0, 10 );

        multiplier = abs( i - n/2 );
        multiplier = deg_to_rad( multiplier / 2 );
        multiplier = cos( multiplier );

        ray = ( multiplier * ( ranges[i] + diff ) );

        if( ray > max_ray )
        {
            max_ray = ray;
            max_theta = i;
        }
    }
    return max_theta;
}
