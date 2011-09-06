#include <ros/ros.h>
#include <stdio.h>
#include "Configure.h"
#include "mst_common/VisionFilter.h"
#include "mst_common/ImageFilter.h"
#include "mst_common/Filter.h"

mst_common::VisionFilter    working_filter;

void printMenu()
{
    printf("\n1) Create new filter\n2) Delete filter\nT) Transmit\nQ) Quit\nChoice:\t");
}

//~ void printFilter(unsigned char* filter)
void printFilter(boost::array<unsigned char, 256ul> filter)
{
    unsigned char min = 255, max = 0;

    for( int i = 0; i < 256; i++ )
    {
        if( filter[i] == 255 )
        {
            if( i > max )
                max = i;
            if( i < min )
                min = i;
        }
    }
    printf("%d-%d",min,max);
}

void printStatus()
{
    printf("Current filters: %d\tHue\tChroma\tWhite\n", (int)working_filter.color.size());

    if( (int)working_filter.color.size() != 0 )
    {
        for( int i = 0; i < (int)working_filter.color.size(); i++ )
        {
            printf("\t\t %d - \t", i);
            for( int j = 0; j < 3; j++ )
            {
                printFilter( working_filter.color[i].filter[j].gain );
                printf("\t");
            }
            printf("\n");
        }
    }
}

int main(int argc, char** argv)
{
    printf("\n\nquick_filter!\n");

    ros::init(argc, argv, "quick_filter", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    ros::Publisher filter_pub = n.advertise<mst_common::VisionFilter>( FILTER_TOPIC, 1 );

    mst_common::ImageFilter     image_filter;

    char  c = '0';
    unsigned int val_0;
    unsigned int val_1;

    while( c != 'q' )
    {
        printStatus();
        printMenu();
        std::cin >> c;

        if( c == 't' || c == 'T' )
        {
            filter_pub.publish( working_filter );
        }
        else
        if( c == 'q' || c == 'Q' )
        {
            c = 'q';
        }
        else
        if( c == '1' )
        {
            for( int j = 0; j < 3; j++ )
                for( int k = 0; k < 255; k++ )
                    image_filter.filter[j].gain[k] = 0;

            printf( "Enter min value for hue:\t" );
            std::cin >> val_0;
            printf( "Enter max value for hue:\t" );
            std::cin >> val_1;

            for( int i = val_0; i <= val_1; i++ )
            {
                image_filter.filter[0].gain[i] = 255;
            }

            printf( "Enter min value for chroma:\t" );
            std::cin >> val_0;
            printf( "Enter max value for chroma:\t" );
            std::cin >> val_1;

            for( int i = val_0; i <= val_1; i++ )
            {
                image_filter.filter[1].gain[i] = 255;
            }

            printf( "Enter min value for white:\t" );
            std::cin >> val_0;
            printf( "Enter max value for white:\t" );
            std::cin >> val_1;

            for( int i = val_0; i <= val_1; i++ )
            {
                image_filter.filter[2].gain[i] = 255;
            }

            working_filter.color.push_back(image_filter);
            printf("\n");
        }
        else
        if( c == '2' )
        {
            printf( "Delete which filter?:\t" );
            std::cin >> val_0;
            working_filter.color.erase(working_filter.color.begin() + val_0);
            printf("\n");
        }
    }
    return 0;
}
