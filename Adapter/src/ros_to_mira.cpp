/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file MiraAdapter.cpp
 *    An adapter for messages between ROS and MIRA.
 *
 * @author Dirk Steindorf
 * @date   2015/07/14
 */

#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>

#include <fw/Framework.h>
#include <fw/ChannelReadWrite.h>
#include <platform/Types.h>
#include <maps/GridMap.h>
#include <image/Img.h>

using std::cout;
using std::endl;

using namespace mira::maps;

//-----------------------------------------------------------------------------
// constants
const int8 ROS_UNKNOWN              = -1;
const int8 ROS_FREE_THRESHOLD       = 20;
const int8 ROS_OCCUPIED_THRESHOLD   = 65; 

const unsigned int MIRA_FREE        = 0;
const unsigned int MIRA_OCCUPIED    = 254;
const unsigned int MIRA_UNKNOWN     = 130; // I guessed this value


//-----------------------------------------------------------------------------
// MIRA-specific stuff
mira::Authority authority; 

// channel for publishing a ROS OccupancyGrid to MIRA
mira::Channel<GridMap<uint8>> ros_grid_to_mira;


//------------------------------------------------------------------------------
// callback for publishing a new map to MIRA

// ROS: occupancy probablities in [0,100] and unknown is -1
// MIRA: occupancy values in [0,254] and unknown (uninitialized) is 255
void onNewMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    unsigned int width = static_cast<unsigned int>(msg->info.width);
    unsigned int height = static_cast<unsigned int>(msg->info.height);

    // create Image for GridMap
    // http://www.mira-project.org/MIRA-doc/classmira_1_1Img.html#ad2d56a3b4dd08c1350d629d47b4881b2
    mira::Img<uint8> tmp_img(width, height);

    // fill image with data from msg->data
    // http://www.mira-project.org/MIRA-doc/classmira_1_1ImgTypedBase.html#a89da19eb2bcb0ecb1f04bda7161d8f64
    unsigned int pixel_index = 0;

    // TODO: check if "tmp_img(x,y) = int_value" works or if the Pixel-class is needed
    for(unsigned int y=0; y<height; y++){
        for(unsigned int x=0; x<width; x++){
            if(msg->data[pixel_index] == ROS_UNKNOWN){
                tmp_img(x,y) = MIRA_UNKNOWN; 
            }
            else if(msg->data[pixel_index] <= ROS_FREE_THRESHOLD){
                tmp_img(x,y) = MIRA_FREE;
            }
            else if(msg->data[pixel_index] >= ROS_OCCUPIED_THRESHOLD){
                tmp_img(x,y) = MIRA_OCCUPIED;
            }
            else{ 
                // not really necessary in this scenario,
                // but discarding this case would lead to a big hole between
                //  ROS_FREE_THRESHOLD and ROS_OCCUPIED_THRESHOLD
                tmp_img(x,y) = MIRA_UNKNOWN;
            }

            ++pixel_index;
        }
    }
    
    // create GridMap
    // http://www.mira-project.org/MIRA-doc/toolboxes/Mapping/classmira_1_1maps_1_1GridMap.html#afce7b1f55777a406dfd873c59e78f294

    // TODO: check if the conversion from position.x (float) to Point2i works as wanted
    GridMap<uint8> tmp_map( tmp_img, 
                            msg->info.resolution, 
                            mira::Point2i(  msg->info.origin.position.x, 
                            msg->info.origin.position.y));

    // publish map to MIRA
    ros_grid_to_mira.post(tmp_map);
}


//------------------------------------------------------------------------------
// prepare the two middlewares and hand over to ROS
int main(int argc, char **argv)
{
    //--------------------------------------------------------------------------
    // initialize ROS and MIRA

	// ros init
	ros::init(argc, argv, "Grid2Mira");

	// create and start the mira framework
	mira::Framework framework(argc, argv, true);

    //--------------------------------------------------------------------------
    // MIRA Channels

	// create mira authority and publish the channels
	authority.checkin("/", "Grid2Mira");
    authority.start();

    // publish map from ROS, adapted to the value range of MIRA
    ros_grid_to_mira = authority.publish<GridMap<uint8>>("sensorMap");

    //--------------------------------------------------------------------------
    // ROS nodes
    
	// subscriber
	ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("map", 1000, onNewMap);

	// do the locomotion
	ros::spin();
	return 0;
}
