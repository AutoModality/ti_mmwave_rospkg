/*
 *  mmWaveDataHdl.cpp
 *
 *  Description:This file implements a ROS nodelet which will open up a serial port provided by the user
 *              at a certain baud rate (also provided by user) that will interface with the 1443EVM mmwDemo
 *              Data UART to be used for board configuration.
 *                       
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include "mmWaveDataHdl.hpp"
#include "DataHandlerClass.h"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveDataHdl, nodelet::Nodelet);

mmWaveDataHdl::mmWaveDataHdl() {}

void mmWaveDataHdl::onInit()
{
   ros::NodeHandle private_nh = getPrivateNodeHandle();
   
   std::string ifname = "can0";
   int mmwave_can_id = 2; // id 2 for mmwave radar
   int myMaxAllowedElevationAngleDeg;
   int myMaxAllowedAzimuthAngleDeg;

   private_nh.getParam("/mmWave_Manager/ifname", ifname);
   
   private_nh.getParam("/mmWave_Manager/mmwave_can_id", mmwave_can_id);
   
   if (!(private_nh.getParam("/mmWave_Manager/max_allowed_elevation_angle_deg", myMaxAllowedElevationAngleDeg)))
   {
      myMaxAllowedElevationAngleDeg = 90;  // Use max angle if none specified
   }

   if (!(private_nh.getParam("/mmWave_Manager/max_allowed_azimuth_angle_deg", myMaxAllowedAzimuthAngleDeg)))
   {
      myMaxAllowedAzimuthAngleDeg = 90;  // Use max angle if none specified
   }

   ROS_INFO("mmWaveDataHdl: ifname = %s", ifname.c_str());
   ROS_INFO("mmWaveDataHdl: mmwave_can_id = %d", mmwave_can_id);
   ROS_INFO("mmWaveDataHdl: max_allowed_elevation_angle_deg = %d", myMaxAllowedElevationAngleDeg);
   ROS_INFO("mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d", myMaxAllowedAzimuthAngleDeg);
   
   DataCANHandler DataHandler(&private_nh, ifname, mmwave_can_id);
   DataHandler.setMaxAllowedElevationAngleDeg( myMaxAllowedElevationAngleDeg );
   DataHandler.setMaxAllowedAzimuthAngleDeg( myMaxAllowedAzimuthAngleDeg );
   DataHandler.start();
   
   NODELET_DEBUG("mmWaveDataHdl: Finished onInit function");
}

}



