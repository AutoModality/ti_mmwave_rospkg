/*
 * DataHandlerClass.cpp
 *
 * This is the implementation of the DataHandlerClass.h
 * Three threads are spawned when start() is called.
 *  1) readIncomingData() thread
 *  2) sortIncomingData() thread
 *  3) syncedBufferSwap() thread
 *  
 * Together they implement a double-buffered read from the data serial port 
 * which sorts the data into the class's mmwDataPacket struct.
 *
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


#include <DataHandlerClass.h>
#include <pthread.h>
#include <algorithm>
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>

#define TIMEOUT_READ_CAN 100

DataCANHandler::DataCANHandler(ros::NodeHandle* nh, std::string ifname, int mmwave_can_id) :
	CanSocket(ifname, mmwave_can_id), currentBufp(&pingPongBuffers[0]), nextBufp(&pingPongBuffers[1])
{
    nodeHandle = nh;
    DataCANHandler_pub = nodeHandle->advertise< sensor_msgs::PointCloud2 >("RScan", 100);
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified
}

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataCANHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
    maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataCANHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
    maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/*Implementation of readIncomingData*/
void *DataCANHandler::readIncomingData(void)
{
    int firstPacketReady = 0;
    uint8_t last8Bytes[8] = {0};
    struct canfd_frame frame;
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    int msg_tl_points = 1;
    int msg_tl_range = 1;
    int msg_tl_noise = 1;
    int msg_tl_stats = 1;
    int count_c1 = 1;
    int count_d1 = 0;
    int count_d2 = 0;
    int count_d3 = 0;
    int count_d6 = 0;
    int count_b1 = 0;
    
    /*Read CAN frame*/
    if( readCAN(&frame) < 0 )
	{
		ROS_INFO("Failed to read CAN bus.");

		return NULL;
	}
    //ROS_INFO("Read one frame from CAN bus.");
	//ROS_INFO_STREAM("CAN ID: 0x" << std::hex << frame.can_id);
	//ROS_INFO_STREAM("Data length: " << (int)frame.len);
	//ROS_INFO_STREAM("Flags: 0x" << std::hex << (int)frame.flags);
	//for (int i = 0; i < frame.len; i++)
	//{
	//	ROS_INFO_STREAM("data[" << i << "]: 0x" << std::hex << (int)frame.data[i]);
	//}

	// Check if it is a header frame
	while((frame.can_id & 0x1FFFFFFFU) != 0xC1)
	{
		/*Read CAN frame*/
		if( readCAN(&frame) < 0 )
		{
			ROS_INFO("Failed to read CAN bus.");

			return NULL;
		}
		//ROS_INFO("Read one frame from CAN bus.");
		//ROS_INFO_STREAM("CAN ID: 0x" << std::hex << (frame.can_id & 0x1FFFFFFFU));
		//ROS_INFO_STREAM("Data length: " << (int)frame.len);
		//ROS_INFO_STREAM("Flags: 0x" << std::hex << (int)frame.flags);
		//for (int i = 0; i < frame.len; i++)
		//{
		//	ROS_INFO_STREAM("data[" << i << "]: 0x" << std::hex << (int)frame.data[i]);
		//}
	}
    
    /*Check if the first 8 bytes are magic numbers, if not something is wrong, not need for CAN bus*/
	last8Bytes[0] = frame.data[0];
	last8Bytes[1] = frame.data[1];
	last8Bytes[2] = frame.data[2];
	last8Bytes[3] = frame.data[3];
	last8Bytes[4] = frame.data[4];
	last8Bytes[5] = frame.data[5];
	last8Bytes[6] = frame.data[6];
	last8Bytes[7] = frame.data[7];
    if(!isMagicWord(last8Bytes))
    {
    	ROS_INFO("Didn't find magic numbers, something wrong.");

    	return NULL;
    }
    //ROS_INFO("Found magic word");
    
    /*Lock nextBufp before entering main loop*/
    pthread_mutex_lock(&nextBufp_mutex);
    
    //push header onto buffer
    for(int i = 8; i < 40; i++)
    {
    	nextBufp->push_back( frame.data[i] );  //push byte onto buffer
    }

	/*print the buffer
	ROS_INFO("Size of buffer: %ld", (*nextBufp).size());
	for(auto i: *nextBufp)
	{
		std::cout << std::hex << (int)i << " " << std::dec;
	}
	std::cout << "\n";*/

    while(ros::ok())
    {
    	/*Read CAN frame*/
		if( readCAN(&frame) < 0 )
		{
			ROS_INFO("Failed to read CAN bus.");

			return NULL;
		}
		//ROS_INFO("Read one frame from CAN bus.");
		//ROS_INFO_STREAM("CAN ID: 0x" << std::hex << frame.can_id);
		//ROS_INFO_STREAM("Data length: " << (int)frame.len);
		//ROS_INFO_STREAM("Flags: 0x" << std::hex << (int)frame.flags);
		//for (int i = 0; i < frame.len; i++)
		//{
		//	ROS_INFO_STREAM("data[" << i << "]: 0x" << std::hex << (int)frame.data[i]);
		//}

		switch((frame.can_id & 0x1FFFFFFFU))
		{
			case 0xC1:
				//ROS_INFO("Number of frame 0xC1: %d", ++count_c1);
			    /*Check if the first 8 bytes are magic numbers, if not something is wrong, not need for CAN bus*/
				last8Bytes[0] = frame.data[0];
				last8Bytes[1] = frame.data[1];
				last8Bytes[2] = frame.data[2];
				last8Bytes[3] = frame.data[3];
				last8Bytes[4] = frame.data[4];
				last8Bytes[5] = frame.data[5];
				last8Bytes[6] = frame.data[6];
				last8Bytes[7] = frame.data[7];
			    if(isMagicWord(last8Bytes))
			    {
			    	//ROS_INFO("Found magic word");

					/*Lock countSync Mutex while unlocking nextBufp so that the swap thread can use it*/
					pthread_mutex_lock(&countSync_mutex);
					pthread_mutex_unlock(&nextBufp_mutex);

					/*increment countSync*/
					countSync++;

					/*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
					if(firstPacketReady == 0)
					{
						countSync++;
						firstPacketReady = 1;
					}

					/*Signal Swap Thread to run if countSync has reached its max value*/
					if(countSync == COUNT_SYNC_MAX)
					{
						pthread_cond_signal(&countSync_max_cv);
					}

					/*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
					pthread_cond_wait(&read_go_cv, &countSync_mutex);

					/*Unlock countSync so that Swap Thread can use it*/
					pthread_mutex_unlock(&countSync_mutex);
					pthread_mutex_lock(&nextBufp_mutex);

					//print the buffer
					//ROS_INFO("Size of buffer: %ld", (*nextBufp).size());
					//for(auto i: *nextBufp)
					//{
					//	std::cout << std::hex << (int)i << " ";
					//}
					//std::cout << std::dec << "\n";

					nextBufp->clear();
					memset(last8Bytes, 0, sizeof(last8Bytes));
					
				    //push header onto buffer
				    for(int i = 8; i < 40; i++)
				    {
				    	nextBufp->push_back( frame.data[i] );  //push bytes onto buffer
				    }
				    
					/*print the buffer
					ROS_INFO("Size of buffer after clear: %ld", (*nextBufp).size());
					for(auto i: *nextBufp)
					{
						std::cout << std::hex << (int)i << " ";
					}
					std::cout << std::dec << "\n";*/
			    }
			    else
			    {
			    	ROS_INFO("Didn't find magic numbers, something wrong.");

			    	return NULL;
			    }

				break;
			case 0xD1:
				//ROS_INFO("Number of frame 0xD1: %d", ++count_d1);
				if(msg_tl_points)
				{
					tlvType = (MmwDemo_Output_TLV_Types)((frame.data[3] << 24) + (frame.data[2] << 16) + (frame.data[1] << 8) + frame.data[0]);
					tlvLen = ((frame.data[7] << 24) + (frame.data[6] << 16) + (frame.data[5] << 8) + frame.data[4]);
					//std::cout << "tlvType: " << tlvType << "; tlvLen: 0x" << std::hex << tlvLen << std::dec << "\n";
					
					for(int i = 0; i < 8; i++)
					{
						nextBufp->push_back( frame.data[i] );
					}

					msg_tl_points = 0;
				}
				else
				{
					if (tlvLen > 64)
					{
						for(int i = 0; i < 64; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						tlvLen -= 64;
					}
					else
					{
						for(int i = 0; i < tlvLen; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						msg_tl_points = 1;					

						//print the buffer
						//ROS_INFO("Size of buffer: %ld", (*nextBufp).size());
						//for(auto i: *nextBufp)
						//{
						//	std::cout << std::hex << (int)i << " ";
						//}
						//std::cout << std::dec << "\n";
					}
				}
				
				break;
			case 0xD2:
				//ROS_INFO("Number of frame 0xD2: %d", ++count_d2);
				if(msg_tl_range)
				{
					tlvType = (MmwDemo_Output_TLV_Types)((frame.data[3] << 24) + (frame.data[2] << 16) + (frame.data[1] << 8) + frame.data[0]);
					tlvLen = ((frame.data[7] << 24) + (frame.data[6] << 16) + (frame.data[5] << 8) + frame.data[4]);
					//std::cout << "tlvType: " << tlvType << "; tlvLen: 0x" << std::hex << tlvLen << std::dec << "\n";
					
					for(int i = 0; i < 8; i++)
					{
						nextBufp->push_back( frame.data[i] );
					}

					msg_tl_range = 0;
				}
				else
				{
					if (tlvLen > 64)
					{
						for(int i = 0; i < 64; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						tlvLen -= 64;
					}
					else
					{
						for(int i = 0; i < tlvLen; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						msg_tl_range = 1;					

						//print the buffer
						//ROS_INFO("Size of buffer: %ld", (*nextBufp).size());
						//for(auto i: *nextBufp)
						//{
						//	std::cout << std::hex << (int)i << " ";
						//}
						//std::cout << std::dec << "\n";
					}
				}

				break;
			case 0xD3:
				//ROS_INFO("Number of frame 0xD3: %d", ++count_d3);
				if(msg_tl_noise)
				{
					tlvType = (MmwDemo_Output_TLV_Types)((frame.data[3] << 24) + (frame.data[2] << 16) + (frame.data[1] << 8) + frame.data[0]);
					tlvLen = ((frame.data[7] << 24) + (frame.data[6] << 16) + (frame.data[5] << 8) + frame.data[4]);
					//std::cout << "tlvType: " << tlvType << "; tlvLen: 0x" << std::hex << tlvLen << std::dec << "\n";
					
					for(int i = 0; i < 8; i++)
					{
						nextBufp->push_back( frame.data[i] );
					}

					msg_tl_noise = 0;
				}
				else
				{
					if (tlvLen > 64)
					{
						for(int i = 0; i < 64; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						tlvLen -= 64;
					}
					else
					{
						for(int i = 0; i < tlvLen; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						msg_tl_noise = 1;					

						//print the buffer
						//ROS_INFO("Size of buffer: %ld", (*nextBufp).size());
						//for(auto i: *nextBufp)
						//{
						//	std::cout << std::hex << (int)i << " ";
						//}
						//std::cout << std::dec << "\n";
					}
				}

				break;
			case 0xD6:
				//ROS_INFO("Number of frame 0xD6: %d", ++count_d6);
				if(msg_tl_stats)
				{
					tlvType = (MmwDemo_Output_TLV_Types)((frame.data[3] << 24) + (frame.data[2] << 16) + (frame.data[1] << 8) + frame.data[0]);
					tlvLen = ((frame.data[7] << 24) + (frame.data[6] << 16) + (frame.data[5] << 8) + frame.data[4]);
					//std::cout << "tlvType: " << tlvType << "; tlvLen: 0x" << std::hex << tlvLen << std::dec << "\n";
					
					for(int i = 0; i < 8; i++)
					{
						nextBufp->push_back( frame.data[i] );
					}

					msg_tl_stats = 0;
				}
				else
				{
					if (tlvLen > 64)
					{
						for(int i = 0; i < 64; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						tlvLen -= 64;
					}
					else
					{
						for(int i = 0; i < tlvLen; i++)
						{
							nextBufp->push_back( frame.data[i] );
						}

						msg_tl_stats = 1;					

						//print the buffer
						//ROS_INFO("Size of buffer: %ld", (*nextBufp).size());
						//for(auto i: *nextBufp)
						//{
						//	std::cout << std::hex << (int)i << " ";
						//}
						//std::cout << std::dec << "\n";
					}
				}

				break;
			case 0xB1:
				//ROS_INFO("Number of frame 0xB1: %d", ++count_b1);
				break;
			default:
				break;
		}
    }
    
    pthread_exit(NULL);
}


int DataCANHandler::isMagicWord(uint8_t last8Bytes[8])
{
    int val = 0, i = 0, j = 0;
    
    for(i = 0; i < 8 ; i++)
    {
    
       if( last8Bytes[i] == magicWord[i])
       {
          j++;
       }
    
    }
    
    if( j == 8)
    {
       val = 1;
    }
    
    return val;  
}

void *DataCANHandler::syncedBufferSwap(void)
{
    while(ros::ok())
    {
        pthread_mutex_lock(&countSync_mutex);
    
        while(countSync < COUNT_SYNC_MAX)
        {
            pthread_cond_wait(&countSync_max_cv, &countSync_mutex);
            
            pthread_mutex_lock(&currentBufp_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            std::vector<uint8_t>* tempBufp = currentBufp;
        
            this->currentBufp = this->nextBufp;
            
            this->nextBufp = tempBufp;
            
            pthread_mutex_unlock(&currentBufp_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            countSync = 0;
            
            pthread_cond_signal(&sort_go_cv);
            pthread_cond_signal(&read_go_cv);
            
        }
    
        pthread_mutex_unlock(&countSync_mutex);

    }

    pthread_exit(NULL);
    
}

void *DataCANHandler::sortIncomingData( void )
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
    unsigned int currentDatap = 0;
    SorterState sorterState = READ_HEADER;
    int i = 0, tlvCount = 0, offset = 0;
    float maxElevationAngleRatioSquared;
    float maxAzimuthAngleRatio;
    
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
    
    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);
    
    pthread_mutex_lock(&currentBufp_mutex);
    
    while(ros::ok())
    {
        
        switch(sorterState)
        {
            
        case READ_HEADER:
            
            /*print the buffer
			ROS_INFO("Size of buffer: %ld", (*currentBufp).size());
			for(auto i: *currentBufp)
			{
				std::cout << std::hex << (int)i << " ";
			}
			std::cout << std::dec << "\n";*/
			
            //make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)            
            if(currentBufp->size() < 12)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            
            //get version (4 bytes)           
            memcpy( &mmwData.header.version, &currentBufp->at(currentDatap), sizeof(mmwData.header.version)); 
			//ROS_INFO("mmwData.header.version: 0x%x", mmwData.header.version);
            currentDatap += ( sizeof(mmwData.header.version) );
            
            //get totalPacketLen (4 bytes)
            memcpy( &mmwData.header.totalPacketLen, &currentBufp->at(currentDatap), sizeof(mmwData.header.totalPacketLen));
			//ROS_INFO("mmwData.header.totalPacketLen: %d", mmwData.header.totalPacketLen);
            currentDatap += ( sizeof(mmwData.header.totalPacketLen) );
            
            //get platform (4 bytes)
            memcpy( &mmwData.header.platform, &currentBufp->at(currentDatap), sizeof(mmwData.header.platform));
			//ROS_INFO("mmwData.header.platform: 0x%x", mmwData.header.platform);
            currentDatap += ( sizeof(mmwData.header.platform) );      
            
            //if packet doesn't have correct header size (which is based on platform and SDK version), throw it away (does not include magicWord since it was already removed)
			if((((mmwData.header.version >> 24) & 0xFF) < 1) || (((mmwData.header.version >> 16) & 0xFF) < 1))  //check if SDK version is older than 1.1
			{
		       //ROS_INFO("mmWave device firmware detected version: 0x%8.8X", mmwData.header.version);
			   headerSize = 28;
			}
		        else if((mmwData.header.platform & 0xFFFF) == 0x1443)
			{
			   headerSize = 28;
			}
			else  // 1642
			{
			   headerSize = 32;
			}
            if(currentBufp->size() < headerSize)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            //ROS_INFO("currentBufp->size(): %ld", currentBufp->size());
            //ROS_INFO("headerSize: %d", headerSize);
            
            //get frameNumber (4 bytes)
            memcpy( &mmwData.header.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.frameNumber));
			//ROS_INFO("mmwData.header.frameNumber: 0x%x", mmwData.header.frameNumber);
            currentDatap += ( sizeof(mmwData.header.frameNumber) );
            
            //get timeCpuCycles (4 bytes)
            memcpy( &mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap), sizeof(mmwData.header.timeCpuCycles));
			//ROS_INFO("mmwData.header.timeCpuCycles: 0x%x", mmwData.header.timeCpuCycles);
            currentDatap += ( sizeof(mmwData.header.timeCpuCycles) );
            
            //get numDetectedObj (4 bytes)
            memcpy( &mmwData.header.numDetectedObj, &currentBufp->at(currentDatap), sizeof(mmwData.header.numDetectedObj));
			//ROS_INFO("mmwData.header.numDetectedObj: %d", mmwData.header.numDetectedObj);
            currentDatap += ( sizeof(mmwData.header.numDetectedObj) );
            
            //get numTLVs (4 bytes)
            memcpy( &mmwData.header.numTLVs, &currentBufp->at(currentDatap), sizeof(mmwData.header.numTLVs));
			//ROS_INFO("mmwData.header.numTLVs: %d", mmwData.header.numTLVs);
            currentDatap += ( sizeof(mmwData.header.numTLVs) );
            
            //get subFrameNumber (4 bytes) (not used for XWR1443)
            if((mmwData.header.platform & 0xFFFF) != 0x1443)
	    	{
               memcpy( &mmwData.header.subFrameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.subFrameNumber));
			   //ROS_INFO("mmwData.header.subFrameNumber: 0x%x", mmwData.header.subFrameNumber);
               currentDatap += ( sizeof(mmwData.header.subFrameNumber) );
	    	}

            //if packet lengths do not patch, throw it away
            //ROS_INFO("mmwData.header.totalPacketLen: %d", mmwData.header.totalPacketLen);
            //ROS_INFO("(48-8-8+8*4+mmwData.header.numDetectedObj*12+4+2*64*8+24): %d", (48-8-8+8*4+mmwData.header.numDetectedObj*12+4+2*64*8+24));
            //ROS_INFO("currentBufp->size(): %ld", currentBufp->size());
            //if(mmwData.header.totalPacketLen == currentBufp->size() )
            if( (48-8-8+8*4+mmwData.header.numDetectedObj*12+4+2*64*8+24) == currentBufp->size() )
            //48: header size
            //8: magic number
            //8: padding of header
            //8*4: D1/D2/D3/D6 tl (type & length)
            //mmwData.header.numDetectedObj*12: each object is 12 bytes
            //4: the first two 16 bits are id and number of objects
            //2*64*8: D2 and D3 are 8 frames of 64 bytes
            //24: length of D6
            {//On CAN bus the packet length seems always not equal to the size of data passed in one packet.
               sorterState = CHECK_TLV_TYPE;
            }
            else sorterState = SWAP_BUFFERS;

            break;
            
        case READ_OBJ_STRUCT:
            //ROS_INFO("READ_OBJ_STRUCT");
            i = 0;
            offset = 0;
            
            //get number of objects
            memcpy( &mmwData.numObjOut, &currentBufp->at(currentDatap), sizeof(mmwData.numObjOut));
            currentDatap += ( sizeof(mmwData.numObjOut) );
            
            //get xyzQFormat
            memcpy( &mmwData.xyzQFormat, &currentBufp->at(currentDatap), sizeof(mmwData.xyzQFormat));
            currentDatap += ( sizeof(mmwData.xyzQFormat) );
            
            RScan->header.seq = 0;
            //RScan->header.stamp = (uint32_t) mmwData.header.timeCpuCycles;
            RScan->header.frame_id = "base_radar_link";
            RScan->height = 1;
            RScan->width = mmwData.numObjOut;
            RScan->is_dense = 1;
            RScan->points.resize(RScan->width * RScan->height);
            
            // Calculate ratios for max desired elevation and azimuth angles
            if ((maxAllowedElevationAngleDeg >= 0) && (maxAllowedElevationAngleDeg < 90))
            {
                maxElevationAngleRatioSquared = tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
                maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
            }
            else
            {
                maxElevationAngleRatioSquared = -1;
            }
            if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90))
            {
                maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
            }
            else
            {
                maxAzimuthAngleRatio = -1;
            }
            //ROS_INFO("maxElevationAngleRatioSquared = %f", maxElevationAngleRatioSquared);
            //ROS_INFO("maxAzimuthAngleRatio = %f", maxAzimuthAngleRatio);
            //ROS_INFO("mmwData.numObjOut before = %d", mmwData.numObjOut);


            //set some parameters for pointcloud
            while( i < mmwData.numObjOut )
            {
                //get object range index
                memcpy( &mmwData.objOut.rangeIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.rangeIdx));
                currentDatap += ( sizeof(mmwData.objOut.rangeIdx) );
                
                //get object doppler index
                memcpy( &mmwData.objOut.dopplerIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.dopplerIdx));
                currentDatap += ( sizeof(mmwData.objOut.dopplerIdx) );
                
                //get object peak intensity value
                memcpy( &mmwData.objOut.peakVal, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.peakVal));
                currentDatap += ( sizeof(mmwData.objOut.peakVal) );
                
                //get object x-coordinate
                memcpy( &mmwData.objOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.x));
                currentDatap += ( sizeof(mmwData.objOut.x) );
                
                //get object y-coordinate
                memcpy( &mmwData.objOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.y));
                currentDatap += ( sizeof(mmwData.objOut.y) );
                
                //get object z-coordinate
                memcpy( &mmwData.objOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.z));
                currentDatap += ( sizeof(mmwData.objOut.z) );
                
                //convert from Qformat to float(meters)
                float temp[4];
                
                temp[0] = (float) mmwData.objOut.x;
                temp[1] = (float) mmwData.objOut.y;
                temp[2] = (float) mmwData.objOut.z;
                //temp[4] = //doppler 
                
                for(int j = 0; j < 3; j++)
                {
                    if(temp[j] > 32767)
                        temp[j] -= 65535;
                    
                    temp[j] = temp[j] / pow(2,mmwData.xyzQFormat);
                 }   
                 
                // Convert intensity to dB
                temp[3] = 10 * log10(mmwData.objOut.peakVal + 1);  // intensity
                
                // Map mmWave sensor coordinates to ROS coordinate system
                RScan->points[i].x = temp[1];   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                RScan->points[i].y = -temp[0];  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                RScan->points[i].z = temp[2];   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                RScan->points[i].intensity = temp[3];
               
                // Keep point if elevation and azimuth angles are less than specified max values
                // (NOTE: The following calculations are done using ROS standard coordinate system axis definitions where X is forward and Y is left)
                if (((maxElevationAngleRatioSquared == -1) ||
                     (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                                                                    RScan->points[i].y * RScan->points[i].y)
                      ) < maxElevationAngleRatioSquared)
                    ) &&
                    ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
		            (RScan->points[i].x != 0)
                   )
                {
                    //ROS_INFO("Kept point");
                    i++;
                }

                // Otherwise, remove the point
                else
                {
                    //ROS_INFO("Removed point");
                    mmwData.numObjOut--;
                }
            }

            // Resize point cloud since some points may have been removed
            RScan->width = mmwData.numObjOut;
            RScan->points.resize(RScan->width * RScan->height);
            
            //ROS_INFO("mmwData.numObjOut after = %d", mmwData.numObjOut);
            //ROS_INFO("DataCANHandler Sort Thread: number of obj = %d", mmwData.numObjOut );
            
            DataCANHandler_pub.publish(RScan);
            
            sorterState = CHECK_TLV_TYPE;
            
            break;
            
        case READ_LOG_MAG_RANGE:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataCANHandler Sort Thread : Parsing Range Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_NOISE:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataCANHandler Sort Thread : Parsing Noise Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
           
            break;
            
        case READ_AZIMUTH:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataCANHandler Sort Thread : Parsing Azimuth Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_DOPPLER:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataCANHandler Sort Thread : Parsing Doppler Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_STATS:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataCANHandler Sort Thread : Parsing Stats Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
        
        case CHECK_TLV_TYPE:
        
            //ROS_INFO("DataCANHandler Sort Thread : tlvCount = %d, numTLV = %d", tlvCount, mmwData.header.numTLVs);
        
            if(tlvCount++ >= mmwData.header.numTLVs)
            {
                //ROS_INFO("DataCANHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                sorterState = SWAP_BUFFERS;
            }
            else
            {
               //get tlvType (32 bits) & remove from queue
                memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                currentDatap += ( sizeof(tlvType) );
                
                //ROS_INFO("DataCANHandler Sort Thread : sizeof(tlvType) = %d", sizeof(tlvType));
            
                //get tlvLen (32 bits) & remove from queue
                memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                currentDatap += ( sizeof(tlvLen) );
                
                //ROS_INFO("DataCANHandler Sort Thread : sizeof(tlvLen) = %d", sizeof(tlvLen));
                
                //ROS_INFO("DataCANHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);
            
                switch(tlvType)
                {
                case MMWDEMO_OUTPUT_MSG_NULL:
                
                    break;
                
                case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                    //ROS_INFO("DataCANHandler Sort Thread : Object TLV");
                    sorterState = READ_OBJ_STRUCT;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                    //ROS_INFO("DataCANHandler Sort Thread : Range TLV");
                    sorterState = READ_LOG_MAG_RANGE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                    //ROS_INFO("DataCANHandler Sort Thread : Noise TLV");
                    sorterState = READ_NOISE;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
                    //ROS_INFO("DataCANHandler Sort Thread : Azimuth Heat TLV");
                    sorterState = READ_AZIMUTH;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                    //ROS_INFO("DataCANHandler Sort Thread : R/D Heat TLV");
                    sorterState = READ_DOPPLER;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_STATS:
                    //ROS_INFO("DataCANHandler Sort Thread : Stats TLV");
                    sorterState = READ_STATS;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_MAX:
                    //ROS_INFO("DataCANHandler Sort Thread : Header TLV");
                    sorterState = READ_HEADER;
                    break;
                
                default: break;
                }
            }
            
        break;
            
       case SWAP_BUFFERS:
       
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&currentBufp_mutex);
                            
            countSync++;
                
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
                
            pthread_cond_wait(&sort_go_cv, &countSync_mutex);
                
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&currentBufp_mutex);
                
            currentDatap = 0;
            tlvCount = 0;
                
            sorterState = READ_HEADER;
            
            break;
                
            
        default: break;
        }
    }
    
    
    pthread_exit(NULL);
}

void DataCANHandler::start(void)
{
    
    pthread_t uartThread, sorterThread, swapThread;
    
    int  iret1, iret2, iret3;
    
    pthread_mutex_init(&countSync_mutex, NULL);
    pthread_mutex_init(&nextBufp_mutex, NULL);
    pthread_mutex_init(&currentBufp_mutex, NULL);
    pthread_cond_init(&countSync_max_cv, NULL);
    pthread_cond_init(&read_go_cv, NULL);
    pthread_cond_init(&sort_go_cv, NULL);
    
    countSync = 0;
    
    /* Create independent threads each of which will execute function */
    iret1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
    if(iret1)
    {
     ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
     ros::shutdown();
    }
    
    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    ros::spin();

    pthread_join(iret1, NULL);
    ROS_INFO("DataCANHandler Read Thread joined");
    pthread_join(iret2, NULL);
    ROS_INFO("DataCANHandler Sort Thread joined");
    pthread_join(iret3, NULL);
    ROS_INFO("DataCANHandler Swap Thread joined");
    
    pthread_mutex_destroy(&countSync_mutex);
    pthread_mutex_destroy(&nextBufp_mutex);
    pthread_mutex_destroy(&currentBufp_mutex);
    pthread_cond_destroy(&countSync_max_cv);
    pthread_cond_destroy(&read_go_cv);
    pthread_cond_destroy(&sort_go_cv);
    
    
}

void* DataCANHandler::readIncomingData_helper(void *context)
{  
    return (static_cast<DataCANHandler*>(context)->readIncomingData());
}

void* DataCANHandler::sortIncomingData_helper(void *context)
{  
    return (static_cast<DataCANHandler*>(context)->sortIncomingData());
}

void* DataCANHandler::syncedBufferSwap_helper(void *context)
{  
    return (static_cast<DataCANHandler*>(context)->syncedBufferSwap());
}
