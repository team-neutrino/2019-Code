/*
 * Neutrino lidar
 * 
 * Heavily edited version of RPLIDAR ultra-simple app,
 * which now serves X and Y coords of valid samples over UDP.
 * 
 * The original information and licence is below.
 */



/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cstring>
#include <math.h>
#include <thread>
#include <atomic>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
//#include <wiringPi.h>
#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header



#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>

#define PI 3.14159265





///////////////////////////////////////////////////
/////////////// IMPORTANT CONSTANTS ///////////////
///////////////////////////////////////////////////


#define DEST_IP "10.39.28.5"
#define DEST_PORT 5800
#define ROBO_IP "10.39.28.2"
#define ROBO_PORT 5801

#define LIDAR_ANGLE_OFFSET 90.0f
#define LIDAR_X_OFFSET 0.0f
#define LIDAR_Y_OFFSET 0.0f


//////////////////////////////////////////////////////////
/////////////// END OF IMPORTANT CONSTANTS ///////////////
//////////////////////////////////////////////////////////



////////////////////////
// IMPORANT VARIABLES //
////////////////////////

std::atomic<bool> activated(false);
std::atomic<bool> sw(true);
std::atomic<float> navxAngleBuf(0.0f);

////////////////////////////////
// END OF IMPORTANT VARIABLES //
////////////////////////////////

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    sw = false;
    ctrl_c_pressed = true;
}



int controlrecv()
{
    printf("Got into control thread\n");
    
    int robo;
    if ( (robo = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket failed");
        return 1;
    }
    
    socklen_t l;
    struct sockaddr_in client;
    memset( &client, 0, sizeof client );
    
    struct sockaddr_in servaddr;
    memset( &servaddr, 0, sizeof(servaddr) );
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons( ROBO_PORT );              
    //roboaddr.sin_addr.s_addr = inet_addr(ROBO_IP);  
    servaddr.sin_addr.s_addr = INADDR_ANY;

    char robobuf[5];


    float fbuf = 0.0f;
    
    
    
    if (bind(robo, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) {
        perror("socket failed");
        return 1;
    }
    
    printf("Got past thread client setup\n");
    
    while(sw.load()){
        recvfrom(robo, robobuf, 5, MSG_WAITALL, (struct sockaddr *) &client, (socklen_t*) &l);
        memcpy(&fbuf, &robobuf, sizeof(float));
        //fbuf=90.0f;
        //printf("%f\n",fbuf);
        navxAngleBuf.store(fbuf);
        
        if((int) robobuf[4] == 0x01){
            printf("GOT ACTIVATION SIGNAL\n");
            activated = true;
        }
        
        if((int) robobuf[4] == 0x02){
            printf("GOT STOP SIGNAL\n");
            activated = false;
            sw = false;
        }
            
    }
    
    close( robo );
    return 0;
}



int main(int argc, const char * argv[]) {
    
    //wiringPiSetup();
    
    const char * opt_com_path = NULL;
    const char * opt_ser_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;
    
    char msgbuf[26];
    
    float navxAngle = 0.0f;
    
    
    
    
    
    printf("Got past basic variables\n");
    
    int dest;
    if ( (dest = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket failed");
        return 1;
    }

    struct sockaddr_in destaddr;
    memset( &destaddr, 0, sizeof destaddr );
    destaddr.sin_family = AF_INET;
    destaddr.sin_port = htons( DEST_PORT );              
    destaddr.sin_addr.s_addr = inet_addr(DEST_IP);  

    printf("Got past server variable setup\n");
    
    
    
    std::thread threadThing(controlrecv);
    
    printf("WAITING FOR ROBORIO\n");
    
    while(!activated.load())
    {
        /////////////////////////
        // Wait for activation //
        /////////////////////////
    }



    printf("Project LiMIT RoboRIO - Lidar adapter \n"
           "Version: "RPLIDAR_SDK_VERSION"\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }


    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
    drv->startMotor();
    // start scan...
    drv->startScan(0,1);

    
    
    // fetech result and print it out...
    while (activated.load()) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                
                if((nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) > 0 || (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)){
                    
                
                    std::memset(msgbuf, 0, sizeof msgbuf);
                
                    if(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT){
                        navxAngle = navxAngleBuf.load();
                    }
                
                    //std::snprintf(msgbuf, sizeof msgbuf, "%06.2f,%08.2f", 
                    //    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    //    nodes[pos].distance_q2/4.0f);
                
                
                    std::snprintf(msgbuf, sizeof msgbuf, "%s,%09.2f,%09.2f,%03.0f", 
                        (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"1":"0",
                        cos((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f * (PI/180) + LIDAR_ANGLE_OFFSET - navxAngle)*nodes[pos].distance_q2/4.0f + LIDAR_X_OFFSET,
                        sin((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f * (PI/180) + LIDAR_ANGLE_OFFSET - navxAngle)*nodes[pos].distance_q2/4.0f + LIDAR_Y_OFFSET,
                        navxAngle);
                    
                

                    sendto( dest, msgbuf, sizeof msgbuf, 0, (struct sockaddr *)&destaddr, sizeof(destaddr));

                
                
                
                
                
                
                    printf("%s,%09.2f,%09.2f,%03.0f \n", 
                        (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"1":"0",
                        cos((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f * (PI/180))*nodes[pos].distance_q2/4.0f,
                        sin((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f * (PI/180))*nodes[pos].distance_q2/4.0f,
                        navxAngle);
                    
                    //printf("%f,\n", navxAngle);
                    
                    //printf("%03.2f - %08.2f - Q: %d", 
                    //    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    //    nodes[pos].distance_q2/4.0f,
                    //    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                    
                }
            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }
    threadThing.join();
    close( dest );
    
    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}

