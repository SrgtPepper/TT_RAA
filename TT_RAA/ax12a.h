#ifndef AX12A_H
#define AX12A_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

// Control table address
#define ADDR_MX_CW_ANGLELIMIT           6
#define ADDR_MX_CCW_ANGLELIMIT          8
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_PRESENT_SPEED           38
#define ADDR_MOVING                     46
#define ADDR_MX_PUNCH                   48

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold



class ax12a
{
public:

    int id;
    int minimum;
    int maximum;
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0; 
    uint8_t dxl_moving = 0;
    uint16_t dxl_present_position = 0;
    uint16_t dxl_present_speed = 0;
    uint8_t dxl_nowmoving;
    int goalpos;
    int punch;
    int speed;
    int lastSpeed=0;
    int acceleration;

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


    ax12a(int id);
    ~ax12a();

    void initport();
    void enabletorque();
    void turnoff();

    void setmingoalpos(int minimum);
    void setmaxgoalpos(int maximum);

    void setposition(int &goalpos);
    void getposition();

    void setspeed(int &speed);
    void getspeed();

    void getAcceleration();

    void setpunch(int &punch);
    int getpunch();

    void ismoving();


};
#endif // AX12A_H
