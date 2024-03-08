#include "ax12a.h"
#include <iostream>
#include <dynamixel_sdk/dynamixel_sdk.h>

using namespace std;

ax12a::ax12a(int id)
  {
    this->id = id;
  }


ax12a::~ax12a()
  {
  }

void ax12a::initport()
  {
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
  }
  }


void ax12a::enabletorque()
  {
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }
  }

void ax12a::setmingoalpos(int minimum)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_MX_CW_ANGLELIMIT, minimum, &dxl_error);
    this->minimum = minimum;
}

void ax12a::setmaxgoalpos(int maximum)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_MX_CCW_ANGLELIMIT, maximum, &dxl_error);
    this->maximum = maximum;
  }

void ax12a::setposition(int &goalpos) //Para escribir valor de posicion deseada
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_MX_GOAL_POSITION, goalpos, &dxl_error);
  }

void ax12a::getposition()  //Para leer posicion actual
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
  }

void ax12a::setspeed(int &speed)  //Para escribir valor de velocidad deseada
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_MX_MOVING_SPEED, speed, &dxl_error);
    this ->speed = speed;
  }

void ax12a::getspeed() //Para leer la velocidad actual
  {
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_MX_PRESENT_SPEED, &dxl_present_speed, &dxl_error);
  }

void ax12a::getAcceleration()
{
    acceleration = abs(dxl_present_speed-lastSpeed);
}

void ax12a::setpunch(int &punch)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_MX_PUNCH, punch, &dxl_error) ;
  }

//int ax12a::getpunch()
//  {

//  }

void ax12a::turnoff()
{
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    portHandler->closePort();
}

void ax12a::ismoving()
{
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, ADDR_MOVING, &dxl_nowmoving, &dxl_error);
}
