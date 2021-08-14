#include "epos2.h"

EPOS2::EPOS2()
{
  EPOS2::init("USB0");
}

EPOS2::EPOS2(const char* port)
{
  EPOS2::init(port);
}

EPOS2::~EPOS2()
{
  closeEPOSDevice();
}

bool EPOS2::isConnected()
{
  return g_isConnected;
}

bool EPOS2::callback(volksbot::velocities::Request& vel, volksbot::velocities::Response& response)
{
  lastcommand = ros::Time::now();
  leftvel = vel.left;
  rightvel = vel.right;
  limitVelocities(leftvel, rightvel);

  return true;
}

void EPOS2::Vcallback(const volksbot::velsConstPtr& vel)
{
  // ROS_INFO("Velocity Callback");
  lastcommand = ros::Time::now();
  leftvel = vel->left;
  rightvel = vel->right;
  limitVelocities(leftvel, rightvel);
}

void EPOS2::CVcallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  lastcommand = ros::Time::now();
  vx = cmd_vel->linear.x;
  vth = cmd_vel->angular.z;
  double linear = -cmd_vel->linear.x * 100.0;  // from m/s to cm/s
  double v_diff = cmd_vel->angular.z * 22.2;

  // 22.2 is half of baseline of the robot
  // for a 180° turn (= pi rad / sec) both wheels need to move half
  // of the circumference of the circle defined by the baseline
  // ,i.e. pi * 22.2
  //  double v_diff = cmd_vel->angular.z * 9.4468085;

  if (linear > 0)
  {
    leftvel = (double)(linear - v_diff);
    rightvel = (double)(linear + v_diff);
  }
  else
  {
    leftvel = (double)(linear - v_diff);
    rightvel = (double)(linear + v_diff);
  }

  limitVelocities(leftvel, rightvel);
}

void EPOS2::limitCallback(const volksbot::vel_limitConstPtr& limit_vel)
{
  if (fabs(limit_vel->left_neg) <= 100)
  {
    left_neg = limit_vel->left_neg;
  }
  if (fabs(limit_vel->right_neg) <= 100)
  {
    right_neg = limit_vel->right_neg;
  }
  if (fabs(limit_vel->left_pos) <= 100)
  {
    left_pos = limit_vel->left_pos;
  }
  if (fabs(limit_vel->right_pos) <= 100)
  {
    right_pos = limit_vel->right_pos;
  }
  ROS_INFO("Updated velocity limits");
}

void EPOS2::limitVelocities(double& leftvel, double& rightvel)
{
  if (leftvel > left_pos)
  {
    leftvel = left_pos;
  }
  else if (leftvel < left_neg)
  {
    leftvel = left_neg;
  }
  if (rightvel > right_pos)
  {
    rightvel = right_pos;
  }
  else if (rightvel < right_neg)
  {
    rightvel = right_neg;
  }
}

void* EPOS2::threadFunction(void* param)
{
  EPOS2* ref = (EPOS2*)param;

  volksbot::ticks t;

  t.header.frame_id = "base_link";

  if (VCS_ActivateProfileVelocityMode(ref->g_pKeyHandle, 0, &ref->g_pErrorCode) == 0 ||
      VCS_ActivateProfileVelocityMode(ref->g_pKeyHandle, 1, &ref->g_pErrorCode) == 0)
  {
    ROS_INFO("Cannot Activate Velocity Mode in EPOS2 Device! (0x%x)", ref->g_pErrorCode);
  }

  while (ref->isConnected())
  {
    ros::Time current = ros::Time::now();
    t.header.stamp = ros::Time::now();

    int tics_left = 0;
    int tics_right = 0;

    if (current - ref->lastcommand < ros::Duration(50.5))
    {
      if (VCS_MoveWithVelocity(ref->g_pKeyHandle, 1, -ref->rightvel * MAX_RPM / 100.0, &ref->g_pErrorCode) == 0 ||
          VCS_MoveWithVelocity(ref->g_pKeyHandle, 0, ref->leftvel * MAX_RPM / 100, &ref->g_pErrorCode) == 0)
      {
        ROS_INFO("Cannot Send Move Instruction to EPOS2 Device! (0x%x)", ref->g_pErrorCode);
      }

      if (VCS_GetPositionIs(ref->g_pKeyHandle, 1, &tics_right, &ref->g_pErrorCode) == 0 ||
          VCS_GetPositionIs(ref->g_pKeyHandle, 0, &tics_left, &ref->g_pErrorCode) == 0)
      {
        ROS_INFO("Cannot Get Position of EPOS2 Device! (0x%x)", ref->g_pErrorCode);
      }
    }
    else
    {
      if (VCS_MoveWithVelocity(ref->g_pKeyHandle, 1, 0, &ref->g_pErrorCode) == 0 ||
          VCS_MoveWithVelocity(ref->g_pKeyHandle, 0, 0, &ref->g_pErrorCode) == 0)
      {
        ROS_INFO("Cannot Send Move Instruction to EPOS2 Device! (0x%x)", ref->g_pErrorCode);
      }
    }

    t.left = tics_left;
    t.right = -tics_right;

    t.vx = ref->vx;
    t.vth = ref->vth;

    ref->pub.publish(t);

    usleep(50000);
  }

  if (VCS_HaltVelocityMovement(ref->g_pKeyHandle, 1, &ref->g_pErrorCode) == 0 ||
      VCS_HaltVelocityMovement(ref->g_pKeyHandle, 0, &ref->g_pErrorCode) == 0)
  {
    ROS_INFO("Cannot Stop Velocity Mode in EPOS2 Device! (0x%x)", ref->g_pErrorCode);
  }
}

void EPOS2::init(const char* port)
{
  leftvel = 0.0;
  rightvel = 0.0;
  vx = 0;
  vth = 0;

  g_pKeyHandle = 0;
  strcpy(g_DeviceName, "EPOS2");
  strcpy(g_ProtocolStackName, "MAXON SERIAL V2");
  strcpy(g_InterfaceName, "USB");
  strcpy(g_PortName, port);
  g_baudrate = 1000000;
  g_isConnected = false;

  if (!openEPOSDevice())
  {
    ROS_INFO("Cannot Open EPOS2 Device! (0x%x)", g_pErrorCode);
  }
  else
  {
    if (!prepareEPOSDevice(0) || !prepareEPOSDevice(1))
    {  // R = 0, L = 1

      ROS_INFO("Error Preparing EPOS2 Device! (0x%x)", g_pErrorCode);
    }
    else
    {
      g_isConnected = true;

      pub = n.advertise<volksbot::ticks>("VMC", 20);
      sub =
          n.subscribe("Vel", 100, &EPOS2::Vcallback, this, ros::TransportHints().reliable().udp().maxDatagramSize(100));
      cmd_vel_sub_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &EPOS2::CVcallback, this,
                                                       ros::TransportHints().reliable().udp().maxDatagramSize(100));
      limit_sub = n.subscribe<volksbot::vel_limit>("vel_limit", 1, &EPOS2::limitCallback, this);
      service = n.advertiseService("Controls", &EPOS2::callback, this);

      pthread_create(&threadId, NULL, &EPOS2::threadFunction, this);
    }
  }
}

bool EPOS2::openEPOSDevice()
{
  bool lResult = false;

  g_pKeyHandle = VCS_OpenDevice(g_DeviceName, g_ProtocolStackName, g_InterfaceName, g_PortName, &g_pErrorCode);

  if (g_pKeyHandle != 0 && g_pErrorCode == 0)
  {
    unsigned int lBaudrate = 0;
    unsigned int lTimeout = 0;

    if (VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, &g_pErrorCode) != 0)
    {
      if (VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, &g_pErrorCode) != 0)
      {
        if (VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, &g_pErrorCode) != 0)
        {
          if (g_baudrate == (int)lBaudrate)
          {
            lResult = true;
          }
        }
      }
    }
  }
  else
  {
    g_pKeyHandle = 0;
  }

  return lResult;
}

bool EPOS2::prepareEPOSDevice(int g_usNodeId)
{
  bool lResult = true;
  int oIsFault = 0;

  if (VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, &g_pErrorCode) == 0)
  {
    ROS_INFO("GetFaultState");
    lResult = false;
  }

  if (lResult)
  {
    if (oIsFault)
    {
      if (VCS_ClearFault(g_pKeyHandle, g_usNodeId, &g_pErrorCode) == 0)
      {
        ROS_INFO("ClearFault");
        lResult = false;
      }
    }

    if (lResult)
    {
      int oIsEnabled = 0;

      if (VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, &g_pErrorCode) == 0)
      {
        ROS_INFO("GetEnableState");
        lResult = false;
      }

      if (lResult)
      {
        if (!oIsEnabled)
        {
          if (VCS_SetEnableState(g_pKeyHandle, g_usNodeId, &g_pErrorCode) == 0)
          {
            ROS_INFO("SetEnableState");
            lResult = false;
          }
        }
      }
    }
  }

  return lResult;
}

bool EPOS2::closeEPOSDevice()
{
  int lResult = false;

  if (VCS_CloseDevice(g_pKeyHandle, &g_pErrorCode) != 0 && g_pErrorCode == 0)
  {
    lResult = true;
  }

  return lResult;
}
