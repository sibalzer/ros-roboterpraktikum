#include "MotorDummy.h"

MotorDummy::MotorDummy()
{
  absolute_rotations_left = 0;
  absolute_rotations_right = 0;

  ticks_per_cm = -1811.9178;  // rotations per 100? second
  frequency = 20;             // Hz
  MotorDummy::init();

  left_neg = -100;
  right_neg = -100;
  left_pos = 100;
  right_pos = 100;
}

MotorDummy::~MotorDummy()
{
}

bool MotorDummy::callback(volksbot::velocities::Request& vel, volksbot::velocities::Response& response)
{
  lastcommand = ros::Time::now();
  leftvel = vel.left;
  rightvel = vel.right;
  limitVelocities(leftvel, rightvel);

  return true;
}

void MotorDummy::Vcallback(const volksbot::velsConstPtr& vel)
{
  // ROS_INFO("Velocity Callback");
  lastcommand = ros::Time::now();
  leftvel = vel->left;
  rightvel = vel->right;
  limitVelocities(leftvel, rightvel);
}

void MotorDummy::CVcallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
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

void MotorDummy::limitCallback(const volksbot::vel_limitConstPtr& limit_vel)
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

void MotorDummy::limitVelocities(double& leftvel, double& rightvel)
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

void* MotorDummy::threadFunction(void* param)
{
  MotorDummy* ref = (MotorDummy*)param;

  volksbot::ticks t;

  t.header.frame_id = "base_link";

  ros::Rate r(ref->frequency);

  while (ros::ok())
  {
    ros::Time current = ros::Time::now();
    t.header.stamp = current;

    int tics_left = 0;
    int tics_right = 0;

    // use ref->rightvel*MAX_RPM/100 for right wheel
    // use ref->leftvel*MAX_RPM/100 for left wheel

    ref->absolute_rotations_left += ref->ticks_per_cm * ref->leftvel / ref->frequency;
    ref->absolute_rotations_right += ref->ticks_per_cm * ref->rightvel / ref->frequency;

    if (current - ref->lastcommand < ros::Duration(50.5))
    {
      // write position into tics_left, tics_right
      tics_left = (int)(ref->absolute_rotations_left);
      tics_right = (int)(ref->absolute_rotations_right);
    }

    // that inversion also happened in EPOS controller
    t.left = -tics_left;
    t.right = -tics_right;

    t.vx = ref->vx;
    t.vth = ref->vth;

    ref->pub.publish(t);

    r.sleep();
  }
  return param;
}

void MotorDummy::init()
{
  printf("MotorDummy initialized\n");
  ROS_INFO("MotorDummy initialized");
  leftvel = 0.0;
  rightvel = 0.0;
  vx = 0;
  vth = 0;

  pub = n.advertise<volksbot::ticks>("VMC", 20);  // yeah, this is stupid...

  sub = n.subscribe("Vel", 100, &MotorDummy::Vcallback, this,
                    ros::TransportHints().reliable().udp().maxDatagramSize(100));

  cmd_vel_sub_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &MotorDummy::CVcallback, this,
                                                   ros::TransportHints().reliable().udp().maxDatagramSize(100));

  limit_sub = n.subscribe<volksbot::vel_limit>("vel_limit", 1, &MotorDummy::limitCallback, this);
  service = n.advertiseService("Controls", &MotorDummy::callback, this);
  pthread_create(&threadId, NULL, &MotorDummy::threadFunction, this);
}
