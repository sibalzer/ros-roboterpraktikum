#include "joystick.h"

Joystick::Joystick(const char* filename)
{
  init(filename);
}

Joystick::Joystick()
{
  init("/dev/input/js0");
}

void Joystick::init(const char* filename)
{
  fd = open(filename, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
  {
    ROS_ERROR("Joystick not found!");
    exit(1);
  }
}

Joystick::~Joystick()
{
  close(fd);
}

void Joystick::waitforevents()
{
  while (true)
  {
    if (read(fd, &je, 8) == 8)
    {
      if (je.type == BUTTON_TYPE)
      {
        handleButton(je.number, je.value == 1, je.time);
      }
      else if (je.type == AXIS_TYPE)
      {
        handleAxis(je.number, je.value, je.time);
      }
    }
  }
}

void Joystick::waitforevent()
{
  if (read(fd, &je, 8) == 8)
  {
    ROS_DEBUG("Time: %d ms; value: %i, type: 0x%x, number: %u", je.time, je.value, je.type, je.number);

    if (je.type == BUTTON_TYPE)
    {
      handleButton(je.number, je.value == 1, je.time);
    }
    else if (je.type == AXIS_TYPE)
    {
      handleAxis(je.number, je.value, je.time);
    }
  }
}
