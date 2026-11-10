#include <Definitions.h>

#include <cstdio>
#include <cstdlib>

namespace
{
int device;
int subdevice;
bool enable_attempted[128]{};

void trace(const char * operation, unsigned int id, long value = 0, long extra = 0)
{
  const char * path = std::getenv("EPOS_TEST_TRACE");
  if (path) {
    if (FILE * file = std::fopen(path, "a")) {
      std::fprintf(file, "%s %u %ld %ld\n", operation, id, value, extra);
      std::fclose(file);
    }
  }
}
}

void * VCS_OpenDevice(char *, char *, char *, char *, unsigned int *)
{
  trace("open", 0);
  return &device;
}
int VCS_SetProtocolStackSettings(void *, unsigned int, unsigned int, unsigned int *) {return 1;}
void * VCS_OpenSubDevice(void *, char *, char *, unsigned int *)
{
  trace("open_subdevice", 0);
  return &subdevice;
}
int VCS_GetGatewaySettings(void * handle, unsigned int * baud, unsigned int * error)
{
  if (handle != &subdevice) {
    *error = 0x5678;
    return 0;
  }
  const char * configured = std::getenv("EPOS_TEST_CAN_BAUD");
  *baud = configured ? std::strtoul(configured, nullptr, 10) : 1000000;
  return 1;
}
int VCS_CloseSubDevice(void *, unsigned int *) {return 1;}
int VCS_CloseDevice(void *, unsigned int *) {return 1;}
int VCS_SetDisableState(void * handle, unsigned short id, unsigned int * error)
{
  trace("disable", id);
  const char * gateway = std::getenv("EPOS_TEST_GATEWAY_ID");
  const unsigned long gateway_id = gateway ? std::strtoul(gateway, nullptr, 10) : 4;
  if (handle != (id == gateway_id ? &device : &subdevice)) {
    *error = 0x5678;
    return 0;
  }
  const char * failed = std::getenv("EPOS_TEST_FAIL_NODE");
  const char * fail_after_enable = std::getenv("EPOS_TEST_FAIL_DISABLE_AFTER_ENABLE");
  if ((failed && std::strtoul(failed, nullptr, 10) == id) ||
    (fail_after_enable && std::strtoul(fail_after_enable, nullptr, 10) == id &&
    enable_attempted[id]))
  {
    *error = 0x1234;
    return 0;
  }
  return 1;
}
int VCS_ClearFault(void *, unsigned short id, unsigned int *) {trace("clear", id); return 1;}
int VCS_ActivateProfileVelocityMode(void *, unsigned short id, unsigned int *)
{trace("velocity_mode", id); return 1;}
int VCS_ActivateProfilePositionMode(void *, unsigned short id, unsigned int *)
{trace("position_mode", id); return 1;}
int VCS_SetMaxProfileVelocity(void *, unsigned short id, unsigned int value, unsigned int *)
{trace("max_velocity", id, value); return 1;}
int VCS_SetVelocityProfile(
  void *, unsigned short id, unsigned int acceleration, unsigned int deceleration, unsigned int *)
{trace("velocity_profile", id, acceleration, deceleration); return 1;}
int VCS_SetPositionProfile(
  void *, unsigned short id, unsigned int velocity, unsigned int acceleration,
  unsigned int deceleration, unsigned int *)
{
  trace("position_profile", id, velocity, acceleration);
  trace("position_deceleration", id, deceleration);
  return 1;
}
int VCS_SetEnableState(void *, unsigned short id, unsigned int * error)
{
  trace("enable", id);
  enable_attempted[id] = true;
  const char * failed = std::getenv("EPOS_TEST_FAIL_ENABLE_NODE");
  if (failed && std::strtoul(failed, nullptr, 10) == id) {
    *error = 0x4321;
    return 0;
  }
  return 1;
}
int VCS_MoveWithVelocity(void *, unsigned short id, long value, unsigned int *)
{trace("move_velocity", id, value); return 1;}
int VCS_MoveToPosition(
  void *, unsigned short id, long value, int absolute, int immediately, unsigned int *)
{trace("move_position", id, value, absolute * 10 + immediately); return 1;}
int VCS_HaltVelocityMovement(void *, unsigned short id, unsigned int *)
{trace("halt_velocity", id); return 1;}
int VCS_HaltPositionMovement(void *, unsigned short id, unsigned int *)
{trace("halt_position", id); return 1;}
