#ifndef POSVELACCEFF_COMMAND_INTERFACE_POSVELACCEFF_COMMAND_INTERFACE_HPP
#define POSVELACCEFF_COMMAND_INTERFACE_POSVELACCEFF_COMMAND_INTERFACE_HPP

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/hardware_interface.h>

namespace posvelacceff_command_interface
{
class PosVelAccEffJointHandle : public hardware_interface::PosVelAccJointHandle
{
public:
  PosVelAccEffJointHandle() : hardware_interface::PosVelAccJointHandle(), cmd_eff_(NULL)
  {
  }

  PosVelAccEffJointHandle(const JointStateHandle& js, double* cmd_pos, double* cmd_vel, double* cmd_acc,
                          double* cmd_eff)
    : PosVelAccJointHandle(js, cmd_pos, cmd_vel, cmd_acc), cmd_eff_(cmd_eff)
  {
    if (!cmd_eff)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Command effort data pointer is null.");
    }
  }

  void setCommand(double cmd_pos, double cmd_vel, double cmd_acc, double cmd_eff)
  {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandAcceleration(cmd_acc);
    setCommandEffort(cmd_eff);
  }

  void setCommandEffort(double cmd_eff)
  {
    assert(cmd_eff_);
    *cmd_eff_ = cmd_eff;
  }
  double getCommandEffort() const
  {
    assert(cmd_eff_);
    return *cmd_eff_;
  }

private:
  double* cmd_eff_;
};

class PosVelAccEffJointInterface
  : public hardware_interface::HardwareResourceManager<PosVelAccEffJointHandle, hardware_interface::ClaimResources>
{
};

}  // namespace posvelacceff_command_interface

#endif
