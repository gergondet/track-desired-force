#include "TrackDesiredForce.h"

TrackDesiredForce::TrackDesiredForce(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  mc_rtc::log::success("TrackDesiredForce init done ");
}

bool TrackDesiredForce::run()
{
  return mc_control::MCController::run();
}

void TrackDesiredForce::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("TrackDesiredForce", TrackDesiredForce)
