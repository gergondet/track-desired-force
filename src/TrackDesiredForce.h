#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>

#include "TrackDesiredForceTask.h"
#include "api.h"

struct TrackDesiredForce_DLLAPI TrackDesiredForce : public mc_control::MCController
{
  TrackDesiredForce(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::shared_ptr<TrackDesiredForceTask> trackForceTask_;
  sva::ForceVecd targetWrench_ = {{0.0, 0.0, 0.0}, {0.0, 0.0, 300.0}};
};
