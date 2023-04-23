#pragma once

#include <mc_control/mc_controller.h>


#include "api.h"

struct TrackDesiredForce_DLLAPI TrackDesiredForce : public mc_control::MCController
{
  TrackDesiredForce(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
};