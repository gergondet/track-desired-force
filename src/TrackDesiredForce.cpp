#include "TrackDesiredForce.h"

#include <mc_solver/TasksQPSolver.h>

TrackDesiredForce::TrackDesiredForce(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);

  mc_rtc::log::success("TrackDesiredForce init done ");
}

bool TrackDesiredForce::run()
{
  return mc_control::MCController::run();
}

void TrackDesiredForce::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  comTask_ = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 1000.0, 2.0);
  solver().addTask(comTask_);

  // Add LeftFoot|RightFoot contacts
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  // Trigger the contact update now so the contact is in the solver
  updateContacts();

  mc_rbdyn::Contact contact = [this]() {
    for(const auto & c : solver().contacts())
    {
      if(c.r1Surface()->name() == "RightFoot")
      {
        return c;
      }
    }
    mc_rtc::log::error_and_throw("Unreachable");
  }();
  auto cid = contact.contactId(robots());

  const auto & tsolver = tasks_solver(solver());
  trackForceTask_ = std::make_shared<TrackDesiredForceTask>(solver(), cid);
  trackForceTask_->setTargetWrench(targetWrench_);
  tasks_solver(solver()).addTask(trackForceTask_.get());

  gui()->addElement(
      {},
      mc_rtc::gui::NumberInput(
          "Weight", [this]() { return trackForceTask_->weight(); }, [this](double w) { trackForceTask_->weight(w); }),
      mc_rtc::gui::ArrayInput(
          "Target wrench", [this]() { return targetWrench_; },
          [this](const sva::ForceVecd & fv) {
            targetWrench_ = fv;
            trackForceTask_->setTargetWrench(fv);
          }),
      mc_rtc::gui::ArrayLabel("QP wrench", [this, contact]() { return solver().desiredContactForce(contact); }));
}

CONTROLLER_CONSTRUCTOR("TrackDesiredForce", TrackDesiredForce)
