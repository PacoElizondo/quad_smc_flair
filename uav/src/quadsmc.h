//  created:    2025/03/02
//  filename:   quadsmc.h
//
//  author:     ateveraz
//
//  version:    $Id: 0.1$
//
//  purpose:    Custom control template
//
//
/*********************************************************************/

#ifndef quadsmc_H
#define quadsmc_H

#include "myCtrl.h"
#include "myTrajectory.h"
#include <UavStateMachine.h>
#include <Vector3D.h>

namespace flair {
namespace gui {
class PushButton;
class GroupBox;
class ComboBox;
class CheckBox;
class Vector3DSpinBox;
class DoubleSpinBox;
} // namespace gui
namespace filter {
class TrajectoryGenerator2DCircle;
class MyController;
class MyTrajectory;
} // namespace filter
namespace meta {
class MetaVrpnObject;
}
namespace sensor {
class TargetController;
}
} // namespace flair

class quadsmc : public flair::meta::UavStateMachine {
public:
  quadsmc(flair::sensor::TargetController *controller);
  ~quadsmc();

private:
  enum class BehaviourMode_t {
    Default,
    PositionHold,
    Circle,
    Regulation,
    Hover,
    Trajectory
  };

  enum class ControlMode_t { Default, Custom };

  BehaviourMode_t behaviourMode;
  ControlMode_t controlMode_t;
  bool vrpnLost;

  void VrpnPositionHold(void); // flight mode
  void Start_task(void);
  void StopCircle(void);
  void ExtraSecurityCheck(void) override;
  void ExtraCheckPushButton(void) override;
  void ExtraCheckJoystick(void) override;
  const flair::core::AhrsData *GetOrientation(void) const override;
  void AltitudeValues(float &z, float &dz) const override;
  void PositionValues(flair::core::Vector3Df &pos_error,
                      flair::core::Vector3Df &vel_error,
                      flair::core::Vector3Df &acc_desired, float &yaw_ref);
  flair::core::AhrsData *GetReferenceOrientation(void) override;
  void SignalEvent(Event_t event) override;
  void ComputeCustomTorques(flair::core::Euler &torques);
  float ComputeCustomThrust(void);
  void StartCustomTorques(void);
  void StopCustomTorques(void);
  void StartDefaultTorques(void);
  void computeMyCtrl(flair::core::Euler &torques);

  flair::filter::Pid *uX, *uY;
  flair::filter::MyController *myCtrl;
  flair::filter::MyTrajectory *myPlanner;

  flair::core::Vector3Df posHold;
  float yawHold;
  float thrust;

  flair::gui::PushButton *startCircle, *stopCircle, *positionHold;
  flair::meta::MetaVrpnObject *targetVrpn, *uavVrpn;
  flair::filter::TrajectoryGenerator2DCircle *circle;
  flair::core::AhrsData *customReferenceOrientation, *customOrientation;

  // Control mode GUI
  flair::gui::GroupBox *controlModeBox;
  flair::gui::ComboBox *control_selection;
  flair::gui::PushButton *on_customController, *off_customController;

  // Custom control law
  flair::gui::DoubleSpinBox *deltaT_custom;

  // Custom task
  flair::gui::ComboBox *task_selection;
  flair::gui::Vector3DSpinBox *desired_position;
  flair::gui::DoubleSpinBox *desired_yaw;
};

#endif // quadsmc_H
