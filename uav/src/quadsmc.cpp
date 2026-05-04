//  created:    2025/03/02
//  filename:   quadsmc.cpp
//
//  author:     ateveraz
//
//  version:    $Id: 0.1$
//
//  purpose:    Custom control template
//
//
/*********************************************************************/

#include "quadsmc.h"
#include <Ahrs.h>
#include <AhrsData.h>
#include <CheckBox.h>
#include <ComboBox.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <DoubleSpinBox.h>
#include <FrameworkManager.h>
#include <GridLayout.h>
#include <GroupBox.h>
#include <Matrix.h>
#include <MetaVrpnObject.h>
#include <Object.h>
#include <Pid.h>
#include <PushButton.h>
#include <Tab.h>
#include <TargetController.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Uav.h>
#include <Vector3D.h>
#include <Vector3DSpinBox.h>
#include <VrpnClient.h>
#include <cmath>
// #include <>
#include <iostream>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

quadsmc::quadsmc(TargetController *controller)
    : UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default),
      vrpnLost(false), controlMode_t(ControlMode_t::Default) {
  Uav *uav = GetUav();

  VrpnClient *vrpnclient =
      new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(), 80,
                     uav->GetDefaultVrpnConnectionType());

  if (vrpnclient->ConnectionType() == VrpnClient::Xbee) {
    uavVrpn = new MetaVrpnObject(uav->ObjectName(), (uint8_t)0);
    targetVrpn = new MetaVrpnObject("target", 1);
  } else if (vrpnclient->ConnectionType() == VrpnClient::Vrpn) {
    uavVrpn = new MetaVrpnObject(uav->ObjectName());
    targetVrpn = new MetaVrpnObject("target");
  } else if (vrpnclient->ConnectionType() == VrpnClient::VrpnLite) {
    uavVrpn = new MetaVrpnObject(uav->ObjectName());
    targetVrpn = new MetaVrpnObject("target");
  }

  // set vrpn as failsafe altitude sensor for mamboedu as us in not working well
  // for the moment
  if (uav->GetType() == "mamboedu") {
    SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
  }

  getFrameworkManager()->AddDeviceToLog(uavVrpn);
  getFrameworkManager()->AddDeviceToLog(targetVrpn);
  vrpnclient->Start();

  uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),
                                      DataPlot::Green);

  startCircle = new PushButton(GetButtonsLayout()->NewRow(), "start_circle");
  stopCircle =
      new PushButton(GetButtonsLayout()->LastRowLastCol(), "stop_circle");
  positionHold =
      new PushButton(GetButtonsLayout()->LastRowLastCol(), "position hold");

  // Groupbox for control selection
  controlModeBox = new GroupBox(GetButtonsLayout()->NewRow(), "Control mode");
  on_customController = new PushButton(controlModeBox->NewRow(), "Activate");
  off_customController =
      new PushButton(controlModeBox->LastRowLastCol(), "Deactivate");
  control_selection =
      new ComboBox(controlModeBox->NewRow(), "Control selection");
  control_selection->AddItem("Default");
  control_selection->AddItem("Custom controller");

  // Custom tasks
  GroupBox *task_selection_box =
      new GroupBox(GetButtonsLayout()->LastRowLastCol(), "Custom task");
  task_selection = new ComboBox(task_selection_box->NewRow(), "Custom task");
  task_selection->AddItem("Hovering at zero");
  task_selection->AddItem("Regulation task");
  task_selection->AddItem("Circle tracking");
  task_selection->AddItem("Trajectory tracking");

  desired_position = new Vector3DSpinBox(task_selection_box->NewRow(),
                                         "Desired position", -3, 3, 0.1, 3);
  desired_yaw = new DoubleSpinBox(task_selection_box->LastRowLastCol(),
                                  "Desired yaw", -M_PI, M_PI, 0.1, 3);

  circle = new TrajectoryGenerator2DCircle(vrpnclient->GetLayout()->NewRow(),
                                           "circle");
  uavVrpn->xPlot()->AddCurve(circle->GetMatrix()->Element(0, 0),
                             DataPlot::Blue);
  uavVrpn->yPlot()->AddCurve(circle->GetMatrix()->Element(0, 1),
                             DataPlot::Blue);
  uavVrpn->VxPlot()->AddCurve(circle->GetMatrix()->Element(1, 0),
                              DataPlot::Blue);
  uavVrpn->VyPlot()->AddCurve(circle->GetMatrix()->Element(1, 1),
                              DataPlot::Blue);
  uavVrpn->XyPlot()->AddCurve(circle->GetMatrix()->Element(0, 1),
                              circle->GetMatrix()->Element(0, 0),
                              DataPlot::Blue, "circle");

  uX = new Pid(setupLawTab->At(1, 0), "u_x");
  uX->UseDefaultPlot(graphLawTab->NewRow());
  uY = new Pid(setupLawTab->At(1, 1), "u_y");
  uY->UseDefaultPlot(graphLawTab->LastRowLastCol());

  // Custom control law
  Tab *setup_custom_controller =
      new Tab(getFrameworkManager()->GetTabWidget(), "Custom controller");
  myCtrl = new MyController(setup_custom_controller->At(0, 0), "Parameters");

  Tab *setup_path_planner =
      new Tab(getFrameworkManager()->GetTabWidget(), "Path planner");
  myPlanner = new MyTrajectory(setup_path_planner->At(0, 0), "Parameters");

  customReferenceOrientation = new AhrsData(this, "reference");
  uav->GetAhrs()->AddPlot(customReferenceOrientation, DataPlot::Yellow);
  AddDataToControlLawLog(customReferenceOrientation);
  AddDeviceToControlLawLog(uX);
  AddDeviceToControlLawLog(uY);
  AddDeviceToControlLawLog(myCtrl);

  customOrientation = new AhrsData(this, "orientation");
}

quadsmc::~quadsmc() {}

void quadsmc::ComputeCustomTorques(Euler &torques) {
  // Implement your custom control law here or call a controller class.
  ComputeDefaultTorques(torques);
  thrust = ComputeDefaultThrust();

  // Selection of the control mode based on the control_selection combobox.
  switch (control_selection->CurrentIndex()) {
  case 1:
    controlMode_t = ControlMode_t::Custom;
    computeMyCtrl(torques);
    ComputeCustomThrust();
    break;

  default:
    controlMode_t = ControlMode_t::Default;
    Thread::Warn("quadsmc: default control law started. Check custom torque "
                 "definition. \n");
    EnterFailSafeMode();
    break;
  }
}

void quadsmc::computeMyCtrl(Euler &torques) {
  // Get position, velocity and quaternion from the VRPN object in its
  // coordinate system (OptiTrack)
  Vector3Df uav_pos, uav_vel;
  Quaternion vrpn_quaternion;
  uavVrpn->GetPosition(uav_pos);
  uavVrpn->GetSpeed(uav_vel);
  uavVrpn->GetQuaternion(vrpn_quaternion);

  // Get current orientation and angular speed from the AHRS object (IMU)
  const AhrsData *currentOrientation = GetDefaultOrientation();
  Quaternion currentQuaternion;
  Vector3Df currentAngularRates;
  currentOrientation->GetQuaternionAndAngularRates(currentQuaternion,
                                                   currentAngularRates);
  Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();

  // Use yaw from VRPN and roll, pitch from IMU
  Euler ahrsEuler = currentQuaternion.ToEuler();
  ahrsEuler.yaw = vrpn_quaternion.ToEuler().yaw;
  Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

  Vector3Df pos_err, vel_err, acc_des;

  // Compute the position and velocity errors in the UAV frame
  // Vector2Df pos_error2D, vel_error2D;
  // Vector3Df pos_error, vel_error, acc_des;
  // Example of desired altitude [m] => (ALWAYS A POSITIVE VALUE)
  // Because the AltitudeValues function returns a positive value also. However,
  // the UAV's altitude is negative in the VRPN coordinate system.
  float altittude_desired = desired_position->Value().z;
  float yaw_ref;
  float z, dz;
  AltitudeValues(z, dz);
  PositionValues(pos_err, vel_err, acc_des, yaw_ref);
  // Notice that the error definition is current - desired for x,y and z.
  // Vector3Df pos_error = Vector3Df(pos_error3D.x, pos_error3D.y,
  // pos_error3D.z); Vector3Df vel_error = Vector3Df(vel_error3D.x,
  // vel_error3D.y, vel_error3D.z); Vector3Df acc_desired = acc_des3D; Set the
  // values of the custom controller and update it
  std::cout << pos_err.x << pos_err.y << pos_err.z
            << " pos_err @ computeMyCtrl \n";

  myCtrl->SetValues(pos_err, vel_err, acc_des, mixQuaternion,
                    currentAngularSpeed, yaw_ref);
  myCtrl->Update(GetTime());

  // // Apply the computed torques and thrust
  torques.roll = myCtrl->Output(0);
  torques.pitch = myCtrl->Output(1);
  torques.yaw = myCtrl->Output(2);
  thrust = myCtrl->Output(3);

  // Just for testing
  // ComputeDefaultTorques(torques);
  // thrust = ComputeDefaultThrust();

  // If you prefer, you can also use the ComputeDefaultThrust() function. E.g.:
  // thrust = ComputeDefaultThrust();
  // The desired take-off altitude will be used as a reference.
}

float quadsmc::ComputeCustomThrust(void) {
  // Implement your custom thrust computation here or asign its value from
  // another function, because it is a global variable.
  if (thrust == 0) {
    // For safety reasons, the default thrust is computed if the custom thrust
    // is not defined.
    thrust = ComputeDefaultThrust();
    std::cout << "Custom thrust not defined, using default thrust: " << thrust
              << std::endl;
  }
  return thrust;
}

void quadsmc::StartCustomTorques(void) {
  if (control_selection->CurrentIndex() == 0) {
    StartDefaultTorques();
    Start_task();
    Thread::Info("quadsmc: default control law started\n");
  } else {
    if (SetTorqueMode(TorqueMode_t::Custom) &&
        SetThrustMode(ThrustMode_t::Custom) &&
        control_selection->CurrentIndex() != 0) {
      controlMode_t = ControlMode_t::Custom;
      myCtrl->Reset();
      Start_task();
      Thread::Info("quadsmc: custom control law started\n");
    } else {
      StopCustomTorques();
      Thread::Err("quadsmc: could not start custom control law\n");
    }
  }
}

void quadsmc::StopCustomTorques(void) {
  StartDefaultTorques();
  controlMode_t = ControlMode_t::Default;
  Thread::Info("quadsmc: custom control law stopped\n");
}

void quadsmc::StartDefaultTorques(void) {
  if (controlMode_t == ControlMode_t::Default) {
    Thread::Warn("quadsmc: already in default control law\n");
    return;
  }

  if (SetTorqueMode(TorqueMode_t::Default) &&
      SetThrustMode(ThrustMode_t::Default)) {
    controlMode_t = ControlMode_t::Default;
    Thread::Info("quadsmc: default control law started\n");
  } else {
    Thread::Err("quadsmc: could not start default control law\n");
  }
}

const AhrsData *quadsmc::GetOrientation(void) const {
  // get yaw from vrpn
  Quaternion vrpnQuaternion;
  uavVrpn->GetQuaternion(vrpnQuaternion);

  // get roll, pitch and w from imu
  Quaternion ahrsQuaternion;
  Vector3Df ahrsAngularSpeed;
  GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion,
                                                        ahrsAngularSpeed);

  // yaw from vrpn and roll, pitch from imu
  Euler ahrsEuler = ahrsQuaternion.ToEuler();
  ahrsEuler.yaw = vrpnQuaternion.ToEuler().yaw;
  Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

  customOrientation->SetQuaternionAndAngularRates(mixQuaternion,
                                                  ahrsAngularSpeed);

  return customOrientation;
}

void quadsmc::AltitudeValues(float &z, float &dz) const {
  Vector3Df uav_pos, uav_vel;

  uavVrpn->GetPosition(uav_pos);
  uavVrpn->GetSpeed(uav_vel);
  // z and dz must be in uav's frame
  z = -uav_pos.z;
  dz = -uav_vel.z;
}

AhrsData *quadsmc::GetReferenceOrientation(void) {
  Vector3Df pos_err;
  Vector3Df vel_err;
  Vector3Df acc_des; // in Uav coordinate system
  float yaw_ref;
  Euler refAngles;

  PositionValues(pos_err, vel_err, acc_des, yaw_ref);

  refAngles.yaw = yaw_ref;

  uX->SetValues(pos_err.x, vel_err.x);
  uX->Update(GetTime());
  refAngles.pitch = uX->Output();

  uY->SetValues(pos_err.y, vel_err.y);
  uY->Update(GetTime());
  refAngles.roll = -uY->Output();

  customReferenceOrientation->SetQuaternionAndAngularRates(
      refAngles.ToQuaternion(), Vector3Df(0, 0, 0));

  return customReferenceOrientation;
}

void quadsmc::PositionValues(Vector3Df &pos_error, Vector3Df &vel_error,
                             Vector3Df &acc_desired, float &yaw_ref) {
  Vector3Df uav_pos, uav_vel; // in VRPN coordinate system
  //   Vector2Df uav_2Dpos, uav_2Dvel; // in VRPN coordinate system

  std::cout << "behaviourMode: " << (int)behaviourMode << "\n";
  uavVrpn->GetPosition(uav_pos);
  uavVrpn->GetSpeed(uav_vel);

  // std::cout << uav_pos.x << uav_pos.y << uav_pos.z << " uav_pos \n";

  //   uav_pos.To2Dxy(uav_2Dpos);
  //   uav_vel.To2Dxy(uav_2Dvel);

  if (behaviourMode == BehaviourMode_t::PositionHold) {
    pos_error = uav_pos - posHold;
    vel_error = uav_vel;
    yaw_ref = yawHold;
  } else if (behaviourMode == BehaviourMode_t::Hover) {
    pos_error = uav_pos;
    vel_error = uav_vel;
    yaw_ref = 0;
  } else if (behaviourMode == BehaviourMode_t::Regulation) {

    Vector3Df desired_position_xyz =
        Vector3Df(desired_position->Value().x, desired_position->Value().y,
                  desired_position->Value().z);
    pos_error = uav_pos - desired_position_xyz;
    vel_error = uav_vel;
    yaw_ref = (float)desired_yaw->Value();
  } else if (behaviourMode == BehaviourMode_t::Trajectory) {
    myPlanner->SetValues(uav_pos);
    myPlanner->Update(GetTime());

    Vector3Df desired_position_xyz(myPlanner->Output(0), myPlanner->Output(1),
                                   myPlanner->Output(2));
    Vector3Df desired_velocity(myPlanner->Output(3), myPlanner->Output(4),
                               myPlanner->Output(5));
    pos_error = uav_pos - desired_position_xyz;
    vel_error = uav_vel - desired_velocity;
    acc_desired = Vector3Df(myPlanner->Output(6), myPlanner->Output(7),
                            myPlanner->Output(8));

    std::cout << pos_error.x << pos_error.y << pos_error.z
              << "pos_error @ quadsmc\n";

    yaw_ref = 0; // You can also define a desired yaw reference for the
                 // trajectory if needed.
  } else {       // Circle
    Vector3Df target_pos;
    Vector2Df circle_pos, circle_vel;
    Vector2Df target_2Dpos;

    targetVrpn->GetPosition(target_pos);
    target_pos.To2Dxy(target_2Dpos);
    circle->SetCenter(target_2Dpos);

    // circle reference
    circle->Update(GetTime());
    circle->GetPosition(circle_pos);
    circle->GetSpeed(circle_vel);

    // error in optitrack frame
    pos_error.x = uav_pos.x - circle_pos.x;
    pos_error.y = uav_pos.y - circle_pos.y;
    vel_error.x = uav_vel.x - circle_vel.x;
    vel_error.y = uav_vel.y - circle_vel.y;
    yaw_ref = atan2(target_pos.y - uav_pos.y, target_pos.x - uav_pos.x);
  }

  // error in uav frame
  Quaternion currentQuaternion = GetCurrentQuaternion();
  Euler currentAngles; // in vrpn frame
  currentQuaternion.ToEuler(currentAngles);
  pos_error.Rotate(-currentAngles.yaw);
  vel_error.Rotate(-currentAngles.yaw);
}

void quadsmc::SignalEvent(Event_t event) {
  UavStateMachine::SignalEvent(event);
  switch (event) {
  case Event_t::TakingOff:
    behaviourMode = BehaviourMode_t::Default;
    vrpnLost = false;
    break;
  case Event_t::EnteringControlLoop:
    if ((behaviourMode == BehaviourMode_t::Circle) && (!circle->IsRunning())) {
      VrpnPositionHold();
    }
    break;
  case Event_t::EnteringFailSafeMode:
    behaviourMode = BehaviourMode_t::Default;
    break;
  }
}

void quadsmc::ExtraSecurityCheck(void) {
  if ((!vrpnLost) && ((behaviourMode == BehaviourMode_t::Circle) ||
                      (behaviourMode == BehaviourMode_t::PositionHold))) {
    if (!targetVrpn->IsTracked(500)) {
      Thread::Err("VRPN, target lost\n");
      vrpnLost = true;
      EnterFailSafeMode();
      Land();
    }
    if (!uavVrpn->IsTracked(500)) {
      Thread::Err("VRPN, uav lost\n");
      vrpnLost = true;
      EnterFailSafeMode();
      Land();
    }
  }
  if ((!vrpnLost) && ((behaviourMode == BehaviourMode_t::Hover) ||
                      (behaviourMode == BehaviourMode_t::Regulation))) {
    if (!uavVrpn->IsTracked(500)) {
      Thread::Err("VRPN, uav lost\n");
      vrpnLost = true;
      EnterFailSafeMode();
      Land();
    }
  }
}

void quadsmc::ExtraCheckPushButton(void) {
  if (startCircle->Clicked()) {
    Start_task();
  }
  if (stopCircle->Clicked()) {
    StopCircle();
  }
  if (positionHold->Clicked()) {
    VrpnPositionHold();
  }
  if (on_customController->Clicked()) {
    StartCustomTorques();
  }
  if (off_customController->Clicked()) {
    StopCustomTorques();
  }
}

void quadsmc::ExtraCheckJoystick(void) {
  /*     Do not use cross, start nor select buttons!!
  0: "start"       1: "select"      2: "square"      3: "triangle"
  4: "circle"      5: "cross";      6: "left 1"      7: "left 2"
  8: "left 3"      9: "right 1"     10: "right 2"    11: "right 3"
  12: "up"         13: "down"       14: "left"       15: "right"
  */

  // R1 and Circle
  if (GetTargetController()->ButtonClicked(4) &&
      GetTargetController()->IsButtonPressed(9)) {
    Start_task();
  }

  // R1 and Cross
  if (GetTargetController()->ButtonClicked(5) &&
      GetTargetController()->IsButtonPressed(9)) {
    StopCircle();
  }

  // R1 and Square
  if (GetTargetController()->ButtonClicked(2) &&
      GetTargetController()->IsButtonPressed(9)) {
    VrpnPositionHold();
  }
}

void quadsmc::Start_task(void) {
  if (behaviourMode == BehaviourMode_t::Circle) {
    Thread::Warn("quadsmc: already in circle mode\n");
    return;
  }
  if (SetOrientationMode(OrientationMode_t::Custom)) {
    Thread::Info("quadsmc: start circle\n");
  } else {
    Thread::Warn("quadsmc: could not start circle\n");
    return;
  }

  // Defining desired task.
  if (task_selection->CurrentIndex() == 0) {
    behaviourMode = BehaviourMode_t::Hover;
    Thread::Info("quadsmc: hovering at zero\n");
  } else if (task_selection->CurrentIndex() == 1) {
    behaviourMode = BehaviourMode_t::Regulation;
    Thread::Info("quadsmc: regulation task\n");
  } else if (task_selection->CurrentIndex() == 2) {
    Vector3Df uav_pos, target_pos;
    Vector2Df uav_2Dpos, target_2Dpos;

    targetVrpn->GetPosition(target_pos);
    target_pos.To2Dxy(target_2Dpos);
    circle->SetCenter(target_2Dpos);

    uavVrpn->GetPosition(uav_pos);
    uav_pos.To2Dxy(uav_2Dpos);
    circle->StartTraj(uav_2Dpos);

    uX->Reset();
    uY->Reset();
    behaviourMode = BehaviourMode_t::Circle;
    Thread::Info("quadsmc: circle tracking\n");
  } else if (task_selection->CurrentIndex() == 3) {
    myPlanner->Reset();
    // uX->Reset();
    // uY->Reset();
    behaviourMode = BehaviourMode_t::Trajectory;
    Thread::Info("quadsmc: trajectory tracking\n");
  } else {
    Thread::Err("quadsmc: unknown task\n");
    return;
  }
}

void quadsmc::StopCircle(void) {
  if (behaviourMode != BehaviourMode_t::Circle) {
    Thread::Warn("quadsmc: not in circle mode\n");
    return;
  }
  circle->FinishTraj();
  // GetJoystick()->Rumble(0x70);
  Thread::Info("quadsmc: finishing circle\n");
}

void quadsmc::VrpnPositionHold(void) {
  if (behaviourMode == BehaviourMode_t::PositionHold) {
    Thread::Warn("quadsmc: already in vrpn position hold mode\n");
    return;
  }
  Quaternion vrpnQuaternion;
  uavVrpn->GetQuaternion(vrpnQuaternion);
  yawHold = vrpnQuaternion.ToEuler().yaw;

  Vector3Df vrpnPosition;
  uavVrpn->GetPosition(vrpnPosition);
  posHold = vrpnPosition;

  uX->Reset();
  uY->Reset();
  behaviourMode = BehaviourMode_t::PositionHold;
  SetOrientationMode(OrientationMode_t::Custom);
  Thread::Info("quadsmc: holding position\n");
}
