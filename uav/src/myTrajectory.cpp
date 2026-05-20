#include "myTrajectory.h"
#include <CheckBox.h>
#include <DataPlot1D.h>
#include <DoubleSpinBox.h>
#include <Euler.h>
#include <GroupBox.h>
#include <Label.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <Matrix.h>
#include <Pid.h>
#include <Quaternion.h>
#include <TabWidget.h>
#include <Vector3D.h>
#include <Vector3DSpinBox.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <regex>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

MyTrajectory::MyTrajectory(const LayoutPosition *position, const string &name)
    : ControlLaw(position->getLayout(), name, 9), first_update(true),
      delta_t(0.001F), initial_time(0.0F) {
  // Input matrix
  input = new Matrix(this, 3, 1, floatType, name);

  // Matrix descriptor for logging. It should be always a nx1 matrix.
  auto *log_labels = new MatrixDescriptor(9, 1);
  log_labels->SetElementName(0, 0, "desired_x");
  log_labels->SetElementName(1, 0, "desired_y");
  log_labels->SetElementName(2, 0, "desired_z");
  log_labels->SetElementName(0, 0, "desired_vel_x");
  log_labels->SetElementName(1, 0, "desired_vel_y");
  log_labels->SetElementName(2, 0, "desired_vel_z");
  log_labels->SetElementName(0, 0, "desired_acc_x");
  log_labels->SetElementName(1, 0, "desired_acc_y");
  log_labels->SetElementName(2, 0, "desired_acc_z");
  state = new Matrix(this, log_labels, floatType, name);
  delete log_labels;

  // GUI for path planning
  auto *gui_quadsmc = new GroupBox(position, name);
  auto *general_parameters = new GroupBox(gui_quadsmc->NewRow(), " ");
  deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(),
                                    "Custom dt [s]", 0, 0.01, 0.001, 4, 0.001);
  amplitude = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                                "Amplitude", 0, 10, 0.01, 4, 1.0);
  z_rate = new DoubleSpinBox(general_parameters->NewRow(), "Z rate", -0.05, 0.05,
                             0.001, 4, 0.0);
  xy_rate = new DoubleSpinBox(general_parameters->NewRow(), "XY rate", -2, 2,
                              0.001, 4, 0.0);

  // Show cartesian errors plot
  plotCartesianErrors(gui_quadsmc->NewRow());

  AddDataToLog(state);
}

MyTrajectory::~MyTrajectory() { delete state; }

void MyTrajectory::UpdateFrom(const io_data *data) {
  if (first_update) {

    initial_time = float(GetTime()) / 1000000000.0F;

    first_update = false;
  }

  bool calibration = false;
  // float current_time = (float(GetTime()) / 1000000000.0F);

  auto amplitude_value = (float)amplitude->Value();
  auto z_rate_value = (float)z_rate->Value();
  auto xy_rate_value = (float)xy_rate->Value();

  if (deltaT_custom->Value() == 0) {
    delta_t = (float)(data->DataDeltaTime()) / 1000000000.0F;
  } else {
    delta_t = (float)deltaT_custom->Value();
  }

  // current_time = current_time + delta_t;
  // std::cout << current_time << "\n";
  if (current_time < 0.1 && !calibration){
    input->GetMutex();
    pos_initial =
    Vector3Df(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    input->ReleaseMutex();
    
    // std::cout << current_time << "current_time @ myTraj \n";
  } else { calibration = true;  }

  current_time = current_time + delta_t;
  Vector3Df desired_position;
  Vector3Df desired_velocity;
  Vector3Df desired_acceleration;
  float ramp = 0.0F;

  ramp = std::fmin(current_time / 5.0F, 1.0F);
  // std::cout << ramp << "ramp \n";

  desired_position.x =
      (ramp * amplitude_value * (std::sin(current_time * xy_rate_value))/xy_rate_value)+ pos_initial.x;
  desired_position.y =
      (ramp * amplitude_value * (std::cos(current_time * xy_rate_value))/xy_rate_value) + pos_initial.y;
  desired_position.z = (current_time * z_rate_value) - 1.5F;
  // std::cout << current_time;
  // std::cout << desired_position.x << "desired p x @ traj \n";

  desired_velocity.x = (ramp * amplitude_value  *  
                        std::cos(current_time * xy_rate_value));
  desired_velocity.y = (-ramp * amplitude_value *  
                        std::sin(current_time * xy_rate_value));
  desired_velocity.z = (z_rate_value);

  desired_acceleration.x =
      (-ramp * amplitude_value *
       std::sin(current_time * xy_rate_value));
  desired_acceleration.y =
      (-ramp * amplitude_value *
       std::cos(current_time * xy_rate_value));
  desired_acceleration.z = (0.0F);

  // std::cout << pos_initial.z << "pos_initial.z @ traj\n";


  // desired_position.x = pos_initial.x;
  // desired_position.y = pos_initial.y;
  // desired_position.z = 0;

  // // std::cout << pos_initial.y << " posinitialy";

  // desired_velocity.x = 0.0F;
  // desired_velocity.y = 0.0F;
  // desired_velocity.z = 0.0F;

  // desired_acceleration.x = 0.0F;
  // desired_acceleration.y = 0.0F;
  // desired_acceleration.z = 0.0F;

  // Send desired position
  output->SetValue(0, 0, desired_position.x);
  output->SetValue(1, 0, desired_position.y);
  output->SetValue(2, 0, desired_position.z);
  // Send desired velocity
  output->SetValue(3, 0, desired_velocity.x);
  output->SetValue(4, 0, desired_velocity.y);
  output->SetValue(5, 0, desired_velocity.z);
  // Send desired acceleration
  output->SetValue(6, 0, desired_acceleration.x);
  output->SetValue(7, 0, desired_acceleration.y);
  output->SetValue(8, 0, desired_acceleration.z);
  // Send data time for logging
  output->SetDataTime(data->DataTime());

  // Log state (duplicated from the 0-3 outputs).
  // state->GetMutex();
  // state->SetValue(0, 0, desired_position.x);
  // state->SetValue(1, 0, desired_position.y);
  // state->SetValue(2, 0, desired_position.z);

  // state->SetValue(3, 0, desired_velocity.x);
  // state->SetValue(4, 0, desired_velocity.y);
  // state->SetValue(5, 0, desired_velocity.z);
  // // Send desired acceleration
  // state->SetValue(6, 0, desired_acceleration.x);
  // state->SetValue(7, 0, desired_acceleration.y);
  // state->SetValue(8, 0, desired_acceleration.z);
  // state->ReleaseMutex();

  ProcessUpdate(output);
}

void MyTrajectory::Reset(void) { first_update = true; }

void MyTrajectory::SetValues(const Vector3Df &Pos_0) {
  // Set the input values for the path planner. Now it only receives a misc
  // variable that can be used to set any value you want. This function is
  // called from the main controller to set the input values.
  input->GetMutex();
  input->SetValue(0, 0, Pos_0.x);
  input->SetValue(1, 0, Pos_0.y);
  input->SetValue(2, 0, Pos_0.z);
  input->ReleaseMutex();
}

void MyTrajectory::plotCartesianErrors(const LayoutPosition *position) {
  // Example of how to plot the desired position in the GUI.
  // Any variable that is defined in the state matrix can be plotted. Just
  // remember to set its value in the UpdateFrom function and to add it to the
  // log_labels matrix in the constructor.
  auto *plot = new DataPlot1D(position, "Desired cartesian position", -3, 3);
  plot->AddCurve(output->Element(0), DataPlot::Red);   // desired x
  plot->AddCurve(output->Element(1), DataPlot::Black); // desired y
  plot->AddCurve(output->Element(2), DataPlot::Blue);  // desired z
}