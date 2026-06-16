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
#include <io_data.h>
#include <iostream>
#include <regex>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

MyTrajectory::MyTrajectory(const LayoutPosition *position, const string &name)
    : ControlLaw(position->getLayout(), name, 10), first_update(true),
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
                                    "delta_t", 0, 0.01, 0.001, 4, 1.0);
  amplitude = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                                "Amplitude", 0, 10, 0.01, 4, 1.0);
  height = new DoubleSpinBox(general_parameters->NewRow(), "Height", -2, -0.1,
                             0.001, 4, -1.5);
  speed = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Speed", 0, 3,
                             0.001, 4, 1);

  GroupBox *traj_selection_box =
    new GroupBox(general_parameters->NewRow(), "Custom trajectory");
  traj_selection = new ComboBox(traj_selection_box->NewRow(), "Custom trajectory");
  traj_selection->AddItem("Straight line");
  traj_selection->AddItem("Circle");
  // traj_selection->AddItem("Circle tracking");
  // traj_selection->AddItem("Trajectory tracking");
  // ADD trajectory options spinbox
  traj_map_ = {
        {0, [this](const TrajectoryContext& ctx) { return ComputeStraightLine(ctx); }},
        {1, [this](const TrajectoryContext& ctx) { return ComputeCircle(ctx); }},
  };


  // Show cartesian errors plot
  plotCartesianErrors(gui_quadsmc->NewRow());

  AddDataToLog(state);
}

MyTrajectory::~MyTrajectory() { delete state; }

void MyTrajectory::UpdateFrom(const io_data *data) {
  if (first_update) {
    
    // initial_time = 0.0F;
    pos_initial =
    Vector3Df(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    
    first_update = false;
  }

  bool calibration = false;
  double double_time = GetTime() / 1e9 - initial_time ;
  float current_time = float(double_time);

  
  auto amplitude_value = (float)amplitude->Value();
  auto height_value = (float)height->Value();
  auto speed_value = (float)speed->Value();
  
  double double_delta = data->DataDeltaTime() / 1e9;
  float dt = float(double_delta);

  Vector3Df desired_position;
  Vector3Df desired_velocity;
  Vector3Df desired_acceleration;
  float desired_heading;
  float ramp = 0.0F;

  ramp = std::fmin(current_time / 5.0F, 1.0F);
  float m = std::fmin(current_time * speed_value, amplitude_value);
  float m_delta = (m - m_prev) / dt;
  


  m_prev = m;

  TrajectoryContext ctx;
  ctx.current_time = current_time;
  ctx.amplitude    = amplitude_value;
  ctx.speed        = speed_value;
  ctx.height       = height_value;
  ctx.ramp         = ramp;
  ctx.pos_initial  = pos_initial;

  auto it = traj_map_.find(traj_selection->CurrentIndex());
  if (it == traj_map_.end()) return;

  TrajectoryOutput desired = it->second(ctx);

       
  // Send desired position
  output->SetValue(0, 0, desired.position.x);
  output->SetValue(1, 0, desired.position.y);
  output->SetValue(2, 0, desired.position.z);
  // Send desired velocity
  output->SetValue(3, 0, desired.velocity.x);
  output->SetValue(4, 0, desired.velocity.y);
  output->SetValue(5, 0, desired.velocity.z);
  // Send desired acceleration
  output->SetValue(6, 0, desired.acceleration.x);
  output->SetValue(7, 0, desired.acceleration.y);
  output->SetValue(8, 0, desired.acceleration.z);
  output->SetValue(9, 0, desired.heading);
  // Send data time for logging
  output->SetDataTime(data->DataTime());

  // Log state (duplicated from the 0-3 outputs).
  state->GetMutex();
  state->SetValue(0, 0, desired.position.x);
  state->SetValue(1, 0, desired.position.y);
  state->SetValue(2, 0, desired.position.z);

  state->SetValue(3, 0, desired.velocity.x);
  state->SetValue(4, 0, desired.velocity.y);
  state->SetValue(5, 0, desired.velocity.z);
  // Send desired acceleration
  state->SetValue(6, 0, desired.acceleration.x);
  state->SetValue(7, 0, desired.acceleration.y);
  state->SetValue(8, 0, desired.acceleration.z);
  state->ReleaseMutex();

  ProcessUpdate(output);
}


MyTrajectory::TrajectoryOutput MyTrajectory::ComputeStraightLine(const TrajectoryContext& ctx) {
    TrajectoryOutput out;
    float t = ctx.current_time;
    float a = ctx.amplitude;
    float ch = coshf(t - 3.0F);
    float th = tanhf(t - 3.0F);

    out.position.x = a * (th + 1.0F);
    out.position.y = 0;
    out.position.z = a * (th - 1.0F) - 0.1F;

    out.velocity.x = a / (ch * ch);
    out.velocity.y = 0;
    out.velocity.z = out.velocity.x;

    out.acceleration.x = (-2.0F * a * th) / (ch * ch);
    out.acceleration.y = 0;
    out.acceleration.z = out.acceleration.x;

    out.heading = 0.0F;
    return out;
}

MyTrajectory::TrajectoryOutput MyTrajectory::ComputeCircle(const TrajectoryContext& ctx) {
    TrajectoryOutput out;
    float t = ctx.current_time;
    float a = ctx.amplitude;
    float s = ctx.speed;
    float r = ctx.ramp;

    out.position.x = (r * a * std::sin(t * s)) + ctx.pos_initial.x;
    out.position.y = (r * a * std::cos(t * s)) + ctx.pos_initial.y;
    out.position.z = std::fmin(ctx.height, -0.1F);

    out.velocity.x =  r * a * std::cos(t * s);
    out.velocity.y = -r * a * std::sin(t * s);
    out.velocity.z = 0.0F;

    out.acceleration.x = -r * a * std::sin(t * s);
    out.acceleration.y = -r * a * std::cos(t * s);
    out.acceleration.z = 0.0F;

    out.heading = atan2f(out.position.y - ctx.pos_initial.y,
                         out.position.x - ctx.pos_initial.x) - 1.57F;
    return out;
}



void MyTrajectory::Reset(void) { first_update = true; initial_time = GetTime() / 1e9; m_prev = 0; }

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

// void MyTrajectory::Update(Time time){
  
  
// }