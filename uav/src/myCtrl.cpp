// Custom SMC Controller - Francisco Elizondo-Coronado

#include "myCtrl.h"
#include <CheckBox.h>
#include <DataPlot.h>
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
#include <Tab.h>
#include <Vector3D.h>
#include <Vector3DSpinBox.h>
#include <cmath>
#include <FrameworkManager.h>
#include <cstdlib>
#include <iostream>


using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

MyController::MyController(const LayoutPosition *position, const string &name)
    : ControlLaw(position->getLayout(), name, 4), first_update(true),
      delta_t(0.001F), initial_time(0.0F), current_time(0.0F) {
  // Input matrix
  input = new Matrix(this, 4, 6, floatType, name);

  // Matrix descriptor for logging. It should be always a nx1 matrix.
  auto *log_labels = new MatrixDescriptor(11, 1);
  log_labels->SetElementName(0, 0, "x_e");
  log_labels->SetElementName(1, 0, "y_e");
  log_labels->SetElementName(2, 0, "z_e");
  log_labels->SetElementName(3, 0, "q0_e");
  log_labels->SetElementName(4, 0, "pitch_e");
  log_labels->SetElementName(5, 0, "roll_e");
  log_labels->SetElementName(6, 0, "yaw_e");
  log_labels->SetElementName(7, 0, "tau_x"); 
  log_labels->SetElementName(8, 0, "tau_y"); 
  log_labels->SetElementName(9, 0, "tau_z"); 
  log_labels->SetElementName(10, 0,"thrust");

  state = new Matrix(this, log_labels, floatType, name);
  delete log_labels;

  

  // GUI for custom controller
  auto *gui_quadsmc = new GroupBox(position, name);
  auto *general_parameters = new GroupBox(gui_quadsmc->NewRow(), " ");
  deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(),
                                    "Custom dt [s]", 0, 1, 0.001, 4);
  mass = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Mass [kg]", 0,
                           10, 0.001, 4, 0.429);
  k_motor = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                              "Motor constant", 0, 50, 0.01, 4, 11.6);
  sat_thrust = new DoubleSpinBox(general_parameters->NewRow(),
                                 "Saturation thrust", 0, 10, 0.01, 3, 1.0);
  sat_pos = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                              "Saturation pos", 0, 10, 0.01, 3, 0.8);
  sat_att = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                              "Saturation att", 0, 10, 0.01, 3, 0.8);

  // Custom cartesian position controller
  auto *custom_position =
      new GroupBox(gui_quadsmc->NewRow(), "Custom SMC position controller");
  K_pos = new Vector3DSpinBox(custom_position->LastRowLastCol(), "K_pos", 0,
                              100, 0.1, 3, Vector3Df(4.0F, 4.0F, 4.0F));
  Lambda_pos =
      new Vector3DSpinBox(custom_position->LastRowLastCol(), "Lambda_pos", 0,
                          100, 0.1, 3, Vector3Df(3.0F, 3.0F, 3.0F));
  // Initial cartesian sliding surface compensator
  K_surf_pos_t0 =
      new Vector3DSpinBox(custom_position->LastRowLastCol(), "K_surf_pos_t0", 0,
                          100, 0.1, 3, Vector3Df(0.0F, 0.0F, 0.0F));

  // Custom attitude controller
  auto *custom_attitude =
      new GroupBox(gui_quadsmc->NewRow(), "Custom attitude controller");
  K_att = new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "K_att", 0,
                              1000, 0.1, 3, Vector3Df(2.0F, 2.0F, 2.0F));
  Lambda_att =
      new Vector3DSpinBox(custom_attitude->LastRowLastCol(), "Lambda_att", 0,
                          100, 0.1, 3, Vector3Df(10.0F, 10.0F, 10.0F));

  
  // Show cartesian errors plot
  Tab *plot_tab =
      new Tab(getFrameworkManager()->GetTabWidget(), "Plots");
  
  plotControllerData(plot_tab->LastRowLastCol());
  
  
  AddDataToLog(state);
}

MyController::~MyController() { delete state; }

void MyController::UpdateFrom(const io_data *data) {
  current_time = current_time + delta_t;
  Vector3Df u_position;
  Vector3Df tau;
  
  auto mass_val = (float)mass->Value();
  const float gravity = 9.81;  
  const Vector3Df J_diag = Vector3Df(0.00209,  0.002102, 0.00406);

  if (first_update) {
    
    initial_time = float(GetTime()) / 1000000000.0F;
    first_update = false;
    q_desired = Quaternion(1.0F, 0.0F, 0.0F, 0.0F);
    q_desired_prev = Quaternion(1.0F, 0.0F, 0.0F, 0.0F);
    omega_desired_prev = Vector3Df(0.0F, 0.0F, 0.0F);
    float thrust_curr = g;

    input->GetMutex();
    pos_error_0 =
        Vector3Df(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    vel_error_0 =
        Vector3Df(input->Value(0, 1), input->Value(1, 1), input->Value(2, 1));
    Surface_pos_t0 = vel_error_0 + Lambda_pos->Value() * pos_error_0;
    input->ReleaseMutex();
  }

  float thrust = thrust_curr;
  float thrust_curr_norm = fabsf(thrust_curr);

  // Obtain state
  input->GetMutex();
  Vector3Df pos_error(input->Value(0, 0), input->Value(1, 0),
                      input->Value(2, 0));
  Vector3Df vel_error(input->Value(0, 1), input->Value(1, 1),
                      input->Value(2, 1));
  Vector3Df acc_desired(input->Value(0, 2), input->Value(1, 2),
                        input->Value(2, 2));
  Quaternion quat(input->Value(0, 3), input->Value(1, 3), input->Value(2, 3),
                  input->Value(3, 3));
  Vector3Df omega(input->Value(0, 4), input->Value(1, 4), input->Value(2, 4));
  float yaw_ref = input->Value(0, 5);
  input->ReleaseMutex();

  // Get tunning parameters from GUI
  Vector3Df K_pos_val(K_pos->Value().x, K_pos->Value().y, K_pos->Value().z);
  Vector3Df Lambda_pos_val(Lambda_pos->Value().x, Lambda_pos->Value().y,
                           Lambda_pos->Value().z);
  Vector3Df K_att_val(K_att->Value().x, K_att->Value().y, K_att->Value().z);
  Vector3Df Lambda_att_val(Lambda_att->Value().x, Lambda_att->Value().y,
                           Lambda_att->Value().z);
  Vector3Df K_surf_pos_t0_val(K_surf_pos_t0->Value().x,
                              K_surf_pos_t0->Value().y,
                              K_surf_pos_t0->Value().z);

  // Cartesian custom controller self.s_p_t0*np.exp(-self.k_t0*self.t)
  Vector3Df surface_pos = Vector3Df(
      vel_error.x + (Lambda_pos_val.x * pos_error.x),
      vel_error.y + (Lambda_pos_val.y * pos_error.y),
      vel_error.z + (Lambda_pos_val.z * pos_error.z) 
      );
  
  Vector3Df surface_pos_dot =
      Vector3Df(-(mass_val * Lambda_pos_val.x * vel_error.x) -
                    (mass_val * K_pos_val.x * 
                     tanhf(surface_pos.x)) + acc_desired.x,
                    //  -sqrtf(fabsf(surface_pos.x)) * tanhf(surface_pos.x)
                -(mass_val * Lambda_pos_val.y * vel_error.y) -
                    (mass_val * K_pos_val.y * 
                     tanhf(surface_pos.y)) + acc_desired.y, 
                    //  - sqrtf(fabsf(surface_pos.y)) * tanhf(surface_pos.y)
                -(mass_val * Lambda_pos_val.z * vel_error.z) -
                    (mass_val * K_pos_val.z * 
                     tanhf(surface_pos.z)) - (mass_val*g) + acc_desired.z
                    //  - sqrtf(fabsf(surface_pos.z)) * tanhf(surface_pos.z)
      );

  float thrust_norm = sqrtf(DotProduct(surface_pos_dot, surface_pos_dot));
  Vector3Df temp = CrossProduct(Vector3Df(0.0F, 0.0F, -1.0F), surface_pos_dot);
  float temp_norm = sqrtf(DotProduct(temp, temp));
  

  Quaternion thrust_q =
      Quaternion(0.0F, surface_pos_dot.x, surface_pos_dot.y, surface_pos_dot.z);
  q_desired = Quaternion(-thrust_q.q3 + thrust_norm, temp.x, temp.y, temp.z);
  q_desired.Normalize();
    
  Quaternion body_z_world =
      q_desired * Quaternion(0.0F, 0.0F, 0.0F, 1.0F) * q_desired.GetConjugate();

  u_position.x = surface_pos_dot.x;
  u_position.y = surface_pos_dot.y;
  u_position.z = surface_pos_dot.z;
  
  float ctrl_z = DotProduct(u_position, Vector3Df(body_z_world.q1, body_z_world.q2, body_z_world.q3));
  u_position.Saturate((float)sat_pos->Value());
  

  Quaternion q_heading = Quaternion(cosf(yaw_ref/2),0.0,0.0,sinf(yaw_ref/2));
  q_desired = q_desired*q_heading;

  // Check for shortest rotation ( see quaternion double cover )
  float dot = q_desired.q0 * quat.q0
            + q_desired.q1 * quat.q1
            + q_desired.q2 * quat.q2
            + q_desired.q3 * quat.q3;
  if (dot < 0.0F) {
      q_desired = -q_desired;  
  }
  
  Quaternion q_error = (q_desired.GetConjugate() * quat);
  q_error.Normalize();
  Vector3Df att_error = 2 * Vector3Df(q_error.q1,q_error.q2, q_error.q3);
  
  Quaternion q_desired_dot;

  // omega desired is compensated by the controller
  omega_desired = Vector3Df(0.0,0.0,0.0);
  Vector3Df omega_error = omega - omega_desired;

  Vector3Df surface_att = omega_error + Lambda_att_val * att_error;

  Vector3Df u_att_sw = Vector3Df(-K_att_val.x * tanhf(surface_att.x),
                                 -K_att_val.y * tanhf(surface_att.y),
                                 -K_att_val.z * tanhf(surface_att.z));

Vector3Df surface_att_dot =
  CrossProduct(omega, (J_diag * omega)) +
  (- u_att_sw);
  
  tau =Vector3Df( surface_att_dot.x, surface_att_dot.y,surface_att_dot.z) ;

  q_desired_prev = q_desired;
  
  applyMotorConstant(tau);
  tau.Saturate((float)sat_att->Value());
  
  // Compute custom thrust
  thrust = ctrl_z ; // This is the thrust needed to counteract gravity and control the z position
  thrust_curr = thrust; //store in global variable

  applyMotorConstant(thrust);
  if(thrust < -sat_thrust->Value())
  {
      thrust = -(float)sat_thrust->Value();
  }
  else if(thrust >= 0)
  {
      thrust = 0; 
  }

  // Send controller output
  output->SetValue(0, 0, tau.x);
  output->SetValue(1, 0, tau.y);
  output->SetValue(2, 0, tau.z);
  output->SetValue(3, 0, thrust);
  output->SetDataTime(data->DataTime());

  // Log state (example).
  // Modify the log_labels matrix in the constructor to add more variables.
  state->GetMutex();
  state->SetValue(0, 0, pos_error.x);
  state->SetValue(1, 0, pos_error.y);
  state->SetValue(2, 0, pos_error.z);
  state->SetValue(3, 0, q_error.q0);
  // state->SetValue(4, 0, q_error.q1);
  // state->SetValue(5, 0, q_error.q2);
  // state->SetValue(6, 0, q_error.q3);
  state->SetValue(4, 0, att_error.x);
  state->SetValue(5, 0, att_error.y);
  state->SetValue(6, 0, att_error.z);
  state->SetValue(7, 0, tau.x);
  state->SetValue(8, 0, tau.y);
  state->SetValue(9, 0, tau.z);
  state->SetValue(10, 0, thrust);
  //   state->SetValue(2, 0, rpy.YawDistanceFrom(yaw_ref));
  state->ReleaseMutex();

  ProcessUpdate(output);
}

void MyController::Reset(void) { first_update = true; }

void MyController::SetValues(const Vector3Df &pos_error,
                             const Vector3Df &vel_error,
                             const Vector3Df &acc_desired,
                             const Quaternion &currentQuaternion,
                             const Vector3Df &omega, float yaw_ref) {
  // Set the input values for the controller.
  // This function is called from the main controller to set the input values.
  input->GetMutex();
  input->SetValue(0, 0, pos_error.x);
  input->SetValue(1, 0, pos_error.y);
  input->SetValue(2, 0, pos_error.z);

  input->SetValue(0, 1, vel_error.x);
  input->SetValue(1, 1, vel_error.y);
  input->SetValue(2, 1, vel_error.z);

  input->SetValue(0, 2, acc_desired.x);
  input->SetValue(1, 2, acc_desired.y);
  input->SetValue(2, 2, acc_desired.z);

  input->SetValue(0, 3, currentQuaternion.q0);
  input->SetValue(1, 3, currentQuaternion.q1);
  input->SetValue(2, 3, currentQuaternion.q2);
  input->SetValue(3, 3, currentQuaternion.q3);

  input->SetValue(0, 4, omega.x);
  input->SetValue(1, 4, omega.y);
  input->SetValue(2, 4, omega.z);

  // Set yaw reference
  input->SetValue(0, 5, yaw_ref);
  input->ReleaseMutex();
}

void MyController::plotControllerData(const LayoutPosition *position) {

  auto *controller_plots_tab = new TabWidget(position, "Controller plots");
  auto graphErrorTab = new Tab(controller_plots_tab, "Tracking Errors");
  auto *plot_cartesian_error = new DataPlot1D(graphErrorTab->LastRowLastCol(), "Cartesian errors", -1, 1);
  plot_cartesian_error->AddCurve(state->Element(0), DataPlot::Red);   // x error
  plot_cartesian_error->AddCurve(state->Element(1), DataPlot::Black); // y error
  plot_cartesian_error->AddCurve(state->Element(2), DataPlot::Blue);  // z error
  auto *plot_attitude_error = new DataPlot1D(graphErrorTab->LastRowLastCol(), "Attitude errors", -1, 1);
  plot_attitude_error->AddCurve(state->Element(4), DataPlot::Red);   
  plot_attitude_error->AddCurve(state->Element(5), DataPlot::Black); 
  plot_attitude_error->AddCurve(state->Element(6), DataPlot::Blue);  

//   auto *input_tab = new TabWidget(position, "Input" );
  auto graphControllerTab = new Tab(controller_plots_tab, "Controller Input");
  auto *plot_controller_thrust = new DataPlot1D(graphControllerTab->LastRowLastCol(), "Thrust Input", -1, 1);  // with no motor constant
  plot_controller_thrust->AddCurve(state->Element(10), DataPlot::Black);
  auto *plot_controller_torque = new DataPlot1D(graphControllerTab->LastRowLastCol(), "Torque Input", -1, 1); 
  plot_controller_torque->AddCurve(state->Element(7), DataPlot::Red); 
  plot_controller_torque->AddCurve(state->Element(8), DataPlot::Black);
  plot_controller_torque->AddCurve(state->Element(9), DataPlot::Blue);
}


void MyController::applyMotorConstant(Vector3Df &signal) {
  auto motor_constant = (float)k_motor->Value();
  signal.x = signal.x / motor_constant;
  signal.y = signal.y / motor_constant;
  signal.z = signal.z / motor_constant;
}

void MyController::applyMotorConstant(float &signal) {
  auto motor_constant = (float)k_motor->Value();
  signal = signal / motor_constant;
}