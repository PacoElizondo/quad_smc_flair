// Custom SMC Controller - Francisco Elizondo-Coronado

#include "myCtrl.h"
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
#include <cmath>
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
  auto *log_labels = new MatrixDescriptor(7, 1);
  log_labels->SetElementName(0, 0, "x_error");
  log_labels->SetElementName(1, 0, "y_error");
  log_labels->SetElementName(2, 0, "yaw_error");
  log_labels->SetElementName(3, 0, "tau_x");
  log_labels->SetElementName(4, 0, "tau_y");
  log_labels->SetElementName(5, 0, "tau_z");
  log_labels->SetElementName(6, 0, "thrust");

  state = new Matrix(this, log_labels, floatType, name);
  delete log_labels;

  // GUI for custom controller
  auto *gui_quadsmc = new GroupBox(position, name);
  auto *general_parameters = new GroupBox(gui_quadsmc->NewRow(), " ");
  deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(),
                                    "Custom dt [s]", 0, 1, 0.001, 4);
  mass = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Mass [kg]", 0,
                           10, 0.01, 4, 1.2);
  k_motor = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                              "Motor constant", 0, 50, 0.01, 4, 29.5870);
  sat_thrust = new DoubleSpinBox(general_parameters->NewRow(),
                                 "Saturation thrust", 0, 10, 0.01, 3);
  sat_pos = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                              "Saturation pos", 0, 10, 0.01, 3);
  sat_att = new DoubleSpinBox(general_parameters->LastRowLastCol(),
                              "Saturation att", 0, 10, 0.01, 3);

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
  plotCartesianErrors(gui_quadsmc->NewRow());

  AddDataToLog(state);
}

MyController::~MyController() { delete state; }

void MyController::UpdateFrom(const io_data *data) {
  // float current_time = (float(GetTime()) / 1000000000.0F) - initial_time;
  std::cout << current_time << "\n";
  current_time = current_time + delta_t;
  Vector3Df u_position;
  Vector3Df tau;
  
  auto mass_val = (float)mass->Value();
  const float gravity = 9.81;
  // const Vector3Df J_diag = Vector3Df(0.000002098,  0.000002102, 0.000004068);
  
  const Vector3Df J_diag = Vector3Df(0.006,  0.006, 0.1);
  // if (deltaT_custom->Value() == 0) {
  //   delta_t = (float)(data->DataDeltaTime()) / 1000000000.0F;
  // } else {
  //   delta_t = (float)deltaT_custom->Value();
  // }

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

//   pos_error.x = 0.0F;
//   pos_error.y = 0.0F;
//   pos_error.z = 0.0F;

  // std::cout << pos_error.x << ", " << pos_error.y << ", " << pos_error.z
            // << " pos_error @ myCtrl from input \n";



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
      vel_error.x + (Lambda_pos_val.x * pos_error.x) +
          (Surface_pos_t0.x * expf(-K_surf_pos_t0_val.x * current_time))*0,
      vel_error.y + (Lambda_pos_val.y * pos_error.y) +
          (Surface_pos_t0.y * expf(-K_surf_pos_t0_val.y * current_time))*0,
      vel_error.z + (Lambda_pos_val.z * pos_error.z) +
          (Surface_pos_t0.z * expf(-K_surf_pos_t0_val.z * current_time))*0
        );

  
  Vector3Df surface_pos_dot =
      Vector3Df(-(mass_val * Lambda_pos_val.x * vel_error.x) -
                    (mass_val * K_pos_val.x * sqrtf(fabsf(surface_pos.x))* 
                     tanhf(surface_pos.x)) + acc_desired.x
                     + (mass_val * K_surf_pos_t0_val.x * Surface_pos_t0.x *
                     expf(-K_surf_pos_t0_val.x * current_time))*0,
                -(mass_val * Lambda_pos_val.y * vel_error.y) -
                    (mass_val * K_pos_val.y * sqrtf(fabsf(surface_pos.y))*
                     tanhf(surface_pos.y)) + acc_desired.y
                     + (mass_val * K_surf_pos_t0_val.y * Surface_pos_t0.y *
                     expf(-K_surf_pos_t0_val.y * current_time))*0,
                -(mass_val * Lambda_pos_val.z * vel_error.z) -
                    (mass_val * K_pos_val.z * sqrtf(fabsf(surface_pos.z))*
                     tanhf(surface_pos.z)) - (mass_val*g) + acc_desired.z
                     + (mass_val * K_surf_pos_t0_val.z * Surface_pos_t0.z *
                 expf(-K_surf_pos_t0_val.z * current_time))*0 
      );




  // std::cout << surface_pos_dot.x << ", " << surface_pos_dot.y << ", " << surface_pos_dot.z
            // << "s_p_dot" << "\n";

  float thrust_norm = sqrtf(DotProduct(surface_pos_dot, surface_pos_dot));
  Vector3Df temp = CrossProduct(Vector3Df(0.0F, 0.0F, -1.0F), surface_pos_dot);
  float temp_norm = sqrtf(DotProduct(temp, temp));

  Quaternion thrust_q =
      Quaternion(0.0F, surface_pos_dot.x, surface_pos_dot.y, surface_pos_dot.z);
  q_desired = Quaternion(-thrust_q.q3 + thrust_norm, temp.x, temp.y, temp.z);
    
  
    q_desired.Normalize();
    // std::cout << q_desired.q0 << "," << q_desired.q1 << "," << q_desired.q2 << "," << q_desired.q3 << " q_des \n";
    std::cout << thrust_norm << " thrust norm \n";


    

  Quaternion body_z_world =
      q_desired * Quaternion(0.0F, 0.0F, 0.0F, 1.0F) * q_desired.GetConjugate();

  u_position.x = surface_pos_dot.x;
  u_position.y = surface_pos_dot.y;
  u_position.z = surface_pos_dot.z;


  // float control_threshold = 1e-3;


  float ctrl_z = DotProduct(u_position, Vector3Df(body_z_world.q1, body_z_world.q2, body_z_world.q3));
  u_position.Saturate((float)sat_pos->Value());

  // u_position = Vector3Df(
  // (fabsf(u_position.x) < control_threshold) ? 0.0F : u_position.x,
  // (fabsf(u_position.y) < control_threshold) ? 0.0F : u_position.y,
  // (fabsf(u_position.z) < control_threshold) ? 0.0F : u_position.z
  // );



//   // Computing omega desired from thrust vector
//   Quaternion u_position_normalized_q = Quaternion(0.0, u_position.x/u_position.GetNorm(), u_position.y/u_position.GetNorm(), u_position.z/u_position.GetNorm());
//   Quaternion thrust_curr_q = Quaternion(0.0,0.0,0.0,thrust_curr/mass_val);
//   Quaternion thrust_curr_normalized_q = Quaternion(0.0,0.0,0.0,thrust_curr/thrust_curr_norm);
//   Quaternion thrust_curr_inertial_q = (quat*thrust_curr_q*quat.GetConjugate());
//   Vector3Df  v_dot = Vector3Df(thrust_curr_inertial_q.q1, thrust_curr_inertial_q.q2, thrust_curr_inertial_q.q3) + Vector3Df(0.0,0.0,g);
//   Vector3Df  acc_error = v_dot - acc_desired;

//   std::cout << acc_error.x << ", " << acc_error.y << ", " << acc_error.z << "acc_error \n ";

  
  
  
//   Vector3Df F_u_dot = Vector3Df(
//     -(Lambda_pos_val.x * acc_error.x) - (K_pos_val.x*(1/coshf(u_position.x))*(1/coshf(u_position.x))),
//     -(Lambda_pos_val.y * acc_error.y) - (K_pos_val.y*(1/coshf(u_position.y))*(1/coshf(u_position.y))),
//     -(Lambda_pos_val.z * acc_error.z) - (K_pos_val.z*(1/coshf(u_position.z))*(1/coshf(u_position.z)))
//   );

//   Vector3Df  F_u_normalized = u_position/u_position.GetNorm();

//   F_u_normalized = Vector3Df(
//     (fabsf(F_u_normalized.x) < control_threshold) ? 0.0F : F_u_normalized.x,
//     (fabsf(F_u_normalized.y) < control_threshold) ? 0.0F : F_u_normalized.y,
//     (fabsf(F_u_normalized.z) < control_threshold) ? 0.0F : F_u_normalized.z
//   );

//   // std::cout << "F_u_dot: " << F_u_dot.x << ", " << F_u_dot.y << ", " << F_u_dot.z << "\n";
  
//   std::cout << "thrust_curr: " << thrust_curr << "\n";
//   std::cout << "normalized thrust curr" << thrust_curr/thrust_norm << "\n";

//   Quaternion F_u_normalized_q = Quaternion(0.0,F_u_normalized.x, F_u_normalized.y, F_u_normalized.z);
//   float dot = F_u_normalized.x * F_u_dot.x 
//             + F_u_normalized.y * F_u_dot.y 
//             + F_u_normalized.z * F_u_dot.z;

//   Vector3Df F_u_dot_normalized = Vector3Df(
//       (F_u_dot.x - F_u_normalized.x * dot) / u_position.GetNorm(),
//       (F_u_dot.y - F_u_normalized.y * dot) / u_position.GetNorm(),
//       (F_u_dot.z - F_u_normalized.z * dot) / u_position.GetNorm()
//   );
//   // Vector3Df F_u_dot_normalized;
//   // F_u_dot_normalized = (F_u_dot - F_u_normalized * F_u_n_F_u_dot) / u_position.GetNorm();


//   std::cout << "F_u_normalized_q: " << F_u_normalized_q.q0 << ", " << F_u_normalized_q.q1 << ", " << F_u_normalized_q.q2 << ", " << F_u_normalized_q.q3 << "\n";
//   std::cout << "F_u_dot: " << F_u_dot.x << ", " << F_u_dot.y << ", " << F_u_dot.z << "\n";
// std::cout << "F_u_dot_normalized: " << F_u_dot_normalized.x << ", " << F_u_dot_normalized.y << ", " << F_u_dot_normalized.z << "\n";
// std::cout << "thrust_curr_normalized_q: " << thrust_curr_normalized_q.q0 << ", " << thrust_curr_normalized_q.q1 << ", " << thrust_curr_normalized_q.q2 << ", " << thrust_curr_normalized_q.q3 << "\n";

//   Quaternion F_u_dot_normalized_q = Quaternion(0.0,F_u_dot_normalized.x, F_u_dot_normalized.y, F_u_dot_normalized.z);

//   Quaternion omega_desired_q = thrust_curr_normalized_q*F_u_normalized_q.GetConjugate()*F_u_dot_normalized_q*thrust_curr_normalized_q.GetConjugate();
//   Vector3Df  omega_desired = Vector3Df(omega_desired_q.q1,omega_desired_q.q2, omega_desired_q.q3);
//   std::cout << omega_desired.x << ", " << omega_desired.y << ", " << omega_desired.z << "omega_desired \n ";


  // Attitude controller

  Quaternion q_error = q_desired.GetConjugate() * quat;
  Vector3Df att_error = 2 * Vector3Df(q_error.q1,q_error.q2, q_error.q3);
  

  // std::cout << att_error.x << att_error.y << att_error.z << "att_error y \n";



  // #TODO: obtain analitic derivative for omega desired and omega_desired_dot
  Quaternion q_desired_dot;
  
  // Vector3Df omega_desired =
  //     Vector3Df(omega_desired_q.q1, omega_desired_q.q2, omega_desired_q.q3);
  omega_desired = Vector3Df(0.0,0.0,0.0);
  Vector3Df omega_error = omega - omega_desired;

  // Vector3Df omega_error = Vector3Df(0.0,0.0,0.0);
  // Vector3Df omega_desired_dot = (omega_desired - omega_desired_prev) / delta_t;

  Vector3Df surface_att = omega_error + Lambda_att_val * att_error;

  Vector3Df u_att_sw = Vector3Df(-K_att_val.x * tanhf(surface_att.x),
                                 -K_att_val.y * tanhf(surface_att.y),
                                 -K_att_val.z * tanhf(surface_att.z));

  // Vector3Df surface_att_dot =
  // CrossProduct(omega, (J_diag * omega)) +
  // J_diag * (-Lambda_att_val * omega_error + u_att_sw);
Vector3Df surface_att_dot =
  CrossProduct(omega, (J_diag * omega)) +
  J_diag*(- u_att_sw);
  
  

  tau =Vector3Df( surface_att_dot.x, surface_att_dot.y,surface_att_dot.z) ;
  
  // std::cout << tau.x << ", " << tau.y << ", " << tau.z << "torques \n" ;
  // std::cout << omega_desired.x << omega_desired.y << omega_desired.z << "omega d \n";

  // tau = Vector3Df(0.0,0.0,0.0);


  // q_desired_prev = q_desired;
  // omega_desired_prev = omega_desired;
  
  applyMotorConstant(tau);
  tau.Saturate((float)sat_att->Value());
  

    // Compute custom thrust
    // float comp_mg = -(float)mass->Value()*g; // This is the thrust needed to counteract gravity. Based on the default PID, it should be -0.397918 in Fl-Air simulator.  
    thrust = ctrl_z ; // This is the thrust needed to counteract gravity and control the z position
    thrust_curr = thrust; //store in global variable

    // std::cout << thrust << "thrust \n";
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
  state->SetValue(3, 0, tau.x);
  state->SetValue(4, 0, tau.y);
  state->SetValue(5, 0, tau.z);
  state->SetValue(6, 0, thrust);
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

void MyController::plotCartesianErrors(const LayoutPosition *position) {
  // Example of how to plot the position errors in the GUI.
  // Any variable that is defined in the state matrix can be plotted. Just
  // remember to set its value in the UpdateFrom function and to add it to the
  // log_labels matrix in the constructor.
  auto *plot = new DataPlot1D(position, "Cartesian errors", -1, 1);
  plot->AddCurve(state->Element(0), DataPlot::Red);   // x error
  plot->AddCurve(state->Element(1), DataPlot::Black); // y error
  plot->AddCurve(state->Element(2), DataPlot::Blue);  // yaw error
}

// void MyController::plotCartesianErrors(const LayoutPosition *position) {
//   // Example of how to plot the position errors in the GUI.
//   // Any variable that is defined in the state matrix can be plotted. Just
//   // remember to set its value in the UpdateFrom function and to add it to the
//   // log_labels matrix in the constructor.
//   auto *plot = new DataPlot1D(position, "Cartesian errors", -1, 1);
//   plot->AddCurve(output->Element(0), DataPlot::Red);   // x error
//   plot->AddCurve(output->Element(1), DataPlot::Black); // y error
//   plot->AddCurve(output->Element(2), DataPlot::Blue);  // yaw error
// }

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