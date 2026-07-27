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
#include <cstddef>
#include <io_data.h>
#include <iostream>
#include <regex>
#include <vector>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;








MyTrajectory::MyTrajectory(const LayoutPosition *position, const string &name)
    : ControlLaw(position->getLayout(), name, 10), first_update(true),
      delta_t(0.001F), initial_time(0.0F) {
  // Input matrix
  input = new Matrix(this, 4, 1, floatType, name);

  // Matrix descriptor for logging. It should be always a nx1 matrix.
  auto *log_labels = new MatrixDescriptor(9, 1);
  log_labels->SetElementName(0, 0, "desired_x");
  log_labels->SetElementName(1, 0, "desired_y");
  log_labels->SetElementName(2, 0, "desired_z");
  log_labels->SetElementName(3, 0, "desired_vel_x");
  log_labels->SetElementName(4, 0, "desired_vel_y");
  log_labels->SetElementName(5, 0, "desired_vel_z");
  log_labels->SetElementName(6, 0, "desired_acc_x");
  log_labels->SetElementName(7, 0, "desired_acc_y");
  log_labels->SetElementName(8, 0, "desired_acc_z");
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
  speed = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Speed", 0, 5,
                             0.001, 4, 1);
  min_height = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Minimum Height", 0.1, 1,
                             0.01, 3, 0.3);

  auto *custom_position =
      new GroupBox(gui_quadsmc->NewRow(), "Waypoint");
  pos_desired = new Vector3DSpinBox(custom_position->LastRowLastCol(), "Desired Pos XYZ", -5,
                              10, 0.1, 3, Vector3Df(0.0F, 0.0F, -2.0F));
  yaw_desired = new DoubleSpinBox(custom_position->LastRowLastCol(), "Desired Yaw", -3.14, 3.14, 0.1, 2, 0.0F);                   
  

  GroupBox *traj_selection_box =
    new GroupBox(general_parameters->NewRow(), "Custom trajectory");
  traj_selection = new ComboBox(traj_selection_box->NewRow(), "Custom trajectory");
  traj_selection->AddItem("Tanh Interpolation");
  traj_selection->AddItem("Recieve Spline");
  traj_selection->AddItem("Circle");
  // traj_selection->AddItem("Circle tracking");
  // traj_selection->AddItem("Trajectory tracking");
  // ADD trajectory options spinbox
  traj_map_ = {
        {0, [this](const TrajectoryContext& ctx) { return ComputeTanhInterpolation(ctx); }},
        {1, [this](const TrajectoryContext& ctx) { return ComputeFixedJerkInterpolation(ctx); }},
        {2, [this](const TrajectoryContext& ctx) { return ComputeCircle(ctx); }},
  };


  // Show cartesian errors plot
  plotCartesianErrors(gui_quadsmc->NewRow());

  // AddDataToLog(output);
  // AddDataToLog(state);
}

MyTrajectory::~MyTrajectory() { delete state; }

void MyTrajectory::UpdateFrom(const io_data *data) {
  if (first_update) {
    
    // initial_time = 0.0F;
    pos_initial =
    Vector3Df(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    heading_initial = input->Value(3,0);
    
    first_update = false;
  }

  bool calibration = false;
  double double_time = GetTime() / 1e9 - initial_time ;
  float current_time = float(double_time);

  
  auto amplitude_value = (float)amplitude->Value();
  auto height_value = (float)height->Value();
  auto speed_value = (float)speed->Value();
  auto min_height_value = (float)min_height->Value();
  
  double double_delta = data->DataDeltaTime() / 1e9;
  float dt = float(double_delta);

  Vector3Df desired_position;
  Vector3Df desired_velocity;
  Vector3Df desired_acceleration;
  
  float ramp = 0.0F;

  ramp = std::fmin(current_time / 5.0F, 1.0F);
  float m = std::fmin(current_time * speed_value, amplitude_value);
  float m_delta = (m - m_prev) / dt;
  


  m_prev = m;

  TrajectoryContext ctx;
  ctx.current_time = current_time;
  ctx.delta_time   = dt;
  ctx.amplitude    = amplitude_value;
  ctx.speed        = speed_value;
  ctx.height       = height_value;
  ctx.ramp         = ramp;
  ctx.pos_initial  = pos_initial;
  ctx.min_height   = min_height_value;
  ctx.heading_initial = heading_initial;
  ctx.pos_desired = pos_desired->Value();
  ctx.yaw_desired = yaw_desired->Value();

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

// MyTrajectory::TrajectoryOutput MyTrajectory::ComputeBackToOrigin(const TrajectoryContext& ctx) {
//     TrajectoryOutput out;
    // Vector3Df pos_init = ctx.pos_initial;
    // Vector3Df pos_origin = Vector3Df(0.0,0.0,-2.0);
//     float t = ctx.current_time;
//     float a = ctx.amplitude;
//     float s = ctx.speed;
//     float mh = ctx.min_height;
//     float init_yaw = ctx.t_localheading_initial;
//     float ch = coshf(t * s - 2.0F);
//     float th = tanhf(t * s - 2.0F);

//     out.position.x = a * (th + 1.0F);
//     out.position.y = 0;
//     out.position.z = a * (th - 1.0F) - mh;

//     out.velocity.x = a / (ch * ch);
//     out.velocity.y = 0;
//     out.velocity.z = out.velocity.x;

//     out.acceleration.x = (-2.0F * a * th) / (ch * ch);
//     out.acceleration.y = 0;
//     out.acceleration.z = out.acceleration.x;

//     out.heading = heading_initial * (0.5F-(0.5F*th));
//     // std::cout << heading_initial << "heading ref \n";

//     // out.heading = 0.0F;
//     return out;
// }


MyTrajectory::TrajectoryOutput MyTrajectory::ComputeTanhInterpolation(const TrajectoryContext& ctx) {
    TrajectoryOutput out;
    Vector3Df pos_init = ctx.pos_initial;
    Vector3Df pos_des = ctx.pos_desired;
    Vector3Df check_change = pos_des - last_desired_position;
    float init_yaw = ctx.heading_initial;
    float yaw_des = ctx.yaw_desired;
    float     check_yaw = yaw_des - last_desired_yaw;
    float current_time = ctx.current_time;
    if (check_change.x != 0.0F || check_change.y != 0.0F|| check_change.z != 0.0F || check_yaw != 0.0F ) {
      last_desired_position = pos_des;
      last_desired_yaw = yaw_des;
      restart_time = current_time;
    };
    
    pos_initial = Vector3Df(input->Value(0, 0), input->Value(1, 0), input->Value(2, 0));
    init_yaw = input->Value(3,0);
    

    float t = current_time - restart_time;
    float a = ctx.amplitude;
    float s = ctx.speed;
    // float hs = ctx.heading_speed;
    float hs = 1.0F;
    float mh = ctx.min_height;

    float ch = coshf(t * s - 2.0F);
    float th = tanhf(t * s - 2.0F);
    float heading_th = tanhf(t * hs - 2.0F);
    float soft = 0.5F + 0.5F * th;
    float soft_dot = 0.5F * s / (ch * ch);
    float soft_dot_dot =  - ((s*s) / (ch * ch)) * th;


    out.position.x = pos_init.x + soft * (pos_des.x - pos_init.x);
    out.position.y = pos_init.y + soft * (pos_des.y - pos_init.y);
    out.position.z = pos_init.z + soft * (pos_des.z - pos_init.z);
    
    // out.position.y = 0;
    // out.position.z = a * (th - 1.0F) - mh;

    out.velocity.x = soft_dot * (pos_des.x - pos_init.x);
    out.velocity.y = soft_dot * (pos_des.y - pos_init.y);
    out.velocity.z = soft_dot * (pos_des.z - pos_init.z);

    out.acceleration.x = soft_dot_dot * (pos_des.x - pos_init.x);
    out.acceleration.y = soft_dot_dot * (pos_des.y - pos_init.y);;
    out.acceleration.z = soft_dot_dot * (pos_des.z - pos_init.z);;

    out.heading = init_yaw +  (0.5F + 0.5F*heading_th)*(yaw_des - init_yaw);
    // std::cout << heading_initial << "heading ref \n";

    // out.heading = 0.0F;
    return out;
}

MyTrajectory::TrajectoryOutput MyTrajectory::ComputeFixedJerkInterpolation(const TrajectoryContext& ctx) {
    TrajectoryOutput out;
    PlannedTrajectory traj;
    const float V_MAX  = 1.0;
    const float A_MAX  = 2.0;
    const float J_FIX  = 4.0;
    const float T_WAIT = 1.0;
    const float DT     = 0.001F;
    float t      = ctx.current_time;
    float a      = ctx.amplitude;
    float mh     = ctx.min_height;
    

    static const std::vector<Vector3Df> waypoints = []() {
        std::vector<Vector3Df> pts;
        int n_pts = 6;
        for (int i = 0; i < n_pts; ++i) {
            double t = i * (2.0 * M_PI / (n_pts - 1));
            pts.push_back({float(std::cos(t)), float(std::sin(t)), float(t / (2.0 * M_PI))});
        }
        pts.push_back({1.0f, 1.0f, 0.1f});
        pts.push_back({1.0f, 1.5f, 0.1f});
        return pts;
    }();

    if (!jerk_traj_initialized) {
        planned_traj_jerk = build_trajectory(waypoints, V_MAX, A_MAX, J_FIX, T_WAIT, DT);
        jerk_start_time = ctx.current_time;
        jerk_traj_initialized = true;
    }

      float t_local = ctx.current_time - jerk_start_time;
    float total_time = planned_traj_jerk.total_time;

    // If trajectory finished, hold the final position
    if (t_local >= total_time || total_time <= 0.0f) {
        // Use the last computed position (end of trajectory)
        size_t last_idx = planned_traj_jerk.pos.size() - 1;
        out.position = planned_traj_jerk.pos[last_idx];
        out.velocity = Vector3Df(0.0f, 0.0f, 0.0f);
        out.acceleration = Vector3Df(0.0f, 0.0f, 0.0f);

        // Heading: you can set it to the last heading or keep it constant.
        // Here we simply point to the next waypoint's direction (or 0).
        out.heading = 0.0f;
    } else {
        // Linear interpolation between samples (simple nearest neighbour is fine at 1 kHz)
        int idx = static_cast<int>(t_local / DT);
        if (idx < 0) idx = 0;
        if (idx >= static_cast<int>(planned_traj_jerk.pos.size()))
            idx = planned_traj_jerk.pos.size() - 1;

        out.position = planned_traj_jerk.pos[idx];
        out.velocity = planned_traj_jerk.vel[idx];
        out.acceleration = planned_traj_jerk.acc[idx];

        // Heading: a smooth tanh transition as in your other functions
        // (you can replace with a simple constant if desired)
        float yaw_start = input->Value(3, 0);   // current yaw from drone
        float yaw_end = atan2f(
            waypoints.back().y - waypoints.front().y,
            waypoints.back().x - waypoints.front().x);  // example: point toward final wp
        float th = tanhf(t_local * 1.0f - 2.0f);
        out.heading = yaw_start + (0.5f + 0.5f * th) * (yaw_end - yaw_start);
    }


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

void MyTrajectory::Reset(void) { 
    first_update = true;
    initial_time = GetTime() / 1e9; 
    m_prev = 0;     
    jerk_traj_initialized = false;
    planned_traj_jerk = PlannedTrajectory();
}

void MyTrajectory::SetValues(const Vector3Df &Pos_0, const float &Yaw_0) {
  // Set the input values for the path planner. Now it only receives a misc
  // variable that can be used to set any value you want. This function is
  // called from the main controller to set the input values.
  input->GetMutex();
  input->SetValue(0, 0, Pos_0.x);
  input->SetValue(1, 0, Pos_0.y);
  input->SetValue(2, 0, Pos_0.z);
  input->SetValue(3, 0, Yaw_0);
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

//-----------------------------------------------------------------------------
// Helper function implementations
//-----------------------------------------------------------------------------

void MyTrajectory::Phase::eval(float t_local, float& a, float& v, float& p) const {
    if (t_local < 0.0) t_local = 0.0;
    if (t_local > duration) t_local = duration;

    if (kind == PhaseKind::Rest) {
        a = 0.0; v = 0.0; p = p0;
    } else if (kind == PhaseKind::Cruise) {
        a = 0.0; v = cruise_v; p = p0 + cruise_v * t_local;
    } else if (kind == PhaseKind::Accel) {
        float a_local, v_local, p_local;
        eval_ramp(t_local, params, a_local, v_local, p_local);
        a = a_local;
        v = v0 + v_local;
        p = p0 + v0 * t_local + p_local;
    } else { // Decel
        float a_local, v_local, p_local;
        eval_ramp(t_local, params, a_local, v_local, p_local);
        a = -a_local;
        v = v0 - v_local;
        p = p0 + v0 * t_local - p_local;
    }
}

MyTrajectory::Ramp MyTrajectory::ramp_params(float delta_v, float acc_max, float jerk) {
    Ramp params;
    if (delta_v <= 0.0) {
        params.acc_peak = 0.0F;
        params.rise_time = 0.0F;
        params.limit_time = 0.0F;
        params.total_time = 0.0F;
        return {params};
    }

    float rise_time = acc_max / jerk;
    float limit_time = delta_v / acc_max;

    float a_pk, rt, lt, tt;

    if (limit_time >= rise_time) {
        a_pk = acc_max;
        rt = rise_time;
        lt = limit_time;
        tt = rt + lt;
    } else {
        a_pk = std::sqrt(delta_v * jerk);
        rt = a_pk / jerk;
        lt = rt;
        tt = 2.0 * rt;
    }
    params.acc_peak = 0.0F;
    params.rise_time = 0.0F;
    params.limit_time = 0.0F;
    params.total_time = 0.0F;

    return {params};
}

void MyTrajectory::eval_ramp(float t, const Ramp& params, float& a, float& v, float& p) {
    float a_pk = params.acc_peak;
    float rt = params.rise_time;
    float lt = params.limit_time;
    float tt = params.total_time;

    if (tt <= 0.0) {
        a = 0.0; v = 0.0; p = 0.0;
        return;
    }

    t = std::min(std::max(t, 0.0F), tt);

    if (t <= rt) {
        a = (rt > 0.0) ? (a_pk * (t / rt)) : 0.0;
        v = (rt > 0.0) ? ((a_pk / rt) * (t * t / 2.0)) : 0.0;
        p = (rt > 0.0) ? ((a_pk / rt) * (t * t * t / 6.0)) : 0.0;
        return;
    }

    float v_rt = a_pk * rt / 2.0;
    float p_rt = a_pk * rt * rt / 6.0;

    if (t <= lt) {
        float delta_t = t - rt;
        a = a_pk;
        v = v_rt + a_pk * delta_t;
        p = p_rt + v_rt * delta_t + a_pk * delta_t * delta_t / 2.0;
        return;
    }

    float delta_lt = lt - rt;
    float v_lt = v_rt + a_pk * delta_lt;
    float p_lt = p_rt + v_rt * delta_lt + a_pk * delta_lt * delta_lt / 2.0;

    float delta_t = t - lt;
    a = (rt > 0.0) ? (a_pk * (1.0 - delta_t / rt)) : 0.0;
    v = (rt > 0.0) ? (v_lt + a_pk * (delta_t - delta_t*delta_t / (2.0*rt))) : 0.0;
    p = (rt > 0.0) ? (p_lt + v_lt * delta_t + a_pk * ((delta_t*delta_t/2.0) - (delta_t*delta_t*delta_t/(6.0*rt)))) : 0.0;
}

// std::tuple<float, float, MyTrajectory::Ramp> MyTrajectory::ramp_totals_full(float delta_v, float acc_max, float jerk) {
//     Ramp params = ramp_params(delta_v, acc_max, jerk);
//     float a, v, p_total;
//     eval_ramp(params.total_time, params, a, v, p_total);
//     return {p_total, params.total_time, params};
// }

std::vector<MyTrajectory::Phase> MyTrajectory::plan_leg(float length, float vel_max, float acc_max, float jerk, float wait) {
    if (length <= 1e-9F) {
        return {};
    }

    // auto [rise_dist, rise_time, full_ramp] = ramp_totals_full(vel_max, acc_max, jerk);
    Ramp params = ramp_params(vel_max, acc_max, jerk); // ramp_totals_full
    float a, v, p_total;
    eval_ramp(params.total_time, params, a, v, p_total);
    auto rise_dist = p_total;
    auto rise_time = params.total_time;
    auto full_ramp = params;

    if (2.0F * rise_dist <= length) {
        // Trapezoidal: reaches cruise velocity
        float cruise_dist = length - 2.0 * rise_dist;
        float cruise_time = cruise_dist / vel_max;

        std::vector<Phase> phases;

        phases.push_back({PhaseKind::Accel, full_ramp.total_time, 0.0F, 0.0F, full_ramp, 0.0F});
        phases.push_back({PhaseKind::Cruise, cruise_time, vel_max, rise_dist, Ramp(), vel_max});
        phases.push_back({PhaseKind::Decel, full_ramp.total_time, vel_max, rise_dist + cruise_dist, full_ramp, 0.0F});
        phases.push_back({PhaseKind::Rest, wait, 0.0F, 2.0F*rise_dist + cruise_dist, Ramp(), 0.0F});
        return phases;
    }

    // Triangular: peak speed < vel_max
    float lo = 0.0, hi = vel_max;
    for (int i = 0; i < 60; ++i) {
        float mid = 0.5 * (lo + hi);
        // auto [d_mid, t_mid, params_mid] = ramp_totals_full(mid, acc_max, jerk);
        Ramp params2 = ramp_params(mid, acc_max, jerk);
        float a2, v2, p_total2;
        eval_ramp(params2.total_time, params2, a2, v2, p_total2);
        auto d_mid = p_total2;
        auto t_mid = params2.total_time;
        auto params_mid = params2;

        if (2.0F * d_mid < length) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    float v_peak = 0.5 * (lo + hi);
    // auto [p_peak, t_ramp, params_peak] = ramp_totals_full(v_peak, acc_max, jerk);
    Ramp params3 = ramp_params(v_peak, acc_max, jerk);
    float a3, v3, p_total3;
    eval_ramp(params3.total_time, params3, a3, v3, p_total3);
    auto p_peak = p_total3;
    auto t_ramp = params3.total_time;
    auto params_peak = params3;

    std::vector<Phase> phases;
    phases.push_back({PhaseKind::Accel, params_peak.total_time, 0.0F, 0.0F, params_peak, 0.0F});
    phases.push_back({PhaseKind::Decel, params_peak.total_time, v_peak, length/2.0F, params_peak, 0.0F});
    phases.push_back({PhaseKind::Rest, wait, 0.0F, length, Ramp(), 0.0F});
    return phases;
}

MyTrajectory::PlannedTrajectory MyTrajectory::build_trajectory(
    const std::vector<Vector3Df>& waypoints,
    float vel_max, float acc_max, float jerk,
    float sleep, float dt) {

    Vector3Df flair_waypoints;
    // flair_waypoints.x = waypoints.data()->x;
    // flair_waypoints.y = waypoints.data()->y;
    // flair_waypoints.z = waypoints.data()->z;

    
    if (waypoints.size() < 2) {
        return {};
    }

    std::vector<Leg> legs;
    float t_curr = 0.0;

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Vector3Df p_start = waypoints[i];
        Vector3Df p_end = waypoints[i+1];
        Vector3Df delta_p = p_end - p_start;
        float length = delta_p.GetNorm();
        if (length < 1e-9) {continue;}
        

        delta_p.Normalize(); // Now direction vector
        auto phases = plan_leg(length, vel_max, acc_max, jerk, sleep);
        float leg_duration = 0.0;
        for (const auto& ph : phases) leg_duration += ph.duration;

        legs.push_back({delta_p, length, phases, p_start, t_curr, leg_duration});
        t_curr += leg_duration;
    }

    float total_time = t_curr;

    int n_samples = static_cast<int>(std::ceil(total_time / dt)) + 1;
    PlannedTrajectory traj;
    traj.total_time = total_time;
    traj.t.resize(n_samples);
    traj.pos.resize(n_samples);
    traj.vel.resize(n_samples);
    traj.acc.resize(n_samples);
    traj.legs = legs;

    

    for (int i = 0; i < n_samples; ++i) {
        traj.t[i] = (i < n_samples - 1) ? i * dt : total_time;
    }

    size_t leg_idx = 0;
    for (int k = 0; k < n_samples; ++k) {
        float t = traj.t[k];
        while (leg_idx < legs.size() - 1 &&
               t > legs[leg_idx].t_start + legs[leg_idx].duration + 1e-12) {
            ++leg_idx;
        }
        const Leg& leg = legs[leg_idx];
        float t_in_leg = t - leg.t_start;

        float a_local = 0.0, v_local = 0.0, p_local = leg.length;
        float t_phase = t_in_leg;
        for (const auto& ph : leg.phases) {
            if (t_phase <= ph.duration + 1e-12 || &ph == &leg.phases.back()) {
                ph.eval(std::min(t_phase, ph.duration), a_local, v_local, p_local);
                break;
            }
            t_phase -= ph.duration;
        }

        traj.pos[k] = leg.p_start + leg.direction * p_local;
        traj.vel[k] = leg.direction * v_local;
        traj.acc[k] = leg.direction * a_local;
    }

    return traj;
}