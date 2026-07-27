#ifndef MYTRAJECTORY_H
#define MYTRAJECTORY_H

#include <ComboBox.h>
#include <ControlLaw.h>
#include <GroupBox.h>
#include <Object.h>
#include <Quaternion.h>
#include <Vector3D.h>
#include <functional>
#include <unordered_map>
#include <vector>

namespace flair {
namespace core {
class Matrix;
class io_data;
} // namespace core
namespace gui {
class LayoutPosition;
class DoubleSpinBox;
class CheckBox;
class Label;
class Vector3DSpinBox;
} // namespace gui
namespace filter {
// If you prefer to use a custom controller class, you can define it here.
// ...
}
} // namespace flair

namespace flair {
namespace filter {
class MyTrajectory : public ControlLaw {
public:
  MyTrajectory(const flair::gui::LayoutPosition *position,
               const std::string &name);
  ~MyTrajectory();
  void UpdateFrom(const flair::core::io_data *data);
  // void Update(flair::core::Time time);
  void Reset(void);
  void SetValues(const flair::core::Vector3Df &Pos_0, const float &Yaw_0);
  double initial_time;
  flair::core::Vector3Df last_desired_position;
  float last_desired_yaw = 0.0F;
  float restart_time;
  struct TrajectoryOutput {
    flair::core::Vector3Df position;
    flair::core::Vector3Df velocity;
    flair::core::Vector3Df acceleration;
    float heading;
};

// Common context passed into every trajectory function
struct TrajectoryContext {
    float     current_time;
    float     delta_time;
    float     amplitude;
    float     speed;
    float     height;
    float     ramp;
    float     heading_initial;
    float     min_height;
    float     yaw_desired;
    flair::core::Vector3Df pos_initial;
    flair::core::Vector3Df pos_desired;
};

struct Ramp {
    float acc_peak;
    float rise_time;
    float limit_time;
    float total_time;

        // Default constructor
    Ramp() : acc_peak(0.0), rise_time(0.0), limit_time(0.0), total_time(0.0) {}
    
    // Parameterized constructor
    Ramp(double ap, double rt, double lt, double tt) 
        : acc_peak(ap), rise_time(rt), limit_time(lt), total_time(tt) {}
};

enum class PhaseKind { Accel, Cruise, Decel, Rest };

struct Phase {
    PhaseKind kind;
    float duration;
    float v0;
    float p0;
    Ramp params;
    float cruise_v = 0.0;
    
    // Add this constructor
    Phase(PhaseKind k, float dur, float v0_, float p0_, Ramp p = Ramp(), float cv = 0.0)
        : kind(k), duration(dur), v0(v0_), p0(p0_), params(p), cruise_v(cv) {}

    void eval(float t_local, float& a, float& v, float& p) const;
};

struct Leg {
    core::Vector3Df direction;
    float length;
    std::vector<Phase> phases;
    core::Vector3Df p_start;
    float t_start;
    float duration;
};

struct PlannedTrajectory {
    std::vector<float> t;
    std::vector<flair::core::Vector3Df> pos;
    std::vector<flair::core::Vector3Df> vel;
    std::vector<flair::core::Vector3Df> acc;
    std::vector<Leg> legs;
    float total_time;
};

// Helper functions
Ramp ramp_params(float delta_v, float acc_max, float jerk);
static void eval_ramp(float t, const Ramp& params, float& a, float& v, float& p);
std::tuple<float, float, Ramp> ramp_totals_full(float delta_v, float acc_max, float jerk);
std::vector<Phase> plan_leg(float length, float vel_max, float acc_max, float jerk, float wait);
PlannedTrajectory build_trajectory(const std::vector<core::Vector3Df>& waypoints,
                                  float vel_max, float acc_max, float jerk,
                                  float sleep, float dt);





private:
  float delta_t;
  float current_time;
  float heading_initial;
  bool  first_update;
  flair::core::Vector3Df pos_initial;
  flair::core::Matrix *state;
  flair::gui::DoubleSpinBox *deltaT_custom, *amplitude, *height, *speed, *min_height, *yaw_desired;
  flair::gui::Vector3DSpinBox *pos_desired;
  flair::gui::GroupBox *traj_selection_box;
  flair::gui::ComboBox *traj_selection;
  float m_prev;


    // For the fixed‑jerk trajectory
    bool jerk_traj_initialized = false;
    float jerk_start_time = 0.0F;          // global time when trajectory began
    PlannedTrajectory planned_traj_jerk;   // pre-computed trajectory

  TrajectoryOutput ComputeTanhInterpolation(      const TrajectoryContext& ctx);
  TrajectoryOutput ComputeFixedJerkInterpolation( const TrajectoryContext& ctx);
  TrajectoryOutput ComputeCircle(                 const TrajectoryContext& ctx);

  using TrajFn = std::function<TrajectoryOutput(const TrajectoryContext&)>;
  std::unordered_map<int, TrajFn> traj_map_;
  
  void plotCartesianErrors(const flair::gui::LayoutPosition *position);





  
};
} // namespace filter
} // namespace flair

#endif // MYTRAJECTORY_H