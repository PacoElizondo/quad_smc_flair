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
  void SetValues(const flair::core::Vector3Df &Pos_0);
  double initial_time;
  struct TrajectoryOutput {
    flair::core::Vector3Df position;
    flair::core::Vector3Df velocity;
    flair::core::Vector3Df acceleration;
    float heading;
};

// Common context passed into every trajectory function
struct TrajectoryContext {
    float     current_time;
    float     amplitude;
    float     speed;
    float     height;
    float     ramp;
    flair::core::Vector3Df pos_initial;
};

private:
  float delta_t;
  float current_time;
  bool first_update;
  flair::core::Vector3Df pos_initial;
  flair::core::Matrix *state;
  flair::gui::DoubleSpinBox *deltaT_custom, *amplitude, *height, *speed;
  flair::gui::GroupBox *traj_selection_box;
  flair::gui::ComboBox *traj_selection;
  float m_prev;

  TrajectoryOutput ComputeStraightLine(const TrajectoryContext& ctx);
  TrajectoryOutput ComputeCircle(const TrajectoryContext& ctx);

  using TrajFn = std::function<TrajectoryOutput(const TrajectoryContext&)>;
  std::unordered_map<int, TrajFn> traj_map_;
  
  void plotCartesianErrors(const flair::gui::LayoutPosition *position);
};
} // namespace filter
} // namespace flair

#endif // MYTRAJECTORY_H