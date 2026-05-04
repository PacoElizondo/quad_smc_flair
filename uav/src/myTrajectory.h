#ifndef MYTRAJECTORY_H
#define MYTRAJECTORY_H

#include <ControlLaw.h>
#include <Object.h>
#include <Quaternion.h>
#include <Vector3D.h>

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
  void Reset(void);
  void SetValues(const flair::core::Vector3Df &Pos_0);

private:
  float delta_t, initial_time;
  float current_time;
  bool first_update;
  flair::core::Vector3Df pos_initial;
  flair::core::Matrix *state;
  flair::gui::DoubleSpinBox *deltaT_custom, *amplitude, *z_rate, *xy_rate;

  void plotCartesianErrors(const flair::gui::LayoutPosition *position);
};
} // namespace filter
} // namespace flair

#endif // MYTRAJECTORY_H