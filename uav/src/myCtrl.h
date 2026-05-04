#ifndef MYCTRL_H
#define MYCTRL_H

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
class MyController : public ControlLaw {
public:
  MyController(const flair::gui::LayoutPosition *position,
               const std::string &name);
  ~MyController();
  void UpdateFrom(const flair::core::io_data *data);
  void Reset(void);
  void SetValues(const flair::core::Vector3Df &pos_error,
                 const flair::core::Vector3Df &vel_error,
                 const flair::core::Vector3Df &acc_desired,
                 const flair::core::Quaternion &currentQuaternion,
                 const flair::core::Vector3Df &omega, float yaw_ref);
  void applyMotorConstant(flair::core::Vector3Df &signal);
  void applyMotorConstant(float &signal);

private:
  float delta_t, initial_time;
  float g = 9.81;
  bool first_update;

  flair::core::Matrix *state;
  flair::gui::Vector3DSpinBox *K_pos, *Lambda_pos, *K_att, *Lambda_att,
      *K_surf_pos_t0;
  flair::gui::DoubleSpinBox *deltaT_custom, *mass, *k_motor, *sat_pos, *sat_att,
      *sat_thrust;

  flair::core::Vector3Df pos_error_0;
  flair::core::Vector3Df vel_error_0;
  flair::core::Vector3Df Surface_pos_t0;
  flair::core::Vector3Df omega_desired_prev;
  flair::core::Quaternion q_desired_prev;
  flair::core::Quaternion q_desired;
  float current_time;

  void plotCartesianErrors(const flair::gui::LayoutPosition *position);
};
} // namespace filter
} // namespace flair

#endif // MYCTRL_H