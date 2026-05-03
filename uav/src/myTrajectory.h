#ifndef MYTRAJECTORY_H
#define MYTRAJECTORY_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Quaternion.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
        class Vector3DSpinBox;
    }
    namespace filter {
        // If you prefer to use a custom controller class, you can define it here.
        // ...
    }
}

namespace flair {
    namespace filter {
        class MyTrajectory : public ControlLaw
        {
            public :
                MyTrajectory(const flair::gui::LayoutPosition *position, const std::string &name);
                ~MyTrajectory();
                void UpdateFrom(const flair::core::io_data *data);
                void Reset(void);
                void SetValues(float misc);

            private : 
                float delta_t, initial_time;
                bool first_update;

                flair::core::Matrix *state;
                flair::gui::DoubleSpinBox *deltaT_custom, *amplitude, *z_rate;

                void plotCartesianErrors(const flair::gui::LayoutPosition *position);
        };
    }
}

#endif // MYTRAJECTORY_H