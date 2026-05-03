#include "myTrajectory.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <Label.h>
#include <Vector3DSpinBox.h>
#include <Pid.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

MyTrajectory::MyTrajectory(const LayoutPosition *position, const string &name) 
    : ControlLaw(position->getLayout(), name, 9), first_update(true), delta_t(0.001F), initial_time(0.0F)
{
    // Input matrix
    input = new Matrix(this, 1, 1, floatType, name);

    // Matrix descriptor for logging. It should be always a nx1 matrix. 
    auto *log_labels = new MatrixDescriptor(3, 1);
    log_labels->SetElementName(0, 0, "desired_x");
    log_labels->SetElementName(1, 0, "desired_y");
    log_labels->SetElementName(2, 0, "desired_z");
    state = new Matrix(this, log_labels, floatType, name);
    delete log_labels;

    // GUI for path planning
    auto *gui_quadsmc = new GroupBox(position, name);
    auto *general_parameters = new GroupBox(gui_quadsmc->NewRow(), " ");
    deltaT_custom = new DoubleSpinBox(general_parameters->NewRow(), "Custom dt [s]", 0, 1, 0.001, 4);
    amplitude = new DoubleSpinBox(general_parameters->LastRowLastCol(), "Amplitude", 0, 10, 0.01, 4, 0.436);
    z_rate = new DoubleSpinBox(general_parameters->NewRow(), "Z rate", 0, 10, 0.01, 4, 0.5);

    // Show cartesian errors plot
    plotCartesianErrors(gui_quadsmc->NewRow());

    AddDataToLog(state);
}

MyTrajectory::~MyTrajectory()
{
    delete state;
}

void MyTrajectory::UpdateFrom(const io_data *data)
{
    float current_time = (float(GetTime())/1000000000.0F) - initial_time;

    auto amplitude_value = (float)amplitude->Value();
    auto z_rate_value = (float)z_rate->Value();

    if(deltaT_custom->Value() == 0)
    {
        delta_t = (float)(data->DataDeltaTime())/1000000000.0F;
    }
    else
    {
        delta_t = (float)deltaT_custom->Value();
    }
   
    if(first_update)
    {
        initial_time = float(GetTime())/1000000000.0F;
        first_update = false;
    }

    Vector3Df desired_position;
    Vector3Df desired_velocity;
    Vector3Df desired_acceleration;


    // ToDo: Compute custom path planning. 
    /* 
    
    */

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
    state->GetMutex();
    state->SetValue(0, 0, desired_position.x);
    state->SetValue(1, 0, desired_position.y);
    state->SetValue(2, 0, desired_position.z);
    state->ReleaseMutex();

    ProcessUpdate(output);
}

void MyTrajectory::Reset(void)
{
    first_update = true;
}

void MyTrajectory::SetValues(float misc)
{
    // Set the input values for the path planner. Now it only receives a misc variable that can be used to set any value you want. This function is called from the main controller to set the input values.
    input->GetMutex();
    input->SetValue(0, 0, misc);

    input->ReleaseMutex();
}

void MyTrajectory::plotCartesianErrors(const LayoutPosition *position)
{
    // Example of how to plot the desired position in the GUI. 
    // Any variable that is defined in the state matrix can be plotted. Just remember to set its value in the UpdateFrom function and to add it to the log_labels matrix in the constructor.
    auto *plot = new DataPlot1D(position, "Desired cartesian position", -3, 3);
    plot->AddCurve(state->Element(0), DataPlot::Red);   // desired x
    plot->AddCurve(state->Element(1), DataPlot::Black); // desired y
    plot->AddCurve(state->Element(2), DataPlot::Blue);  // desired z
}