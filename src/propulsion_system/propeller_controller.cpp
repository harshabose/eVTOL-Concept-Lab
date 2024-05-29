//
// Created by Harshavardhan Karnati on 27/05/2024.
//

#include "../../include/propulsion_system/propeller_controller.h"

concpt::detail::propeller_controller_settings::propeller_controller_settings (concpt::detail::propeller_control_var IN_CONTROLLER) :
    control_var(IN_CONTROLLER) {}



// SET_PITCH_BOUNDS
concpt::declare::set_pitch_bounds::set_pitch_bounds (concpt::declare::use_pitch *IN_BUILDER) :
    concpt::declare::set_bounds_interface<concpt::declare::use_pitch>(IN_BUILDER) {}
concpt::declare::use_pitch& concpt::declare::set_pitch_bounds::set_lower_bound (const double &IN_VALUE) {
    this->builder->settings.lower_bounds[0] = IN_VALUE;
    return *this->builder;
}
concpt::declare::use_pitch& concpt::declare::set_pitch_bounds::set_upper_bound (const double &IN_VALUE) {
    this->builder->settings.upper_bounds[0] = IN_VALUE;
    return *this->builder;
}


// SET_RPM_BOUNDS
concpt::declare::set_rpm_bounds::set_rpm_bounds (concpt::declare::use_rpm *IN_BUILDER) :
    concpt::declare::set_bounds_interface<concpt::declare::use_rpm>(IN_BUILDER) {}
concpt::declare::use_rpm& concpt::declare::set_rpm_bounds::set_lower_bound (const double &IN_VALUE) {
    this->builder->settings.lower_bounds[0] = IN_VALUE;
    return *this->builder;
}
concpt::declare::use_rpm& concpt::declare::set_rpm_bounds::set_upper_bound (const double &IN_VALUE) {
    this->builder->settings.upper_bounds[0] = IN_VALUE;
    return *this->builder;
}


// SET_PITCH_AND_RPM_BOUNDS_PITCH
concpt::declare::set_pitch_and_rpm_bounds_pitch::set_pitch_and_rpm_bounds_pitch
    (concpt::declare::use_pitch_and_rpm *IN_BUILDER) :
    concpt::declare::set_bounds_interface<concpt::declare::use_pitch_and_rpm>(IN_BUILDER) {}
concpt::declare::use_pitch_and_rpm&
concpt::declare::set_pitch_and_rpm_bounds_pitch::set_lower_bound (const double &IN_VALUE) {
    this->builder->settings.lower_bounds[0] = IN_VALUE;
    this->builder->pitch_lower_bounds_set = true;
    return *this->builder;
}
concpt::declare::use_pitch_and_rpm &
concpt::declare::set_pitch_and_rpm_bounds_pitch::set_upper_bound (const double &IN_VALUE) {
    this->builder->settings.upper_bounds[0] = IN_VALUE;
    this->builder->pitch_upper_bounds_set = true;
    return *this->builder;
}


// SET_PITCH_AND_RPM_BOUNDS_RPM
concpt::declare::set_pitch_and_rpm_bounds_rpm::set_pitch_and_rpm_bounds_rpm (
        concpt::declare::use_pitch_and_rpm *IN_BUILDER) :
        concpt::declare::set_bounds_interface<concpt::declare::use_pitch_and_rpm>(IN_BUILDER) {}
concpt::declare::use_pitch_and_rpm &
concpt::declare::set_pitch_and_rpm_bounds_rpm::set_lower_bound (const double &IN_VALUE) {
    this->builder->settings.lower_bounds[1] = IN_VALUE;
    this->builder->rpm_lower_bounds_set = true;
    return *this->builder;
}
concpt::declare::use_pitch_and_rpm &
concpt::declare::set_pitch_and_rpm_bounds_rpm::set_upper_bound (const double &IN_VALUE) {
    this->builder->settings.upper_bounds[1] = IN_VALUE;
    this->builder->rpm_upper_bounds_set = true;
    return *this->builder;
}

// USE_PITCH
concpt::declare::use_pitch::use_pitch () : settings(detail::PITCH) {}
concpt::declare::use_pitch& concpt::declare::use_pitch::set_pitch (const float &IN_VALUE) {
    concpt::detail::propeller_controller_settings::nlopt_arr[0] = IN_VALUE;
    this->settings.current_pitch = &concpt::detail::propeller_controller_settings::nlopt_arr[0];
    return *this;
}
concpt::declare::use_pitch& concpt::declare::use_pitch::set_rpm (const float &IN_VALUE) {
    concpt::detail::propeller_controller_settings::nlopt_arr[1] = IN_VALUE;
    this->settings.current_rpm = &concpt::detail::propeller_controller_settings::nlopt_arr[1];
    return *this;
}
bool concpt::declare::use_pitch::perform_checks () const {
    if (this->pitch_set && this->rpm_set)
        if (this->lower_bound_set && this->upper_bound_set)
            return false;
    return true;
}
concpt::detail::propeller_controller_settings concpt::declare::use_pitch::build () const {
    if (this->perform_checks())
        throw std::runtime_error("Unconstrained controller inputs...");
    return this->settings;
}


// USE_RPM
concpt::declare::use_rpm::use_rpm() : settings(detail::RPM) {}
concpt::declare::use_rpm& concpt::declare::use_rpm::set_rpm (const float &IN_VALUE) {
    concpt::detail::propeller_controller_settings::nlopt_arr[0] = IN_VALUE;
    this->settings.current_rpm = &concpt::detail::propeller_controller_settings::nlopt_arr[0];
    return *this;
}
concpt::declare::use_rpm& concpt::declare::use_rpm::set_pitch (const float &IN_VALUE) {
    concpt::detail::propeller_controller_settings::nlopt_arr[1] = IN_VALUE;
    this->settings.current_pitch = &concpt::detail::propeller_controller_settings::nlopt_arr[1];
    return *this;
}
bool concpt::declare::use_rpm::perform_checks () const {
    if (this->pitch_set && this->rpm_set)
        if (this->lower_bound_set && this->upper_bound_set)
            return false;
    return true;
}
concpt::detail::propeller_controller_settings concpt::declare::use_rpm::build () const {
    if (this->perform_checks())
        throw std::runtime_error("Unconstrained controller inputs...");
    return this->settings;
}


// USE_PITCH_AND_RPM
concpt::declare::use_pitch_and_rpm::use_pitch_and_rpm () : settings(detail::BOTH){}
concpt::declare::use_pitch_and_rpm& concpt::declare::use_pitch_and_rpm::set_pitch (const float &IN_VALUE) {
    concpt::detail::propeller_controller_settings::nlopt_arr[0] = IN_VALUE;
    this->settings.current_pitch = &concpt::detail::propeller_controller_settings::nlopt_arr[0];
    return *this;
}
concpt::declare::use_pitch_and_rpm& concpt::declare::use_pitch_and_rpm::set_rpm (const float &IN_VALUE) {
    concpt::detail::propeller_controller_settings::nlopt_arr[1] = IN_VALUE;
    this->settings.current_rpm = &concpt::detail::propeller_controller_settings::nlopt_arr[1];
    return *this;
}
bool concpt::declare::use_pitch_and_rpm::perform_checks () const {
    if (this->pitch_set && this->rpm_set)
        if (this->settings.current_pitch && this->settings.current_rpm)
            if (this->pitch_lower_bounds_set && this->pitch_upper_bounds_set)
                if (this->rpm_lower_bounds_set && this->rpm_upper_bounds_set)
                    return false;
    return true;
}
concpt::detail::propeller_controller_settings concpt::declare::use_pitch_and_rpm::build () const {
    if (this->perform_checks())
        throw std::runtime_error("Unconstrained controller inputs...");
    return this->settings;
}

