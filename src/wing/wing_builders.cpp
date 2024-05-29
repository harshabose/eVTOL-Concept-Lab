//
// Created by Harshavardhan Karnati on 25/05/2024.
//

#include "../../include/wing/wing_builders.h"


concpt::declare::mesh_wing_section& concpt::declare::mesh_wing_section::set_airfoil_name (const std::string &IN_AIRFOIL_NAME) noexcept {
    this->propagate_airfoil_name = false;
    this->airfoil_name = IN_AIRFOIL_NAME;
    return *this;
}

concpt::declare::mesh_wing_section&  concpt::declare::mesh_wing_section::set_position (const float &IN_POSITION) noexcept {
    this->set_position_b = true;
    this->position = IN_POSITION;
    return *this;
}

concpt::declare::mesh_wing_section& concpt::declare::mesh_wing_section::set_chord (const float &IN_CHORD) noexcept {
    this->propagate_chord = false;
    this->chord = IN_CHORD;
    return *this;
}

concpt::declare::mesh_wing_section& concpt::declare::mesh_wing_section::set_angle_of_attack (const float &IN_AOA) noexcept {
    this->propagate_angle_of_attack = false;
    this->angle_of_attack = IN_AOA;
    return *this;
}

concpt::declare::mesh_wing_section& concpt::declare::mesh_wing_section::set_sweep_offset (const float &IN_SWEEP_OFFSET) noexcept {
    this->propagate_sweep_offset = false;
    this->sweep_offset = IN_SWEEP_OFFSET;
    return *this;
}

concpt::declare::mesh_wing_section& concpt::declare::mesh_wing_section::set_dihederal_offset (const float &IN_DIHEDERAL_OFFSET) noexcept {
    this->propagate_dihederal_offset = false;
    this->dihederal_offset = IN_DIHEDERAL_OFFSET;
    return *this;
}

std::unique_ptr<concpt::declare::wing_section> concpt::declare::mesh_wing_section::build () {
    if (this->set_position_b == false)
        throw std::invalid_argument("At least section position needs to inputted...");
    return std::make_unique<concpt::declare::mesh_wing_section>(*this);
}

void concpt::declare::mesh_wing_section::propagate_from (concpt::declare::mesh_wing_section* IN_PREV) {
    if (this->propagate_airfoil_name)
        this->airfoil_name = IN_PREV->airfoil_name;
    if (this->propagate_chord)
        this->chord = IN_PREV->chord;
    if (this->propagate_angle_of_attack)
        this->angle_of_attack = IN_PREV->angle_of_attack;
    if (this->propagate_sweep_offset)
        this->sweep_offset = IN_PREV->sweep_offset;
    if (this->propagate_dihederal_offset)
        this->dihederal_offset = IN_PREV->dihederal_offset;
}







concpt::declare::math_wing_section& concpt::declare::math_wing_section::set_airfoil_name (const std::string &IN_AIRFOIL_NAME) noexcept  {
    this->propagate_airfoil_name = false;
    this->airfoil_name = IN_AIRFOIL_NAME;
    return *this;
}

concpt::declare::math_wing_section& concpt::declare::math_wing_section::set_position (const float &IN_POSITION) noexcept {
    this->set_position_b = true;
    this->position = IN_POSITION;
    return *this;
}

concpt::declare::math_wing_section& concpt::declare::math_wing_section::set_root_position (const float &IN_ROOT_POSITION) noexcept {
    this->propagate_root_position = false;
    this->root_position = IN_ROOT_POSITION;
    return *this;
}

concpt::declare::math_wing_section& concpt::declare::math_wing_section::set_taper_ratio (const float &IN_TAPER_RATIO, const float &IN_ROOR_CHORD,
                                                                        const float &IN_HALF_SPAN) noexcept {
    this->propagate_taper_ratio = false;
    this->root_chord = IN_ROOR_CHORD;
    this->taper_ratio = IN_TAPER_RATIO;
    this->half_span = IN_HALF_SPAN;
    return *this;
}

concpt::declare::math_wing_section& concpt::declare::math_wing_section::set_twist_ratio (const float &IN_TWIST_RATIO, const float &IN_ROOT_AOA) noexcept {
    this->propagate_twist_ratio = false;
    this->twist_ratio = IN_TWIST_RATIO;
    this->root_angle_of_attack = IN_ROOT_AOA;
    return *this;
}

concpt::declare::math_wing_section& concpt::declare::math_wing_section::set_sweep_angle (const float &IN_SWEEP_ANGLE) noexcept {
    this->propagate_sweep_angle = false;
    this->sweep_angle = IN_SWEEP_ANGLE;
    return *this;
}

concpt::declare::math_wing_section& concpt::declare::math_wing_section::set_dihederal_angle (const float &IN_DIHEDERAL_ANGLE) noexcept {
    this->propagate_dihederal_angle = false;
    this->dihederal_angle = IN_DIHEDERAL_ANGLE;
    return *this;
}

void concpt::declare::math_wing_section::propagate_from (concpt::declare::math_wing_section* PREV) noexcept {
    if (this->propagate_airfoil_name)
        this->airfoil_name = PREV->airfoil_name;
    if (this->propagate_root_position)
        this->root_position = PREV->root_position;
    if (this->propagate_taper_ratio) {
        this->taper_ratio = PREV->taper_ratio;
        this->root_chord = PREV->root_chord;
        this->half_span = PREV->half_span;
    }
    if (this->propagate_twist_ratio) {
        this->twist_ratio = PREV->twist_ratio;
        this->root_angle_of_attack = PREV->angle_of_attack;
    }
    if (this->propagate_sweep_angle)
        this->sweep_angle = PREV->sweep_angle;
    if (this->propagate_dihederal_angle)
        this->dihederal_angle = PREV->dihederal_angle;
}

std::unique_ptr<concpt::declare::wing_section> concpt::declare::math_wing_section::build () {
    if (this->set_position_b == false)
        throw std::invalid_argument("At least position must be inputted...");

    this->chord = ((this->position - this->root_position) / this->half_span) * this->taper_ratio * this->root_chord;
    this->angle_of_attack = this->root_angle_of_attack + (this->position - this->root_position) * this->twist_ratio;
    this->sweep_offset = (this->position - this->root_position) *
                         std::tan(concpt::aux::rads_to_degrees(this->sweep_angle));
    this->dihederal_angle = (this->position - this->root_position) *
                            std::tan(concpt::aux::rads_to_degrees(this->dihederal_angle));

    return std::make_unique<concpt::declare::math_wing_section>(*this);
}







concpt::declare::wing_details_build& concpt::declare::wing_details_build::set_aerodynamic_centre (const float &value_) noexcept {
    this->wing_details->aerodynamic_centre = value_;
    return *this;
}

concpt::declare::wing_details_build& concpt::declare::wing_details_build::set_centre_of_gravity (const float &value_) noexcept {
    this->wing_details->centre_of_gravity = value_;
    return *this;
}

concpt::declare::wing_details_build& concpt::declare::wing_details_build::set_reference_surface_area (const float &value_) noexcept {
    this->wing_details->reference_surface_area = value_;
    return *this;
}

concpt::declare::wing_details_build& concpt::declare::wing_details_build::set_reference_length (const float &value_) noexcept {
    this->wing_details->reference_length = value_;
    return *this;
}

concpt::declare::wing_details_build& concpt::declare::wing_details_build::set_symmetric (const bool &value_) noexcept {
    this->wing_details->symmetric = value_;
    return *this;
}

float concpt::declare::wing_details_build::get_aerodynamic_centre () const noexcept {
    return this->wing_details->aerodynamic_centre;
}

template<class sectionType>
requires(std::is_same_v<sectionType, concpt::declare::mesh_wing_section> ||
         std::is_same_v<sectionType, concpt::declare::math_wing_section>)
concpt::declare::wing_details_build& concpt::declare::wing_details_build::add_section (sectionType &section_) {
    if (!this->wing_details->sectional_data.empty()) {
        if (const auto prev_ = dynamic_cast<concpt::declare::math_wing_section*>(this->wing_details->sectional_data.back().get())) {
            this->propagate_data(prev_, section_);
        }
        if (auto prev_ = dynamic_cast<concpt::declare::mesh_wing_section*>(this->wing_details->sectional_data.back().get())) {
            this->propagate_data(prev_, section_);
        }
    }
    this->wing_details->sectional_data.emplace_back(std::move(section_.build()));
    return *this;
}

std::shared_ptr<concpt::declare::wing_details>
concpt::declare::wing_details_build::build_shared () const {
    if (this->perform_checks()) {
        throw std::invalid_argument("Few parameters are missing to properly constraint the wing...");
    }
    return this->wing_details;
}

void concpt::declare::wing_details_build::propagate_data (concpt::declare::mesh_wing_section *prev_,
                                                                 concpt::declare::mesh_wing_section &next_) noexcept {
    next_.propagate_from(prev_);
}

void concpt::declare::wing_details_build::propagate_data (concpt::declare::math_wing_section *prev_,
                                                                 concpt::declare::mesh_wing_section &next_) noexcept {
    //next_.propagate_from(prev_);
    //TODO: implement this propagation mechanism
}

void concpt::declare::wing_details_build::propagate_data (concpt::declare::math_wing_section *prev_,
                                                                 concpt::declare::math_wing_section &next_) noexcept {
    next_.propagate_from(prev_);
}

void concpt::declare::wing_details_build::propagate_data (concpt::declare::mesh_wing_section *prev_,
                                                                 concpt::declare::math_wing_section &next_) noexcept {
    //next_.propagate_from(prev_);
    //TODO: implement this propagation mechanism
}

bool concpt::declare::wing_details_build::perform_checks () const {
    return false;
}