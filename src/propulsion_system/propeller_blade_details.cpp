//
// Created by Harshavardhan Karnati on 27/05/2024.
//

#include "../../include/propulsion_system/propeller_blade_details.h"

const concpt::detail::section_details& concpt::detail::blade_details::get_sectional_data (const float &IN_LOCATION) {
    this->section_details[this->find_blade_index(IN_LOCATION)];
}
concpt::detail::aero_results_get concpt::detail::blade_details::get_aero_results (const float &IN_LOCATION) {
    return {*this, IN_LOCATION};
}
std::size_t concpt::detail::blade_details::find_blade_index (const float &IN_BLADE_LOCATION) const {
    auto if_ahead = [&IN_BLADE_LOCATION] (const concpt::detail::section_details &IN_SECTION) -> bool {return IN_SECTION.location >= IN_BLADE_LOCATION;};
    return std::ranges::distance(this->section_details.begin(),
                                 std::ranges::next(std::ranges::find_if(this->section_details.begin(),
                                                                        section_details.end(),
                                                                        if_ahead),
                                                   -1
                                 )
    );
}
std::array<std::vector<std::pair<float, float>> *, 2>
concpt::detail::blade_details::get_airfoil_coordinates (const float &IN_BLADE_LOCATION) {
    const concpt::detail::section_details& current_section = this->section_details[this->find_blade_index(IN_BLADE_LOCATION)];

    std::vector<std::pair<float, float>> upper = this->airfoil_polar->hash_airfoil(current_section.airfoil_name).airfoil_coordinates_upper;
    std::vector<std::pair<float, float>> lower = this->airfoil_polar->hash_airfoil(current_section.airfoil_name).airfoil_coordinates_lower;

    if (upper.empty() || lower.empty())
            throw std::runtime_error("Airfoil coordinates are not available...");

    return {&(this->airfoil_polar->hash_airfoil(current_section.airfoil_name).airfoil_coordinates_upper),
            &(this->airfoil_polar->hash_airfoil(current_section.airfoil_name).airfoil_coordinates_lower)};
}
float concpt::detail::blade_details::get_max_thickness_ratio (const float &IN_BLADE_LOCATION) {
    const concpt::detail::section_details& current_section = this->section_details[this->find_blade_index(IN_BLADE_LOCATION)];
    const float max_thickeness = this->airfoil_polar->hash_airfoil(current_section.airfoil_name).max_thickness_ratio;
    if (max_thickeness == 0.0f)
        throw std::runtime_error("No thickness data exists for this airfoil");
    else
        return max_thickeness;
}


concpt::detail::aero_results_get::aero_results_get (const concpt::detail::blade_details &IN_BLADE,
                                                    const float &IN_LOCATION) :
    blade(IN_BLADE), location(IN_LOCATION) {}
concpt::detail::aero_results_get& concpt::detail::aero_results_get::set_alpha(const float &IN_ALPHA) {
    this->alpha = IN_ALPHA;
    return *this;
}
concpt::detail::aero_results_get& concpt::detail::aero_results_get::set_reynolds_number (const float &IN_RE) {
    this->renolds_number = IN_RE;
    return *this;
}
concpt::detail::aero_results_get& concpt::detail::aero_results_get::set_mach_number (const float &IN_MACH) {
    this->mach_number = IN_MACH;
    return *this;
}
concpt::detail::aero_results_get& concpt::detail::aero_results_get::set_multiplier (const float &IN_MULTIPLIER) {
    this->multiplier = IN_MULTIPLIER;
    return *this;
}
std::pair<float, float> concpt::detail::aero_results_get::get () {
    const concpt::detail::section_details &current_section = this->blade.section_details[this->blade.find_blade_index(this->location)];
    if (this->perform_checks(current_section))
        throw std::out_of_range("Reynolds Number out-of-range...");
    return this->blade.airfoil_polar->get_aero_values(current_section.airfoil_name, alpha, renolds_number, mach_number, multiplier);
}
bool concpt::detail::aero_results_get::perform_checks (const concpt::detail::section_details &IN_CURRENT_SECTION) {
    const float min_re = this->blade.airfoil_polar->hash_airfoil(IN_CURRENT_SECTION.airfoil_name).min_Re;
    const float max_re = this->blade.airfoil_polar->hash_airfoil(IN_CURRENT_SECTION.airfoil_name).max_Re;

    const float min_alpha = this->blade.airfoil_polar->hash_airfoil(IN_CURRENT_SECTION.airfoil_name).min_alpha;
    const float max_alpha = this->blade.airfoil_polar->hash_airfoil(IN_CURRENT_SECTION.airfoil_name).max_alpha;

    if ((this->renolds_number < min_re || this->renolds_number > max_re) ||
        (this->alpha < min_alpha || this->alpha > max_alpha))
        return true;
    return false;
}


concpt::declare::propeller_section_details_build::propeller_section_details_build () : section() {}
concpt::declare::propeller_section_details_build &
concpt::declare::propeller_section_details_build::set_location (const float &IN_LOCATION) noexcept {
    this->section.location = IN_LOCATION;
    this->location_set = true;
    return *this;
}
concpt::declare::propeller_section_details_build &
concpt::declare::propeller_section_details_build::set_airfoil (const std::string &IN_AIRFOIL_NAME) noexcept {
    this->section.airfoil_name = IN_AIRFOIL_NAME;
    return *this;
}
concpt::declare::propeller_section_details_build &
concpt::declare::propeller_section_details_build::set_chord (const float &IN_CHORD) noexcept {
    this->section.chord = IN_CHORD;
    return *this;
}
concpt::declare::propeller_section_details_build &
concpt::declare::propeller_section_details_build::set_sweep (const float &IN_SWEEP) noexcept {
    this->section.sweep = IN_SWEEP;
    return *this;
}
concpt::declare::propeller_section_details_build &
concpt::declare::propeller_section_details_build::set_offset (const float &IN_OFFSET) noexcept {
    this->section.offset = IN_OFFSET;
    return *this;
}
concpt::declare::propeller_section_details_build &
concpt::declare::propeller_section_details_build::set_twist (const float &IN_TIWST) noexcept {
    this->section.twist = IN_TIWST;
    return *this;
}
concpt::detail::section_details concpt::declare::propeller_section_details_build::build () noexcept {
    return this->section;
}