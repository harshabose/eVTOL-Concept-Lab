//
// Created by Harshavardhan Karnati on 23/05/2024.
//

#include "../../include/wing/wing_tool.h"

concpt::wing::wing (const std::shared_ptr<concpt::operational_point> &IN_ATMOSPHERE,
                    const std::shared_ptr<concpt::declare::wing_details> &IN_WING_DETAILS, std::mutex &IN_MUX) :
        atmosphere(IN_ATMOSPHERE), wing_details(IN_WING_DETAILS), container(),
        mux(std::shared_ptr<std::mutex>(&IN_MUX, [](std::mutex*) {})) {}

void concpt::wing::calculate_aerodynamics () {
    std::scoped_lock lock(this->mux.operator*());
    this->container = concpt::detail::aerosandbox_solver(this);
}

void concpt::wing::connect_to_plane (const std::shared_ptr<concpt::plane> &IN_PLANE) {
    this->plane = IN_PLANE;
}

template<typename type>
requires (concpt::detail::check_input_f<type> || concpt::detail::check_input_arr<type>) type
concpt::wing::results (concpt::result_name KEY) {
    return this->container.operator[]<type>(KEY);
}


//********************************************************************************//
//************************************BUILDERS************************************//
//********************************************************************************//



