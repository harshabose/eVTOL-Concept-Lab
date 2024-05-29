//
// Created by Harshavardhan Karnati on 22/05/2024.
//


#include "../../include/forces/forces.h"

void concpt::plane_forces_visitor::resolve_force (const concpt::plane_force &IN_PLANE) {

}
void concpt::plane_forces_visitor::resolve_force (const concpt::wing_force &IN_WING) {
    std::array<float, 3> force_vector = IN_WING.get_force_vector();
    std::array<float, 3> moment_vector = IN_WING.get_moment_vector();
    std::array<float, 3> position_vector = IN_WING.get_position();
}
void concpt::plane_forces_visitor::resolve_force (const concpt::propulsion_system_force &IN_POPULSION_SYSTEM) {

}
void concpt::plane_forces_visitor::resolve_force (const concpt::propeller_force &IN_PROPELLER) {

}

