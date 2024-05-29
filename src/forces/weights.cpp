//
// Created by Harshavardhan Karnati on 21/05/2024.
//

#include "../../include/forces/weights.h"
#include "../../include/useful_expressions/euler_angles.h"
#include "../../include/useful_expressions/element_wise.h"

void concpt::plane_weight_visitor::resolve_weight (const concpt::plane_weight &IN_PLANE) {
    std::array<float, 3> weight_vector = IN_PLANE.get_force_vector();
    std::array<float, 3> moment_vector = IN_PLANE.get_moment_vector();      // might have error. Imagine 3D arm
    std::array<float, 3> position_vector = IN_PLANE.get_position();

    std::array<float, 3> visitor_position_vector = this->plane_weight.get_position();
    std::array<float, 3> visitor_euler_angles = this->plane_weight.get_euler_angles();

    std::array<float, 3> distance = concpt::aux::element_wise(visitor_position_vector) - (position_vector);

    std::array<float, 3> new_moment = concpt::aux::element_wise(distance).cross_product(weight_vector);
    std::array<float, 3> total_moment = concpt::aux::element_wise(moment_vector) + (new_moment);

    const auto weight_in_current_axis = concpt::aux::euler_angle_operator<float>(weight_vector)
                .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
                .calc<std::array<float, 3>>();

    const auto moment_in_current_axis = concpt::aux::euler_angle_operator<float>(total_moment)
            .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
            .calc<std::array<float, 3>>();

}
void concpt::plane_weight_visitor::resolve_weight (const concpt::wing_weight &IN_PROP_SYSTEM) {
    std::array<float, 3> weight_vector = IN_PROP_SYSTEM.get_force_vector();
    std::array<float, 3> moment_vector = IN_PROP_SYSTEM.get_moment_vector();
    std::array<float, 3> position_vector = IN_PROP_SYSTEM.get_position();

    std::array<float, 3> visitor_position_vector = this->plane_weight.get_position();
    std::array<float, 3> visitor_euler_angles = this->plane_weight.get_euler_angles();

    std::array<float, 3> distance = concpt::aux::element_wise(visitor_position_vector) - (position_vector);

    std::array<float, 3> new_moment = concpt::aux::element_wise(distance).cross_product(weight_vector);
    std::array<float, 3> total_moment = concpt::aux::element_wise(moment_vector) + (new_moment);

    const auto weight_in_current_axis = concpt::aux::euler_angle_operator<float>(weight_vector)
            .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
            .calc<std::array<float, 3>>();

    const auto moment_in_current_axis = concpt::aux::euler_angle_operator<float>(total_moment)
            .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
            .calc<std::array<float, 3>>();
}
void concpt::plane_weight_visitor::resolve_weight (const concpt::propulsion_system_weight &IN_PROP_SYSTEM) {
    std::array<float, 3> weight_vector = IN_PROP_SYSTEM.get_force_vector();
    std::array<float, 3> moment_vector = IN_PROP_SYSTEM.get_moment_vector();
    std::array<float, 3> position_vector = IN_PROP_SYSTEM.get_position();

    std::array<float, 3> visitor_position_vector = this->plane_weight.get_position();
    std::array<float, 3> visitor_euler_angles = this->plane_weight.get_euler_angles();

    std::array<float, 3> distance = concpt::aux::element_wise(visitor_position_vector) - (position_vector);

    std::array<float, 3> new_moment = concpt::aux::element_wise(distance).cross_product(weight_vector);
    std::array<float, 3> total_moment = concpt::aux::element_wise(moment_vector) + (new_moment);

    const auto weight_in_current_axis = concpt::aux::euler_angle_operator<float>(weight_vector)
            .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
            .calc<std::array<float, 3>>();

    const auto moment_in_current_axis = concpt::aux::euler_angle_operator<float>(total_moment)
            .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
            .calc<std::array<float, 3>>();
}
void concpt::plane_weight_visitor::resolve_weight (const concpt::propeller_weight &IN_PROP) {
    std::array<float, 3> weight_vector = IN_PROP.get_force_vector();
    std::array<float, 3> moment_vector = IN_PROP.get_moment_vector();
    std::array<float, 3> position_vector = IN_PROP.get_position();

    std::array<float, 3> visitor_position_vector = this->plane_weight.get_position();
    std::array<float, 3> visitor_euler_angles = this->plane_weight.get_euler_angles();

    std::array<float, 3> distance = concpt::aux::element_wise(visitor_position_vector) - (position_vector);

    std::array<float, 3> new_moment = concpt::aux::element_wise(distance).cross_product(weight_vector);
    std::array<float, 3> total_moment = concpt::aux::element_wise(moment_vector) + (new_moment);

    const auto weight_in_current_axis = concpt::aux::euler_angle_operator<float>(weight_vector)
            .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
            .calc<std::array<float, 3>>();

    const auto moment_in_current_axis = concpt::aux::euler_angle_operator<float>(total_moment)
            .set_psi(visitor_euler_angles[0]).set_theta(visitor_euler_angles[1]).set_psi(visitor_euler_angles[2])
            .calc<std::array<float, 3>>();
}

#include "../../include/plane.h"
#include "../../include/wing/wing_tool.h"
#include "../../include/propulsion_system/propulsion_system_new.h"
#include "../../include/propulsion_system/propeller.h"

std::array<float, 3> concpt::plane_weight::get_force_vector () const {
    const float weight = 0.0f;
    std::array<float, 3> weight_vector = {0.0f, 0.0f, weight};
    for (const auto &each_wing : this->component.array_of_wings) {
        concpt::aux::element_wise(weight_vector) += each_wing->weight.get_force_vector();
    }
    concpt::aux::element_wise(weight_vector) += this->component.propulsion_system->weight.get_force_vector();
    return weight_vector;
}

//TODO: FINISH THE MOMENTS AND FORCES

std::array<float, 3> concpt::plane_weight::get_moment_vector () const {
    std::array<float, 3> moment_vector = {0.0f, 0.0f, 0.0f};
    for (const auto &each_wing : this->component.array_of_wings) {
        concpt::aux::element_wise(moment_vector) += each_wing->weight.get_moment_vector();
    }
    concpt::aux::element_wise(moment_vector) += this->component.propulsion_system->weight.get_moment_vector();
    return moment_vector;
}

std::array<float, 3> concpt::plane_weight::get_position () const {
    return std::array<float, 3>{this->component.centre_of_gravity, 0.0f, 0.0f};
}

std::array<float, 3> concpt::plane_weight::get_euler_angles () const {
    return std::array<float, 3>{this->component.get_flight_state(concpt::flight_states::YAW_ANGLE),
    this->component.get_flight_state(concpt::flight_states::PITCH_ANGLE),
    this->component.get_flight_state(concpt::flight_states::ROLL_ANGLE)};
}

std::array<float, 3> concpt::plane_weight::get_rotaional_inertial () const {
    throw std::runtime_error("'get_rotaional_inertial' Method not implemented");
}



std::array<float, 3> concpt::wing_weight::get_force_vector () const {
    const float weight = 0.0f;
    return std::array<float, 3>{0.0f, 0.0f, weight};
}

std::array<float, 3> concpt::wing_weight::get_moment_vector () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::wing_weight::get_position () const {
    return std::array<float, 3>{this->component.wing_details->aerodynamic_centre -
        this->component.wing_details->centre_of_gravity, 0.0f, 0.0f};
}

std::array<float, 3> concpt::wing_weight::get_euler_angles () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::wing_weight::get_rotaional_inertial () const {
    throw std::runtime_error("'get_rotaional_inertial' Method not implemented");
}



std::array<float, 3> concpt::propulsion_system_weight::get_force_vector () const {
    std::array<float, 3> weight_vector = {0.0f, 0.0f, 0.0f};
    for (const auto &each_propeller : this->component.array_of_propellers) {
        std::array<float, 3> current_weight = concpt::aux::element_wise(each_propeller->propeller->weight.get_force_vector()) *
                static_cast<float>(each_propeller->number_of_same_propellers);
        concpt::aux::element_wise(weight_vector) += current_weight;
    }
    return weight_vector;
}

std::array<float, 3> concpt::propulsion_system_weight::get_moment_vector () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::propulsion_system_weight::get_position () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::propulsion_system_weight::get_euler_angles () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::propulsion_system_weight::get_rotaional_inertial () const {
    throw std::runtime_error("'get_rotaional_inertial' Method not implemented");
}




std::array<float, 3> concpt::propeller_weight::get_force_vector () const {
    const float weight = 0.0f;
    return std::array<float, 3>{0.0f, 0.0f, weight};
}

std::array<float, 3> concpt::propeller_weight::get_moment_vector () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::propeller_weight::get_position () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::propeller_weight::get_euler_angles () const {
    return std::array<float, 3>{0.0f, 0.0f, 0.0f};
}

std::array<float, 3> concpt::propeller_weight::get_rotaional_inertial () const {
    throw std::runtime_error("'get_rotaional_inertial' Method not implemented");
}