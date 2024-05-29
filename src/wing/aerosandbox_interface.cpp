//
// Created by Harshavardhan Karnati on 19/05/2024.
//


#include "../../include/wing/aerosandbox_interface.h"
#include "../../include/wing/wing_tool.h"
#include "../../include/plane.h"

concpt::detail::aerosandbox_interface::aerosandbox_interface (const concpt::wing *IN_WING) :
    wing(IN_WING), container() {
    this->get_wing_xsecs_py();
    this->get_wing_py();
    this->translate_wing_py();
    this->get_reference_area();
    this->get_reference_lenght();
    this->get_angles_dynamic_pressure();
    this->get_atmosphere();

    if (this->check_if_standalone_wing()) {
        this->get_operational_point_standalone_wing();
    } else {
        this->get_operational_point();
    }

    this->get_solver();
}

void concpt::detail::aerosandbox_interface::get_wing_xsecs_py () {
    for (const auto &section : this->wing->wing_details->sectional_data) {
        py::object airfoil_name_py = this->asb.attr("Airfoil")(py::cast(section->airfoil_name));
        std::vector<double> xyz = {-0.25 * section->chord + section->sweep_offset, section->position, section->dihederal_offset};

        this->aerosandbox_vars.wing_xsecs_py.append(this->asb.attr("WingXSec")(
                py::arg("xyz_le") = py::list(py::cast(xyz)),
                py::arg("chord") = py::cast(static_cast<double>(section->chord)),
                py::arg("twist") = py::cast(static_cast<double>(section->angle_of_attack)),
                py::arg("airfoil") = airfoil_name_py
        ));
    }
}

void concpt::detail::aerosandbox_interface::get_wing_py () {
    this->aerosandbox_vars.wing_py = this->asb.attr("Wing")(
            py::arg("name") = "Front Wing",
            py::arg("symmetric") = py::cast(this->wing->wing_details->symmetric),
            py::arg("xsecs") = this->aerosandbox_vars.wing_xsecs_py
    );
}

void concpt::detail::aerosandbox_interface::translate_wing_py () {
    this->aerosandbox_vars.wing_py = this->aerosandbox_vars.wing_py.attr("translate")(
            py::arg("xyz") = py::list(py::cast(
                    std::vector<double>{static_cast<double>(this->wing->wing_details->aerodynamic_centre), 0, 0})
            )
    );
}

void concpt::detail::aerosandbox_interface::get_reference_area () {
    if (this->wing->wing_details->reference_surface_area == 0.0f)
        this->wing->wing_details->reference_surface_area = (this->aerosandbox_vars.wing_py.attr("area")()).cast<float>();
}

void concpt::detail::aerosandbox_interface::get_reference_lenght () {
    if (this->wing->wing_details->reference_length == 0.0f)
        this->wing->wing_details->reference_length = (this->aerosandbox_vars.wing_py.attr("mean_aerodynamic_chord")()).cast<float>();
}

py::object concpt::detail::aerosandbox_interface::get_plane_py () {
    return this->asb.attr("Airplane")(
            py::arg("name") = "plane",
            py::arg("xyz_ref") = py::list(py::cast(std::vector<double>{this->wing->wing_details->centre_of_gravity, 0, 0})),
            py::arg("s_ref") = py::cast(static_cast<double>(this->wing->wing_details->reference_surface_area)),
            py::arg("c_ref") = py::cast(static_cast<double>(this->wing->wing_details->reference_length)),
            py::arg("wings") = py::list(py::cast(std::vector<py::object>{this->aerosandbox_vars.wing_py}))
    );
}

bool concpt::detail::aerosandbox_interface::check_if_standalone_wing () noexcept {
    return this->wing->plane.expired();
}

void concpt::detail::aerosandbox_interface::get_angles_dynamic_pressure () {
    this->aerosandbox_vars.angle_of_attack = -1.0f * this->wing->atmosphere->get(concpt::operational_params::angle_of_attack);
    this->aerosandbox_vars.side_slip_angle = this->wing->atmosphere->get(concpt::operational_params::angle_of_sideslip);
    this->aerosandbox_vars.dynamic_pressure = this->wing->atmosphere->get(concpt::operational_params::dynamic_pressure);
}

void concpt::detail::aerosandbox_interface::get_atmosphere () {
    this->aerosandbox_vars.atmosphere_py =  asb.attr("Atmosphere")(
            py::arg("altitude") = py::cast(static_cast<double>(this->wing->atmosphere->get(concpt::operational_params::altitude)))
    );
}

void concpt::detail::aerosandbox_interface::get_operational_point_standalone_wing () {
    this->aerosandbox_vars.operational_point =  asb.attr("OperatingPoint")(
            py::arg("atmosphere") = this->aerosandbox_vars.atmosphere_py,
            py::arg("velocity") = py::cast(static_cast<double>(this->wing->atmosphere->get(concpt::operational_params::velocity))),
            py::arg("alpha") = py::cast(static_cast<double>(this->aerosandbox_vars.angle_of_attack)),
            py::arg("beta") = py::cast(static_cast<double>(this->aerosandbox_vars.side_slip_angle))
    );
}

void concpt::detail::aerosandbox_interface::get_operational_point () {
    if (const auto plane_ = this->wing->plane.lock()) {
        this->aerosandbox_vars.operational_point =  asb.attr("OperatingPoint")(
                py::arg("atmosphere") = this->aerosandbox_vars.atmosphere_py,
                py::arg("velocity") = py::cast(static_cast<double>(plane_->get_flight_state(concpt::flight_states::VELOCITY))),
                py::arg("alpha") = py::cast(static_cast<double>(-1.0f * plane_->get_flight_state(concpt::flight_states::PITCH_ANGLE) +
                        this->aerosandbox_vars.angle_of_attack)),
                py::arg("beta") = py::cast(static_cast<double>(plane_->get_flight_state(concpt::flight_states::YAW_ANGLE) +
                        this->aerosandbox_vars.side_slip_angle)),
                py::arg("p") = py::cast(static_cast<double>(plane_->get_flight_state(concpt::flight_states::ROLL_RATE))),
                py::arg("q") = py::cast(static_cast<double>(plane_->get_flight_state(concpt::flight_states::PITCH_RATE))),
                py::arg("r") = py::cast(static_cast<double>(plane_->get_flight_state(concpt::flight_states::YAW_RATE)))
        );
    } else
        throw std::bad_weak_ptr();
}

void concpt::detail::aerosandbox_interface::get_solver () {
    this->aerosandbox_vars.solver = this->asb.attr("AeroBuildup")(
            py::arg("airplane") = this->aerosandbox_vars.plane_py,
            py::arg("op_point") = this->aerosandbox_vars.operational_point
    );
}

concpt::detail::aerosandbox_result_container concpt::detail::aerosandbox_interface::solve () {
    const auto aspect_ratio = this->aerosandbox_vars.wing_py.attr("aspect_ratio")().cast<float>();
    py::object results_py = this->aerosandbox_vars.solver.attr("run_with_stability_derivatives")();

    concpt::detail::aerosandbox_result results = concpt::detail::aerosandbox_result(results_py, this->container,
                      dynamic_pressure, this->wing->wing_details->reference_surface_area, aspect_ratio);
    results.parse();
    return this->container;
}


