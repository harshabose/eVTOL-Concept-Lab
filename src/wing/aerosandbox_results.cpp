//
// Created by Harshavardhan Karnati on 24/05/2024.
//

#include "../../include/wing/aerosandbox_results.h"

concpt::detail::aerosandbox_result::aerosandbox_result (const py::dict &IN_DICT, result_container& IN_CONTAINER, const float &IN_DYNAMIC_PRESSURE,
                                    const float &IN_SURFACE_AREA, const float &IN_ASPECT_RATIO) :
        asd_dict(IN_DICT), dynamic_pressure(IN_DYNAMIC_PRESSURE), surface_area(IN_SURFACE_AREA),
        aspect_ratio(IN_ASPECT_RATIO), container(IN_CONTAINER) {}

concpt::detail::aerosandbox_result::result_container concpt::detail::aerosandbox_result::parse () {
    for (std::size_t key = 0; key < concpt::result_name::WING_AERO_LAST; key++) {
        this->pick_parse_method(key);
    }
    return std::move(this->container);
}

void concpt::detail::aerosandbox_result::pick_parse_method (const std::size_t &IN_KEY) {
    float oswald_factor, induced_factor;
    {
        py::list wing_aero_components = this->asd_dict["wing_aero_components"];
        py::object first_element = wing_aero_components[0];
        oswald_factor = first_element.attr(py::cast(this->keys_alias[IN_KEY])).cast<float>();
        induced_factor = std::numbers::pi_v<float> * this->aspect_ratio * oswald_factor;
    }

    switch (IN_KEY) {
        case 0: case 1: case 2: case 3: case 4: case 5:
            this->parse_array(IN_KEY);
            break;
        case 51: case 52:
            this->parse_float(IN_KEY);
            this->container.operator[]<float>(IN_KEY) = this->container.operator[]<float>(IN_KEY) /
                                                        (this->dynamic_pressure * this->surface_area);
            break;
        case 53:
            this->parse_float(IN_KEY, oswald_factor);
            break;
        case 54:
            this->parse_float(IN_KEY, 1.0f / induced_factor);
            break;
        default:
            this->parse_float(IN_KEY);
    }
}

void concpt::detail::aerosandbox_result::parse_float (const std::size_t &IN_KEY) {
    const auto value = this->asd_dict[py::cast(this->keys_alias[IN_KEY])].cast<float>();
    this->container.input_data(IN_KEY, value);
}

void concpt::detail::aerosandbox_result::parse_float (const std::size_t &IN_KEY, const float &IN_VALUE) {
    this->container.input_data(IN_KEY, IN_VALUE);
}

void concpt::detail::aerosandbox_result::parse_array (const std::size_t &IN_KEY) {
    const auto value = this->asd_dict[py::cast(this->keys_alias[IN_KEY])].cast<float>();
    this->asd_dict[py::cast(IN_KEY)].cast<std::array<float, 3>>();
}