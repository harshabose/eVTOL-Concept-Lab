//
// Created by Harshavardhan Karnati on 24/05/2024.
//

#ifndef CONCEPTUAL_AEROSANDBOX_RESULTS_H
#define CONCEPTUAL_AEROSANDBOX_RESULTS_H

#include <iostream>
#include <numbers>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include "aerosandbox_results_container.h"

namespace py = pybind11;

namespace concpt::detail {
    class aerosandbox_result {
    public:
        using result_container = concpt::detail::aerosandbox_result_container;

        explicit aerosandbox_result (const py::dict &IN_DICT, result_container& IN_CONTAINER, const float &IN_DYNAMIC_PRESSURE,
                                     const float &IN_SURFACE_AREA, const float &IN_ASPECT_RATIO);

        concpt::detail::aerosandbox_result::result_container parse ();
    private:
        const py::dict &asd_dict;
        const float &dynamic_pressure;
        const float &surface_area;
        const float &aspect_ratio;
        const concpt::detail::aerosandbox_internal_keys_t& keys_alias = concpt::detail::aerosandbox_internal_keys;
        result_container &container;

        void pick_parse_method (const std::size_t& IN_KEY);

        void parse_float (const std::size_t& IN_KEY);

        void parse_float (const std::size_t& IN_KEY, const float &IN_VALUE);

        void parse_array (const std::size_t& IN_KEY);
    };
}

#endif //CONCEPTUAL_AEROSANDBOX_RESULTS_H
