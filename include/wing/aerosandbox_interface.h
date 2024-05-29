//
// Created by Harshavardhan Karnati on 23/05/2024.
//

#ifndef CONCEPTUAL_AEROSANDBOX_INTERFACE_H
#define CONCEPTUAL_AEROSANDBOX_INTERFACE_H

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include "../includes.h"
#include "aerosandbox_results.h"


namespace concpt::detail {
    namespace {
        struct aerosandbox_vars {
            aerosandbox_vars () = default;
            py::list wing_xsecs_py;
            py::object wing_py;
            py::object plane_py;
            py::object atmosphere_py;
            py::object operational_point;
            float angle_of_attack = 0.0f;
            float side_slip_angle = 0.0f;
            float dynamic_pressure = 0.0f;
            py::object solver;
        };
    }

    class aerosandbox_interface {
    public:
        explicit aerosandbox_interface (const concpt::wing* IN_WING);
        virtual ~aerosandbox_interface () = default;

        [[nodiscard]] concpt::detail::aerosandbox_result_container solve ();
    private:
        py::module asb = py::module::import("aerosandbox");
        py::module np = py::module::import("aerosandbox.numpy");
        aerosandbox_vars aerosandbox_vars;

        const concpt::wing *wing;
        concpt::detail::aerosandbox_result_container container;

        void get_wing_xsecs_py ();

        void get_wing_py ();

        void translate_wing_py ();

        void get_reference_area ();

        void get_reference_lenght ();

        py::object get_plane_py ();

        bool check_if_standalone_wing () noexcept;

        void get_angles_dynamic_pressure ();

        void get_atmosphere ();

        void get_operational_point_standalone_wing ();

        void get_operational_point ();

        void get_solver ();
    };

    concpt::detail::aerosandbox_result_container aerosandbox_solver (const concpt::wing *IN_WING) {
        return concpt::detail::aerosandbox_interface(IN_WING).solve();
    }
}


#endif //CONCEPTUAL_AEROSANDBOX_INTERFACE_H
