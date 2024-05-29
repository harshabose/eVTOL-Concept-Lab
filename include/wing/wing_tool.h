//
// Created by Harshavardhan Karnati on 14/05/2024.
//

#ifndef CONCEPTUAL_WING_TOOL_H
#define CONCEPTUAL_WING_TOOL_H

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <numbers>

//#include "/opt/homebrew/opt/python@3.11/Frameworks/Python.framework/Versions/3.11/include/python3.11/Python.h"

#include "../plane.h"
#include "../forces/weights.h"
#include "aerosandbox_interface.h"
#include "../unsupported/meta_checks.h"
#include "../unsupported/useful_expressions.h"

namespace concpt::declare {
    class wing_details_build;
    class mesh_wing_section;
    class math_wing_section;
    class wing_section_build;

    namespace {
        std::mutex WING_MUTEX{};      // file level mutex

        class wing_section {
        public:
            friend class concpt::detail::aerosandbox_interface;
            wing_section () = default;
            virtual ~wing_section() = default;

            virtual wing_section& set_airfoil_name (const std::string &IN_AIRFOIL_NAME) noexcept = 0;
            virtual wing_section& set_position (const float &IN_POSITION) noexcept = 0;
        protected:
            std::string airfoil_name;
            float position = 0.0f;
            float chord = 0.0f;
            float angle_of_attack = 0.0f;
            float sweep_offset = 0.0f;
            float dihederal_offset = 0.0f;
        };

        class wing_details {
        public:
            friend class concpt::declare::wing_details_build;
            friend class concpt::wing;
            friend class concpt::wing_weight;
            friend class concpt::detail::aerosandbox_interface;

            wing_details () = default;

            virtual ~wing_details () = default;
        protected:
            float aerodynamic_centre = 0.0f;
            float centre_of_gravity = 0.0f;
            float reference_surface_area = 0.0f;
            float reference_length = 0.0f;
            bool symmetric = true;
            std::vector<std::unique_ptr<concpt::declare::wing_section>> sectional_data;
            //maybe an instance of concpt::airfoil is needed?
        };
    }   // end of (anonymous) namespace
}

namespace concpt {
    class wing : public std::enable_shared_from_this<concpt::wing> {
    public:
        friend class concpt::detail::aerosandbox_interface;
        friend class concpt::wing_weight;

        wing () = default;
        virtual ~wing () = default;
        wing (const std::shared_ptr<concpt::operational_point> &IN_ATMOSPHERE,
              const std::shared_ptr<concpt::declare::wing_details> &IN_WING_DETAILS,
              std::mutex &IN_MUX = concpt::declare::WING_MUTEX);

        void calculate_aerodynamics ();
        void connect_to_plane (const std::shared_ptr<concpt::plane> &IN_PLANE);
        template <typename type> requires (concpt::detail::check_input_f<type> || concpt::detail::check_input_arr<type>)
        type results (concpt::result_name KEY);
        concpt::wing_weight weight = concpt::wing_weight(*this);
    private:
        std::shared_ptr<concpt::operational_point> atmosphere = nullptr;
        std::weak_ptr<concpt::plane> plane;
        std::shared_ptr<concpt::declare::wing_details> wing_details = nullptr;
        concpt::detail::aerosandbox_result_container container;
        std::shared_ptr<std::mutex> mux = nullptr;
    };
}   // end of concpt namespace
#endif //CONCEPTUAL_WING_TOOL_H
