//
// Created by Harshavardhan Karnati on 25/05/2024.
//

#ifndef CONCEPTUAL_WING_BUILDERS_H
#define CONCEPTUAL_WING_BUILDERS_H

#include "wing_tool.h"

namespace concpt::declare {
    class mesh_wing_section : public concpt::declare::wing_section {
    public:
        mesh_wing_section () = default;

        mesh_wing_section& set_airfoil_name (const std::string &IN_AIRFOIL_NAME) noexcept override;
        mesh_wing_section& set_position (const float &IN_POSITION) noexcept override;
        mesh_wing_section& set_chord (const float &IN_CHORD) noexcept;
        mesh_wing_section& set_angle_of_attack (const float &IN_AOA) noexcept;
        mesh_wing_section& set_sweep_offset (const float &IN_SWEEP_OFFSET) noexcept;
        mesh_wing_section& set_dihederal_offset (const float &IN_DIHEDERAL_OFFSET) noexcept;
        [[nodiscard]] std::unique_ptr<concpt::declare::wing_section> build ();
        void propagate_from (concpt::declare::mesh_wing_section* IN_PREV);

    private:
        bool set_position_b = false;
        bool propagate_airfoil_name = true;
        bool propagate_chord = true;
        bool propagate_angle_of_attack = true;
        bool propagate_sweep_offset = true;
        bool propagate_dihederal_offset = true;
    };

    class math_wing_section : public wing_section {
    public:
        math_wing_section () = default;
        math_wing_section& set_airfoil_name (const std::string &IN_AIRFOIL_NAME) noexcept override;
        math_wing_section& set_position (const float &IN_POSITION) noexcept override;
        math_wing_section& set_root_position (const float &IN_ROOT_POSITION) noexcept;
        math_wing_section& set_taper_ratio (const float &IN_TAPER_RATIO, const float &IN_ROOR_CHORD,
                                            const float &IN_HALF_SPAN) noexcept;
        math_wing_section& set_twist_ratio (const float &IN_TWIST_RATIO, const float &IN_ROOT_AOA) noexcept;
        math_wing_section& set_sweep_angle (const float &IN_SWEEP_ANGLE) noexcept;
        math_wing_section& set_dihederal_angle (const float &IN_DIHEDERAL_ANGLE) noexcept;
        void propagate_from (concpt::declare::math_wing_section* PREV) noexcept;
        [[nodiscard]] std::unique_ptr<concpt::declare::wing_section> build ();
    private:
        float root_position = 0.0f;
        float taper_ratio = 0.0f;
        float root_chord = 0.0f;
        float half_span = 1.0f;
        float twist_ratio = 0.0f;
        float root_angle_of_attack = 0.0f;
        float sweep_angle = 0.0f;
        float dihederal_angle = 0.0f;

        bool set_position_b = false;
        bool propagate_airfoil_name = true;
        bool propagate_root_position = true;
        bool propagate_taper_ratio = true;
        bool propagate_twist_ratio = true;
        bool propagate_sweep_angle = true;
        bool propagate_dihederal_angle = true;
    };

    class wing_section_build : public concpt::declare::mesh_wing_section, public concpt::declare::math_wing_section {
    public:
        wing_section_build() = default;

        mesh_wing_section& use_mesh_inputs () {
            return *this;
        };

        math_wing_section& use_mathematical_inputs () {
            return *this;
        };
    };

    class wing_details_build {
    public:
        wing_details_build () : wing_details(std::make_shared<concpt::declare::wing_details>()) {}
        wing_details_build& set_aerodynamic_centre (const float &value_) noexcept;
        wing_details_build& set_centre_of_gravity (const float &value_) noexcept;
        wing_details_build& set_reference_surface_area (const float &value_) noexcept;
        wing_details_build& set_reference_length (const float &value_) noexcept;
        wing_details_build& set_symmetric (const bool &value_) noexcept;
        [[nodiscard]] float get_aerodynamic_centre () const noexcept;

        template<class sectionType>
        requires(std::is_same_v<sectionType, concpt::declare::mesh_wing_section> ||
                 std::is_same_v<sectionType, concpt::declare::math_wing_section>)
        wing_details_build& add_section (sectionType &section_);
        [[nodiscard]] std::shared_ptr<concpt::declare::wing_details> build_shared () const;
    private:
        std::shared_ptr<concpt::declare::wing_details> wing_details;

        static void propagate_data (concpt::declare::mesh_wing_section *prev_,
                                    concpt::declare::mesh_wing_section &next_) noexcept;
        static void propagate_data (concpt::declare::math_wing_section *prev_,
                                    concpt::declare::mesh_wing_section &next_) noexcept;
        static void propagate_data (concpt::declare::math_wing_section *prev_,
                                    concpt::declare::math_wing_section &next_) noexcept;
        static void propagate_data (concpt::declare::mesh_wing_section *prev_,
                                    concpt::declare::math_wing_section &next_) noexcept;
        [[nodiscard]] bool perform_checks () const;
    };
}

#endif //CONCEPTUAL_WING_BUILDERS_H
