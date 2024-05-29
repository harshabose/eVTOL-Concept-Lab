//
// Created by Harshavardhan Karnati on 27/05/2024.
//

#ifndef CONCEPTUAL_PROPELLER_BLADE_DETAILS_H
#define CONCEPTUAL_PROPELLER_BLADE_DETAILS_H

#include <iostream>
#include <string>
#include <tuple>
#include <ranges>

#include "../airfoil.h"

namespace concpt::declare {

}

namespace concpt::detail {
    struct aero_results_get;

    namespace {
        struct section_details {
            float location, chord, sweep, offset, twist;
            std::string airfoil_name;
        };
    }

    class blade_details {
    public:
        friend struct aero_results_get;

        const section_details& get_sectional_data (const float &IN_LOCATION);
        aero_results_get get_aero_results (const float &IN_LOCATION);
        std::array<std::vector<std::pair<float, float>>*, 2> get_airfoil_coordinates (const float &IN_BLADE_LOCATION);
        float get_max_thickness_ratio (const float &IN_BLADE_LOCATION);
    private:
        std::vector<concpt::detail::section_details> section_details;
        std::shared_ptr<concpt::airfoil_polar>airfoil_polar = nullptr;
        std::size_t nodes_radius = 250, nodes_azimuthal = 1;

        [[nodiscard]] std::size_t find_blade_index (const float &IN_BLADE_LOCATION) const;
    };

    struct aero_results_get {
    public:
        aero_results_get (const concpt::detail::blade_details &IN_BLADE, const float &IN_LOCATION);

        aero_results_get& set_alpha (const float &IN_ALPHA);
        aero_results_get& set_reynolds_number (const float &IN_RE);
        aero_results_get& set_mach_number (const float &IN_MACH);
        aero_results_get& set_multiplier (const float &IN_MULTIPLIER);
        std::pair<float, float> get ();
    private:
        float alpha = 0.0f;
        float renolds_number = 1000000;
        float mach_number = 0.0f;
        float multiplier = 1.0f;
        const concpt::detail::blade_details &blade;
        const float &location;

        bool perform_checks (const concpt::detail::section_details &IN_CURRENT_SECTION);
    };
}

namespace concpt::declare {
    class propeller_section_details_build {
    public:
        propeller_section_details_build ();
        concpt::declare::propeller_section_details_build& set_location (const float &IN_LOCATION) noexcept;
        concpt::declare::propeller_section_details_build& set_airfoil (const std::string &IN_AIRFOIL_NAME) noexcept;
        concpt::declare::propeller_section_details_build& set_chord (const float &IN_CHORD) noexcept;
        concpt::declare::propeller_section_details_build& set_sweep (const float &IN_SWEEP) noexcept;
        concpt::declare::propeller_section_details_build& set_offset (const float &IN_OFFSET) noexcept;
        concpt::declare::propeller_section_details_build& set_twist (const float &IN_TIWST) noexcept;
        concpt::detail::section_details build () noexcept;
    private:
        bool location_set = false;
        concpt::detail::section_details section;
    };

    class propeller_blade_build {
    public:
        propeller_blade_build () : blade() {}
        concpt::declare::propeller_section_details_build& set_radius_nodes () noexcept;
        concpt::declare::propeller_section_details_build& set_azimuthal_nodes () noexcept;
        concpt::declare::propeller_section_details_build& add_section
            (const concpt::declare::propeller_section_details_build &IN_SECTION) noexcept;
        concpt::declare::propeller_section_details_build& add_airfoil_polars (bool IS_COMPLETE = true);
        concpt::detail::blade_details build ();
        std::shared_ptr<concpt::detail::blade_details> build_shared ();
    private:
        concpt::detail::blade_details blade;
        bool perform_checks () const noexcept;
    };
}

#endif //CONCEPTUAL_PROPELLER_BLADE_DETAILS_H
