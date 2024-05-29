//
// Created by Harshavardhan Karnati on 28/03/2024.
//

#ifndef CONCEPTUAL_PROPULSION_SYSTEM_NEW_H
#define CONCEPTUAL_PROPULSION_SYSTEM_NEW_H

#include <iostream>
#include <utility>
#include <vector>
#include <iterator>
#include <array>
#include <string>
#include <memory>
#include <algorithm>
#include <utility>
#include <numbers>
#include <type_traits>
#include <cmath>
#include <concepts>

#include "../operationalPoint.h"
#include "../forces/weights.h"
#include "../unsupported/meta_checks.h"
#include "../unsupported/useful_expressions.h"
#include "../unsupported/debug_utils.h"

namespace concpt {
    namespace declare {
        struct create_propeller_array;
        struct detailed_output;
    }

    class propulsion_system : public std::enable_shared_from_this<propulsion_system> {
    public:
        friend class concpt::propulsion_system_weight;
        propulsion_system () = default;
        virtual ~propulsion_system () = default;

        template<class... propellerArrayType>
        requires (std::is_same_v<typename meta_checks::remove_all_qual<propellerArrayType>::element_type, concpt::declare::create_propeller_array> && ...)
        explicit propulsion_system (const std::shared_ptr<::operationalPoint> &IN_OPERATIONAL_POINT, propellerArrayType&&... IN_ARRAY_OF_PROPELLERS);

        propulsion_system (concpt::propulsion_system&& other) noexcept;

        void connect_propeller();
        std::shared_ptr<concpt::propulsion_system> get_shared_ptr ();
        void set_required_thrust (const float &IN_X_THRUST, const float &IN_Z_THRUST) const noexcept;
        [[nodiscard]] float get_power_required (const std::shared_ptr<concpt::declare::detailed_output> &IN_OUTPUT = nullptr, const std::string &IN_MESSAGE = "") const;
        float get_acoustic_dB (const float &IN_OBSERVER_DISTANCE, const float &IN_ELEVATION_ANGLE) const;

        std::vector<float> get_pitch_for_all_propellers () const noexcept;
        std::vector<float> get_rpm_for_all_propellers () const noexcept;
        std::vector<float> get_error_for_all_propellers () const noexcept;
        std::shared_ptr<operationalPoint_h::operationalPoint> atmosphere = nullptr;
        concpt::propulsion_system_weight weight = concpt::propulsion_system_weight(*this);
    private:
        std::vector<std::shared_ptr<concpt::declare::create_propeller_array>> array_of_propellers;
        std::size_t number_of_propellers{};
        std::size_t get_number_of_propellers () const noexcept;
    };

    namespace impl {
        void generate_propulsion_system_design_space ();
        void detailed_analysis();
        void hover_performance_analysis ();
    }
}

#endif //CONCEPTUAL_PROPULSION_SYSTEM_NEW_H
