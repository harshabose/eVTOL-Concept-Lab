//
// Created by Harshavardhan Karnati on 26/05/2024.
//

#ifndef CONCEPTUAL_PROPULSION_SYSTEM_BUILD_H
#define CONCEPTUAL_PROPULSION_SYSTEM_BUILD_H

#include "../includes.h"
#include "propulsion_system_new.h"

namespace concpt::declare {
    struct propeller_params : public concpt::propeller {
        std::array<float, 3> propeller_euler_angles{};
        std::array<float, 3> propeller_position{};
    };


    class create_propeller_array {
        create_propeller_array () = default;
        virtual ~create_propeller_array () = default;

        create_propeller_array& add_propeller () {
            return *this;
        }

        std::shared_ptr<concpt::propulsion_system> build () {

        }

    private:
        std::vector<concpt::declare::propeller_params> propeller_array;
    };
}

#endif //CONCEPTUAL_PROPULSION_SYSTEM_BUILD_H
