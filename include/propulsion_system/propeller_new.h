//
// Created by Harshavardhan Karnati on 27/05/2024.
//

#ifndef CONCEPTUAL_PROPELLER_NEW_H
#define CONCEPTUAL_PROPELLER_NEW_H

#include <iostream>
#include <vector>
#include <array>

#include "../includes.h"
#include "propulsion_system_new.h"
#include "propeller_blade_details.h"
#include "propeller_controller.h"
#include "propeller_theories.h"
#include "../forces/weights.h"
#include "../forces/propeller_forces.h"


namespace concpt {

    namespace {
        struct solution {
            float power_required;
            float torque;
            float thrust;
            double root_pitch;
            double rpm;
            float sound_pressure_level;
        };
    }

    class propeller {
    public:
        friend class concpt::propeller_weight;
        friend class concpt::propeller_force;

        void solve () {
            for (const auto &theory : this->theories) {
                theory->solve(this);
            }
        }

        mutable concpt::solution solution;
    private:
        std::weak_ptr<concpt::propulsion_system> propulsion_system;
        concpt::detail::blade_details blade;
        concpt::detail::propeller_controller_settings controller;
        concpt::propeller_weight weight = concpt::propeller_weight(*this);
        concpt::propeller_force forces = concpt::propeller_force(*this);
        std::vector<std::unique_ptr<concpt::detail::propeller_theory>> theories;
    };
}

#endif //CONCEPTUAL_PROPELLER_NEW_H
