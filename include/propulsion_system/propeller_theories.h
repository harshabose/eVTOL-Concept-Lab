//
// Created by Harshavardhan Karnati on 27/05/2024.
//

#ifndef CONCEPTUAL_PROPELLER_THEORIES_H
#define CONCEPTUAL_PROPELLER_THEORIES_H

#include <array>

#include "../includes.h"
#include "../nlopt/nlopt.h"

namespace concpt::detail {
    class propeller_theory {
    public:
        virtual void solve (const concpt::propeller* const propeller) = 0;
    private:
        concpt::propeller *propeller = nullptr;
    };

    class BEMT : public concpt::detail::propeller_theory {
    public:
        void solve(const concpt::propeller* const propeller) override;
    private:
        struct other_vars {
            float root_cut_off = 0.1f;
            float effective_area, area;
            float tip_loss_factor, speed_of_sound;
            float thrust_required, thrust_coeffecient;
            float max_thrust_coeffecient = 2.0f;
            std::vector<float>psi_angles_divisions, radius_divisions;
        } other_vars;
        struct nlopt_methods {
            static double nlopt_objective_function (unsigned n, const double *x, double *grad, void *data);
            static double nlopt_equality_constraint (unsigned n, const double *x, double *grad, void *data);
        } nlopt_methods;
        void set_BEMT_parameters ();
        void calculate_thrust_coeffecients ();
        std::array<float, 3> get_induced_velocity ();
        float get_instantaneous_inflow_velocity();
        std::array<float, 2> get_blade_section_aero ();
        void calculate_propeller_forces_moments();
    };

    class helicoidal_acoustics : public concpt::detail::propeller_theory {
    public:
        void solve(const concpt::propeller* const propeller) override;
    private:
    };
}

#endif //CONCEPTUAL_PROPELLER_THEORIES_H
