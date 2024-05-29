//
// Created by Harshavardhan Karnati on 27/05/2024.
//

#ifndef CONCEPTUAL_PROPELLER_FORCES_H
#define CONCEPTUAL_PROPELLER_FORCES_H

#include "forces.h"


namespace concpt {
    class propeller_force : public concpt::forces<concpt::propeller> {
    public:
        propeller_force (const concpt::propeller& IN_PROPELLER) : concpt::forces<concpt::propeller>(IN_PROPELLER) {}
        std::array<float, 3> get_force_vector() const override;
        std::array<float, 3> get_moment_vector() const override;
        std::array<float, 3> get_position() const override;
        std::array<float, 3> get_euler_angles() const override;
    };
}

#endif //CONCEPTUAL_PROPELLER_FORCES_H
