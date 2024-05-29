//
// Created by Harshavardhan Karnati on 24/05/2024.
//

#ifndef CONCEPTUAL_AEROSANDBOX_RESULT_TYPES_H
#define CONCEPTUAL_AEROSANDBOX_RESULT_TYPES_H

#endif //CONCEPTUAL_AEROSANDBOX_RESULT_TYPES_H

#include <iostream>
#include <array>

namespace concpt {
    enum result_name {FORCE_EARTH, FORCE_BODY, FORCE_WIND, MOMENT_EARTH, MOMENT_BODY, MOMENT_WIND,
        LIFT_FORCE, SIDE_FORCE, DRAG_FORCE, ROLL_MOMENT, PITCH_MOMENT, YAW_MOMENT,
        LIFT_FORCE_COEFF, SIDE_FORCE_COEFF, DRAG_FORCE_COEFF, ROLL_MOMENT_COEFF, PITCH_MOMENT_COEFF, YAW_MOMENT_COEFF,
        LIFT_FORCE_COEFF_ALPHA, SIDE_FORCE_COEFF_ALPHA, DRAG_FORCE_COEFF_ALPHA, ROLL_MOMENT_COEFF_ALPHA, PITCH_MOMENT_COEFF_ALPHA, YAW_MOMENT_COEFF_ALPHA,
        LIFT_FORCE_COEFF_BETA, SIDE_FORCE_COEFF_BETA, DRAG_FORCE_COEFF_BETA, ROLL_MOMENT_COEFF_BETA, PITCH_MOMENT_COEFF_BETA, YAW_MOMENT_COEFF_BETA,
        LIFT_FORCE_COEFF_ROLL, SIDE_FORCE_COEFF_ROLL, DRAG_FORCE_COEFF_ROLL, ROLL_MOMENT_COEFF_ROLL, PITCH_MOMENT_COEFF_ROLL, YAW_MOMENT_COEFF_ROLL,
        LIFT_FORCE_COEFF_PITCH, SIDE_FORCE_COEFF_PITCH, DRAG_FORCE_COEFF_PITCH, ROLL_MOMENT_COEFF_PITCH, PITCH_MOMENT_COEFF_PITCH, YAW_MOMENT_COEFF_PITCH,
        LIFT_FORCE_COEFF_YAW, SIDE_FORCE_COEFF_YAW, DRAG_FORCE_COEFF_YAW, ROLL_MOMENT_COEFF_YAW, PITCH_MOMENT_COEFF_YAW, YAW_MOMENT_COEFF_YAW,
        NEUTRAL_POINT, PROFILE_DRAG_FORCE, INDUCED_DRAG_FORCE, PROFILE_DRAG_FORCE_COEFF, INDUCED_DRAG_FORCE_COEFF, OSWALS_EFFECIENCY_FACTOR, INDUCED_FACTOR,
        WING_AERO_LAST};

    namespace detail {
        using aerosandbox_internal_keys_t = std::array<std::string, concpt::result_name::WING_AERO_LAST>;
        const aerosandbox_internal_keys_t aerosandbox_internal_keys = {"F_g", "F_b", "F_w",
                                                                                    "M_g", "M_b", "M_w",
                                                                                    "L", "Y", "D",
                                                                                    "l_b", "m_b", "n_b",
                                                                                    "CL", "CY", "CD",
                                                                                    "Cl", "Cm", "Cn",
                                                                                    "CLa", "CDa", "CYa",
                                                                                    "Cla", "Cma", "Cna",
                                                                                    "CLb", "CDb", "CYb",
                                                                                    "Clb", "Cmb", "Cnb",
                                                                                    "CLp", "CDp", "CYp",
                                                                                    "Clp", "Cmp", "Cnp",
                                                                                    "CLq", "CDq", "CYq",
                                                                                    "Clq", "Cmq", "Cnq",
                                                                                    "CLr", "CDr", "CYr",
                                                                                    "Clr", "Cmr", "Cnr",
                                                                                    "x_np", "D_profile",
                                                                                    "D_induced",
                                                                                    "D_profile",
                                                                                    "D_induced",
                                                                                    "oswalds_efficiency",
                                                                                    "INDUCED_COEFF"
        };
    }
}