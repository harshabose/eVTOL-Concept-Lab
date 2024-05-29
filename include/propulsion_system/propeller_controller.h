//
// Created by Harshavardhan Karnati on 27/05/2024.
//

#ifndef CONCEPTUAL_PROPELLER_CONTROLLER_H
#define CONCEPTUAL_PROPELLER_CONTROLLER_H

#include <array>

namespace concpt::declare {
    struct set_pitch_bounds;
    struct set_rpm_bounds;
    struct set_pitch_and_rpm_bounds_pitch;
    struct set_pitch_and_rpm_bounds_rpm;
    class use_pitch;
    class use_rpm;
    class use_pitch_and_rpm;
}   //forward declarations

namespace concpt::detail {
    enum propeller_control_var {PITCH, RPM, BOTH, NONE};


    class propeller_controller_settings {
    public:
        friend struct concpt::declare::set_pitch_bounds;
        friend struct concpt::declare::set_rpm_bounds;
        friend struct concpt::declare::set_pitch_and_rpm_bounds_pitch;
        friend struct concpt::declare::set_pitch_and_rpm_bounds_rpm;
        friend class concpt::declare::use_pitch;
        friend class concpt::declare::use_rpm;
        friend class concpt::declare::use_pitch_and_rpm;

        explicit propeller_controller_settings (concpt::detail::propeller_control_var IN_CONTROLLER);
    private:
        concpt::detail::propeller_control_var control_var = PITCH;
        std::size_t number_of_control_vars = 1;
        static double nlopt_arr[2];
        double *current_pitch = nullptr;
        double *current_rpm = nullptr;
        std::array<double, 2> lower_bounds = {0.0, 0.0};    //first = pitch
        std::array<double, 2> upper_bounds = {0.0, 0.0};    //first = pitch
    };
}

namespace concpt::declare {

    template<class builderType>
    struct set_bounds_interface {
        explicit set_bounds_interface (builderType *IN_BUILDER) : builder(IN_BUILDER) {}
        virtual builderType& set_lower_bound (const double &IN_VALUE) = 0;
        virtual builderType& set_upper_bound (const double &IN_VALUE) = 0;
    protected:
        builderType *builder;
    };

    struct set_pitch_bounds : public set_bounds_interface<concpt::declare::use_pitch> {
        explicit set_pitch_bounds (concpt::declare::use_pitch *IN_BUILDER);
        concpt::declare::use_pitch& set_lower_bound(const double &IN_VALUE) override;
        concpt::declare::use_pitch& set_upper_bound(const double &IN_VALUE) override;
    };

    struct set_rpm_bounds : public set_bounds_interface<concpt::declare::use_rpm> {
        explicit set_rpm_bounds (concpt::declare::use_rpm *IN_BUILDER);
        concpt::declare::use_rpm& set_lower_bound(const double &IN_VALUE) override;
        concpt::declare::use_rpm& set_upper_bound(const double &IN_VALUE) override;
    };

    struct set_pitch_and_rpm_bounds_pitch : public set_bounds_interface<concpt::declare::use_pitch_and_rpm> {
        explicit set_pitch_and_rpm_bounds_pitch (concpt::declare::use_pitch_and_rpm *IN_BUILDER);
        concpt::declare::use_pitch_and_rpm& set_lower_bound(const double &IN_VALUE) override;
        concpt::declare::use_pitch_and_rpm& set_upper_bound(const double &IN_VALUE) override;
    };

    struct set_pitch_and_rpm_bounds_rpm : public set_bounds_interface<concpt::declare::use_pitch_and_rpm> {
        explicit set_pitch_and_rpm_bounds_rpm (concpt::declare::use_pitch_and_rpm *IN_BUILDER);
        concpt::declare::use_pitch_and_rpm& set_lower_bound(const double &IN_VALUE) override;
        concpt::declare::use_pitch_and_rpm& set_upper_bound(const double &IN_VALUE) override;
    };

    class use_pitch {
    public:
        friend struct concpt::declare::set_pitch_bounds;
        explicit use_pitch ();
        concpt::declare::set_pitch_bounds set_bounds = concpt::declare::set_pitch_bounds(this);
        concpt::declare::use_pitch& set_pitch(const float &IN_VALUE);
        concpt::declare::use_pitch& set_rpm(const float &IN_VALUE);
        [[nodiscard]] concpt::detail::propeller_controller_settings build () const;
    private:
        concpt::detail::propeller_controller_settings settings;
        bool pitch_set = false;
        bool rpm_set = false;
        bool lower_bound_set = false;
        bool upper_bound_set = false;
        [[nodiscard]] bool perform_checks () const;
    };

    class use_rpm {
    public:
        friend struct concpt::declare::set_rpm_bounds;
        explicit use_rpm ();
        concpt::declare::set_rpm_bounds set_bounds = concpt::declare::set_rpm_bounds(this);
        concpt::declare::use_rpm& set_pitch(const float &IN_VALUE);
        concpt::declare::use_rpm& set_rpm(const float &IN_VALUE);
        [[nodiscard]] concpt::detail::propeller_controller_settings build () const;
    private:
        concpt::detail::propeller_controller_settings settings;
        bool pitch_set = false;
        bool rpm_set = false;
        bool lower_bound_set = false;
        bool upper_bound_set = false;
        [[nodiscard]] bool perform_checks () const;
    };

    class use_pitch_and_rpm {
    public:
        friend struct concpt::declare::set_pitch_and_rpm_bounds_pitch;
        friend struct concpt::declare::set_pitch_and_rpm_bounds_rpm;

        explicit use_pitch_and_rpm ();
        concpt::declare::set_pitch_and_rpm_bounds_pitch set_pitch_bounds = concpt::declare::set_pitch_and_rpm_bounds_pitch(this);
        concpt::declare::set_pitch_and_rpm_bounds_rpm set_rpm_bounds = concpt::declare::set_pitch_and_rpm_bounds_rpm(this);
        concpt::declare::use_pitch_and_rpm& set_pitch(const float &IN_VALUE);
        concpt::declare::use_pitch_and_rpm& set_rpm(const float &IN_VALUE);
        [[nodiscard]] concpt::detail::propeller_controller_settings build () const;
    private:
        concpt::detail::propeller_controller_settings settings;
        bool pitch_set = false;
        bool rpm_set = false;
        bool pitch_lower_bounds_set = false;
        bool pitch_upper_bounds_set = false;
        bool rpm_lower_bounds_set = false;
        bool rpm_upper_bounds_set = false;
        [[nodiscard]] bool perform_checks () const;
    };
}


#endif //CONCEPTUAL_PROPELLER_CONTROLLER_H
