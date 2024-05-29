//
// Created by Harshavardhan Karnati on 20/05/2024.
//

#ifndef CONCEPTUAL_WEIGHTS_H
#define CONCEPTUAL_WEIGHTS_H

#include "../includes.h"
#include "forces.h"

namespace concpt {

    class plane_weight;
    class wing_weight;
    class propulsion_system_weight;
    class propeller_weight;

    template<class component_type>
    class weights : public concpt::forces<component_type> {
    public:
        explicit weights (const component_type& IN_COMPONENT) : concpt::forces<component_type>(IN_COMPONENT) {}
        [[nodiscard]] virtual std::array<float, 3> get_rotaional_inertial () const = 0;
    protected:
    };

    template<class component_type>
    class weights_visitor {
    public:
        virtual void resolve_weight (const concpt::plane_weight &IN_PLANE) = 0;
        virtual void resolve_weight (const concpt::wing_weight &IN_PROP_SYSTEM) = 0;
        virtual void resolve_weight (const concpt::propulsion_system_weight &IN_PROP_SYSTEM) = 0;
        virtual void resolve_weight (const concpt::propeller_weight &IN_PROP) = 0;
    };

    class plane_weight_visitor;
    class wing_weight_visitor;
    class propulsion_system_weight_visitor;
    class propeller_weight_visitor;

    class plane_weight : public concpt::weights<concpt::plane> {
    public:
        explicit plane_weight (const concpt::plane& IN_COMPONENT) : concpt::weights<concpt::plane>(IN_COMPONENT) {}
        [[nodiscard]] std::array<float, 3> get_force_vector() const override;
        [[nodiscard]] std::array<float, 3> get_moment_vector() const override;
        [[nodiscard]] std::array<float, 3> get_position () const override;
        [[nodiscard]] std::array<float, 3> get_euler_angles() const override;
        [[nodiscard]] std::array<float, 3> get_rotaional_inertial() const override;
    };

    class wing_weight : public concpt::weights<concpt::wing> {
    public:
        explicit wing_weight (const concpt::wing& IN_COMPONENT) : concpt::weights<concpt::wing>(IN_COMPONENT) {}
        [[nodiscard]] std::array<float, 3> get_force_vector() const override;
        [[nodiscard]] std::array<float, 3> get_moment_vector() const override;
        [[nodiscard]] std::array<float, 3> get_position () const override;
        [[nodiscard]] std::array<float, 3> get_euler_angles() const override;
        [[nodiscard]] std::array<float, 3> get_rotaional_inertial() const override;
    };

    class propulsion_system_weight : public concpt::weights<concpt::propulsion_system> {
    public:
        explicit propulsion_system_weight (const concpt::propulsion_system& IN_COMPONENT) : concpt::weights<concpt::propulsion_system>(IN_COMPONENT) {}
        [[nodiscard]] std::array<float, 3> get_force_vector() const override;
        [[nodiscard]] std::array<float, 3> get_moment_vector() const override;
        [[nodiscard]] std::array<float, 3> get_position () const override;
        [[nodiscard]] std::array<float, 3> get_euler_angles() const override;
        [[nodiscard]] std::array<float, 3> get_rotaional_inertial() const override;
    };

    class propeller_weight : public concpt::weights<concpt::propeller> {
    public:
        friend class concpt::propeller;
        explicit propeller_weight (const concpt::propeller& IN_COMPONENT) : concpt::weights<concpt::propeller>(IN_COMPONENT) {}
        [[nodiscard]] std::array<float, 3> get_force_vector() const override;
        [[nodiscard]] std::array<float, 3> get_moment_vector() const override;
        [[nodiscard]] std::array<float, 3> get_position () const override;
        [[nodiscard]] std::array<float, 3> get_euler_angles() const override;
        [[nodiscard]] std::array<float, 3> get_rotaional_inertial() const override;
    };

    class plane_weight_visitor : public concpt::weights_visitor<concpt::plane> {
    public:
        explicit plane_weight_visitor (const plane_weight& IN_PLANE_WEIGHT) : plane_weight(IN_PLANE_WEIGHT) {}
        void resolve_weight(const concpt::plane_weight &IN_PLANE) override;
        void resolve_weight(const concpt::wing_weight &IN_PROP_SYSTEM) override;
        void resolve_weight(const concpt::propulsion_system_weight &IN_PROP_SYSTEM) override;
        void resolve_weight(const concpt::propeller_weight &IN_PROP) override;
    private:
        const plane_weight& plane_weight;
    };
/*
    class wing_weight_visitor : public concpt::weights_visitor<concpt::wing> {
    public:
        explicit wing_weight_visitor (const wing_weight& IN_WING_WEIGHT) : wing_weight(IN_WING_WEIGHT){}
        void transform(const concpt::plane_weight &IN_PLANE) override;
        void transform(const concpt::wing_weight &IN_PROP_SYSTEM) override;
        void transform(const concpt::propulsion_system_weight &IN_PROP_SYSTEM) override;
        void transform(const concpt::propeller_weight &IN_PROP) override;
    private:
        const wing_weight& wing_weight;
    };

    class propulsion_system_weight_visitor : public concpt::weights_visitor<concpt::propulsion_system> {
    public:
        explicit propulsion_system_weight_visitor (const propulsion_system_weight& IN_PROPULSION_SYSTEM_WEIGHT) :
            propulsion_system_weight(IN_PROPULSION_SYSTEM_WEIGHT) {}
        void transform(const concpt::plane_weight &IN_PLANE) override;
        void transform(const concpt::wing_weight &IN_PROP_SYSTEM) override;
        void transform(const concpt::propulsion_system_weight &IN_PROP_SYSTEM) override;
        void transform(const concpt::propeller_weight &IN_PROP) override;
    private:
        const propulsion_system_weight& propulsion_system_weight;
    };

    class propeller_weight_visitor : public concpt::weights_visitor<concpt::propeller> {
    public:
        explicit propeller_weight_visitor (const propeller_weight& IN_PROPELLER_WEIGHT) :
            propeller_weight(IN_PROPELLER_WEIGHT) {}
        void transform(const concpt::plane_weight &IN_PLANE) override;
        void transform(const concpt::wing_weight &IN_PROP_SYSTEM) override;
        void transform(const concpt::propulsion_system_weight &IN_PROP_SYSTEM) override;
        void transform(const concpt::propeller_weight &IN_PROP) override;
    private:
        const propeller_weight& propeller_weight;
    };
*/
}


#endif //CONCEPTUAL_WEIGHTS_H
