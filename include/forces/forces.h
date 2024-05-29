//
// Created by Harshavardhan Karnati on 21/05/2024.
//

#ifndef CONCEPTUAL_FORCES_H
#define CONCEPTUAL_FORCES_H

#include "../includes.h"
#include <array>
#include <memory>

namespace concpt {

    class plane_force;
    class wing_force;
    class propulsion_system_force;
    class propeller_force;

    template <class component_type>
    class forces {
    public:
        explicit forces (const component_type& IN_COMPONENT) : component(IN_COMPONENT) {}
        virtual ~forces () = default;

        [[nodiscard]] virtual std::array<float, 3> get_force_vector () const = 0;
        [[nodiscard]] virtual std::array<float, 3> get_moment_vector () const = 0;
        [[nodiscard]] virtual std::array<float, 3> get_position () const = 0;
        [[nodiscard]] virtual std::array<float, 3> get_euler_angles () const = 0;
    protected:
        const component_type& component;
    };

    template<class component_type>
    class forces_visitor {
    public:
        virtual void resolve_force (const concpt::plane_force& IN_PLANE) = 0;
        virtual void resolve_force (const concpt::wing_force& IN_PLANE) = 0;
        virtual void resolve_force (const concpt::propulsion_system_force& IN_PLANE) = 0;
        virtual void resolve_force (const concpt::propeller_force& IN_PLANE) = 0;
    };

    //specialised abstract class to attack to specific components.
    class plane_force : public concpt::forces<concpt::plane> {};
    class wing_force : public concpt::forces<concpt::wing> {};
    class propulsion_system_force : public concpt::forces<propulsion_system> {};


    class plane_forces_visitor : public concpt::forces_visitor<concpt::plane> {
    public:
        explicit plane_forces_visitor (const concpt::plane_force& IN_PLANE_FORCE) : plane_force(IN_PLANE_FORCE) {}
        void resolve_force(const concpt::plane_force &IN_PLANE) override;
        void resolve_force(const concpt::wing_force &IN_WING) override;
        void resolve_force(const concpt::propulsion_system_force &IN_POPULSION_SYSTEM) override;
        void resolve_force(const concpt::propeller_force &IN_PROPELLER) override;
    private:
        const plane_force& plane_force;
    };

    class wing_forces_visitor : public concpt::forces_visitor<concpt::wing> {
    public:
        explicit wing_forces_visitor (const wing_force& IN_WING) : wing_force(IN_WING) {}
        void resolve_force(const concpt::plane_force &IN_PLANE) override;
        void resolve_force(const concpt::wing_force &IN_WING) override;
        void resolve_force(const concpt::propulsion_system_force &IN_POPULSION_SYSTEM) override;
        void resolve_force(const concpt::propeller_force &IN_PROPELLER) override;
    private:
        const wing_force& wing_force;
    };

    class propulsion_system_forces_visitor : public concpt::forces_visitor<concpt::wing> {
    public:
        explicit propulsion_system_forces_visitor (const propulsion_system_force& IN_PROPULSION_SYSTEM) :
            propulsion_system_force(IN_PROPULSION_SYSTEM) {}
        void resolve_force(const concpt::plane_force &IN_PLANE) override;
        void resolve_force(const concpt::wing_force &IN_WING) override;
        void resolve_force(const concpt::propulsion_system_force &IN_POPULSION_SYSTEM) override;
        void resolve_force(const concpt::propeller_force &IN_PROPELLER) override;
    private:
        const propulsion_system_force& propulsion_system_force;
    };

    class propeller_forces_visitor : public concpt::forces_visitor<concpt::wing> {
    public:
        explicit propeller_forces_visitor (const propeller_force& IN_PROPELLER) : propeller_force(IN_PROPELLER) {}
        void resolve_force(const concpt::plane_force &IN_PLANE) override;
        void resolve_force(const concpt::wing_force &IN_WING) override;
        void resolve_force(const concpt::propulsion_system_force &IN_POPULSION_SYSTEM) override;
        void resolve_force(const concpt::propeller_force &IN_PROPELLER) override;
    private:
        const propeller_force& propeller_force;
    };
}

#endif //CONCEPTUAL_FORCES_H
