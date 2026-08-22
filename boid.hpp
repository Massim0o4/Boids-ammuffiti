#ifndef BOID_HPP
#define BOID_HPP


#include "vector3.hpp"

namespace boids {

class Boid {
 private:
    Vector3 position_;
    Vector3 velocity_;
    double view_angle_;

 public:
    Boid(Vector3 position, Vector3 velocity, double view_angle);

    Vector3 const& position() const;
    Vector3 const& velocity() const;
    double const & view_angle() const;

    void set_position(Vector3 const& position);
    void set_velocity(Vector3 const& velocity);
};
}
#endif