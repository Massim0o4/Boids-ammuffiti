#ifndef BOID_HPP
#define BOID_HPP


#include "vector3.hpp"

namespace boids {

class Boid {
 private:
    Vector3 position_;
    Vector3 velocity_;

 public:
    Boid(Vector3 position, Vector3 velocity);

    Vector3 const& position() const;
    Vector3 const& velocity() const;

    void set_position(Vector3 const& position);
    void set_velocity(Vector3 const& velocity);
};
}
#endif