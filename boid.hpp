#ifndef BOID_HPP
#define BOID_HPP


#include "vector2.hpp"

namespace boids {

class Boid {
 private:
    Vector2 position_;
    Vector2 velocity_;

 public:
    Boid(Vector2 position, Vector2 velocity);

    Vector2 const& position() const;
    Vector2 const& velocity() const;

    void set_position(Vector2 const& position);
    void set_velocity(Vector2 const& velocity);
};
}
#endif