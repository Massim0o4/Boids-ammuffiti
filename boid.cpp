#include "boid.hpp"

namespace boids {

Boid::Boid(Vector2 position, Vector2 velocity) : position_{position}, velocity_{velocity} {}

Vector2 const& Boid::position() const {
    return position_;}

Vector2 const& Boid::velocity() const {
    return velocity_;}

void Boid::set_position(Vector2 const& position) {
    position_ = position;}

void Boid::set_velocity(Vector2 const& velocity){
    velocity_ = velocity;}

}
