#include "boid.hpp"

namespace boids {

Boid::Boid(Vector3 position, Vector3 velocity) : position_{position}, velocity_{velocity} {}

Vector3 const& Boid::position() const {
    return position_;}

Vector3 const& Boid::velocity() const {
    return velocity_;}

void Boid::set_position(Vector3 const& position) {
    position_ = position;}

void Boid::set_velocity(Vector3 const& velocity){
    velocity_ = velocity;}

}
