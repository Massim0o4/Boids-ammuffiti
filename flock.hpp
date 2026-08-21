#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <cstddef>
#include <vector>

#include "boid.hpp"

namespace boids {

class Flock {
 private:
    std::vector<Boid> boids_;

    double d_;
    double ds_;
    double s_;
    double a_;
    double c_;

    Vector2 separation(std::size_t i) const;
    Vector2 alignment(std::size_t i) const; //non modificano i double sopra o il vettore, calcolano i dati nuovi v1, v2 e v3
    Vector2 cohesion(std::size_t i) const;

 public:
    Flock(std::vector<Boid> boids,
          double d,
          double ds,
          double s,
          double a,
          double c);

    void update(double dt);

    double mean_speed() const;
    double speed_stddev() const;

    double mean_distance() const;
    double distance_stddev() const;

    std::vector<Boid> const& boids() const;
};

}  // namespace boids

#endif