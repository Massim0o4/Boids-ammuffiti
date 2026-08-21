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
  double max_speed_;

  double Lx_;
  double Ly_;
  double Lz_;

/*  double toroidal_distance(Vector3 const& a, Vector3 const& b) const; */

  Vector3 toroidal_displacement(Vector3 const& a, Vector3 const& b) const; //tiene conto dello spazio toroidale nel calcolare la distanza tra due boids
  Vector3 wrap_position(Vector3 const& position) const; //fa rispuntare il boid dall'altro lato

  Vector3 separation(std::size_t i) const;
  Vector3 alignment(std::size_t i) const; // non modificano i double sopra o il vettore,
  Vector3 cohesion(std::size_t i) const;  // calcolano i dati nuovi v1, v2 e v3

 public:
  Flock(std::vector<Boid> boids, double d, double ds, double s, double a, double c, double max_speed, double Lx_, double Ly_, double Lz_);

  void update(double dt);

  double mean_speed() const;
  double speed_stddev() const;

  double mean_distance() const;
  double distance_stddev() const;

  std::vector<Boid> const& boids() const;
};

} // namespace boids

#endif