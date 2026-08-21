#include "flock.hpp"

#include <cmath>

namespace boids {

Flock::Flock(std::vector<Boid> boids,
             double d,
             double ds,
             double s,
             double a,
             double c)
    : boids_{boids},
      d_{d},
      ds_{ds},
      s_{s},
      a_{a},
      c_{c}
{
}