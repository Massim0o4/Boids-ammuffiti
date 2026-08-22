#ifndef EPSTEIN_HPP
#define EPSTEIN_HPP

#include "boid.hpp"

namespace boids {

class Pred : public Boid
{
    private:
    int cooldown_timer_;
    bool hungry_;
 public:
 Pred(int cooldown_timer, bool hungry);
 int cooldown_timer();
 bool hungry();
  void hunt(Boid prey);
  void cooldown();
  void find_prey();
};
} // namespace boids

#endif