#include "epstein.hpp"

namespace boids {
    Pred::(int cooldown_timer, bool hungry) : cooldown_timer_{cooldown_timer}, hungry_{hungry} {}
void Pred::hunt (Boid prey) {
set_velocity (prey.velocity()-velocity());
}
void Pred::cooldown() {
    Vector3 origin{0.0, 0.0, 0.0};
    if (position() == origin) {} 
    else {
        set_velocity( position() * -1);
    }
}
}