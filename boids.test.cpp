#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boid.hpp"
#include "vector3.hpp"
#include <cmath>

TEST_CASE ("Vector3 addition, norm and distance")
{
   boids::Vector3 const a{1., 2.};
   boids::Vector3 const b{1., 1.};

    auto const c = a + b;

    CHECK(c.x == doctest::Approx(2.));
    CHECK(c.y == doctest::Approx(3.));
    CHECK(boids::distance (a, b) == doctest::Approx(1.));
    CHECK(boids::norm (c) == doctest::Approx(3.60555));
}

TEST_CASE ("boids setting and refresh")
{
   boids::Vector3 const a{1., 2.};
   boids::Vector3 const b{1., 1.};
   boids::Boid c {a,b}; 

    CHECK(c.position() == a); //è pericoloso usare == se si fa un calcolo in quanto si fanno dei calcoli su dei double, nel caso meglio usare l'approx
    CHECK(c.velocity() == b);
   
   c.set_position (b);
   c.set_velocity (a);

    CHECK (c.position() == b);
    CHECK (c.velocity() == a);

}