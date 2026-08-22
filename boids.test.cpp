#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boid.hpp"
#include "vector3.hpp"
#include "flock.hpp"
#include <cmath>
#include <vector>

TEST_CASE ("Vector3 addition, norm and distance")
{
   boids::Vector3 const a{1., 2., 0.};
   boids::Vector3 const b{1., 1., 0.};

    auto const c = a + b;

    CHECK(c.x == doctest::Approx(2.));
    CHECK(c.y == doctest::Approx(3.));
    CHECK(c.z == doctest::Approx(0.));
    CHECK(boids::distance (a, b) == doctest::Approx(1.));
    CHECK(boids::norm (c) == doctest::Approx(3.60555));
}

TEST_CASE ("boids setting and refresh")
{
   boids::Vector3 const a{1., 2., 0.};
   boids::Vector3 const b{1., 1., 0.};
   boids::Boid c{a, b, 1.}; //da errore perche in boid.cpp ancora non è stato messo il nuovo parametro view_angle

    CHECK(c.position() == a); //è pericoloso usare == se si fa un calcolo in quanto si fanno dei calcoli su dei double, nel caso meglio usare l'approx
    CHECK(c.velocity() == b);
   
   c.set_position (b);
   c.set_velocity (a);

    CHECK (c.position() == b);
    CHECK (c.velocity() == a);

}

TEST_CASE("flock update") {
    double const Lx = 10.;
    double const Ly = 10.;
    double const Lz = 10.;
    double const max_speed = 10.;
    
    SUBCASE("update without neighbours") { //funzionamento base
        boids::Boid const bird{boids::Vector3{1., 1., 0.}, boids::Vector3{1., 0., 0.}, 1.}; //creo boid con posizione e velocità
        std::vector<boids::Boid> birds{bird}; //lo metto in un vector perchè flock vuole un vectro come primo parametro
        boids::Flock flock{ birds, 2., 1., 1., 1., 1., max_speed, Lx, Ly, Lz};
        flock.update(1.); //l'argomento di update è il dt
        auto const& updated_bird = flock.boids()[0]; //recupera il boid dopo update: mi permette di vedere i boid nel flock creato
        CHECK(updated_bird.position().x == doctest::Approx(2.));
        CHECK(updated_bird.position().y == doctest::Approx(1.));
        CHECK(updated_bird.position().z == doctest::Approx(0.));
        CHECK(updated_bird.velocity().x == doctest::Approx(1.));
        CHECK(updated_bird.velocity().y == doctest::Approx(0.));
        CHECK(updated_bird.velocity().z == doctest::Approx(0.));

    }
    SUBCASE("update with neighbours") { //controlla indirettamente separation, alignment e cohesion, e quindi anche toroidal_displacement
        boids::Boid const bird1{boids::Vector3{0., 0., 0.}, boids::Vector3{1., 0., 0.}, 1.};
        boids::Boid const bird2{boids::Vector3{1., 0., 0.}, boids::Vector3{0., 0., 0.}, 1.};
        std::vector<boids::Boid> birds{bird1, bird2};
        boids::Flock flock{ birds, 5., 2., 1., 1., 1., max_speed, Lx, Ly, Lz}; //d=5 in modo tale che siano considerati vicini, ds=2 per far entrare in gioco separation
        flock.update(1.);
        CHECK(flock.boids()[0].velocity().x == doctest::Approx(0.)); //con vel iniziale=1, separation=-1, alignment=-1, cohesion=1
    }
    SUBCASE("update across boundary") {//controlla indirettamente wrap_position e quindi anche toroidal_shortcut
       boids::Boid const bird{boids::Vector3{4., 0., 0.}, boids::Vector3{2., 0., 0.}, 1.}; 
       std::vector<boids::Boid> birds{bird};
       boids::Flock flock{ birds, 1., 1., 1., 1., 1., max_speed, Lx, Ly, Lz};
       flock.update(1.);
       CHECK(flock.boids()[0].position().x == doctest::Approx(-4.));
    }
}

TEST_CASE("flock statistics") {
    boids::Boid const bird1{boids::Vector3{0., 0., 0.}, boids::Vector3{3., 0., 0.}, 1.};
    boids::Boid const bird2{boids::Vector3{2., 0., 0.}, boids::Vector3{1., 0., 0.}, 1.};
    std::vector<boids::Boid> birds{bird1, bird2};
    boids::Flock flock{ birds, 1., 1., 1., 1., 1., 10., 10., 10., 10.}; 
    CHECK(flock.mean_speed() == doctest::Approx(2.));
    CHECK(flock.speed_stddev() == doctest::Approx(1.));
    CHECK(flock.mean_distance() == doctest::Approx(2.));
    CHECK(flock.distance_stddev() == doctest::Approx(0.));

}
