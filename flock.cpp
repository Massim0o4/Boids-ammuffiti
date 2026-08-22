#include "flock.hpp"
#include <stdexcept>
#include <cmath>

namespace boids {

Flock::Flock(std::vector<Boid> boids, double d, double ds, double s, double a, double c, double max_speed, double Lx, double Ly, double Lz)
    : boids_{boids}, d_{d}, ds_{ds}, s_{s}, a_{a}, c_{c}, max_speed_{max_speed}, Lx_ {Lx}, Ly_ {Ly},  Lz_ {Lz} {} 

double Flock::toroidal_shortcut(double d, double L) const { //deve essere const perché usato in altre funzioni const
        while (d > L / 2.0) {
            d -= L;} 
        while (d < -L / 2.0) {
            d += L;}
        return d;
        };    

 Vector3 Flock::wrap_position(Vector3 position) const {
    position.x = toroidal_shortcut(position.x, Lx_); //implica che è un cubo centrato in 0, quindi inizia a -L/2!
    position.y = toroidal_shortcut(position.y, Ly_);
    position.z = toroidal_shortcut(position.z, Lz_);
    return position;
} 

Vector3 Flock::toroidal_displacement(Vector3 const& a, Vector3 const& b) const{
        Vector3 delta = b - a;

        delta.x = toroidal_shortcut(delta.x, Lx_);
        delta.y = toroidal_shortcut(delta.y, Ly_);
        delta.z = toroidal_shortcut(delta.z, Lz_);
        return delta;
    }

bool Flock::visible (std::size_t i, Vector3 const& delta) const {
    Vector3 const velocity = boids_[i].velocity();
    if ( norm(velocity ) == 0 || norm(delta) == 0) {
        return true;
    }
    double cos_teta = scalar_product(velocity, delta)/ (norm (velocity) * norm (delta)); 
    
    return std::cos(boids_[i].view_angle() / 2.) <= cos_teta ;
}

Vector3 Flock::separation(std::size_t i) const {
    Vector3 result{0., 0., 0.};

    for (std::size_t j = 0; j < boids_.size(); ++j) {
        if (j == i) {
            continue;
        }
        Vector3 delta = toroidal_displacement (boids_[i].position(), boids_[j].position());

        if (norm(delta) < ds_) {

            result = result - delta * s_;
        }
    }
    return result;
} 

Vector3 Flock::alignment(std::size_t i) const
{
    Vector3 velocity_diff_sum{0., 0., 0.};
    std::size_t neighbours{0};

    for (std::size_t j = 0; j < boids_.size(); ++j) {
        if (j == i) {
            continue;
        }
        Vector3 delta = toroidal_displacement (boids_[i].position(), boids_[j].position());

        if (norm(delta) < d_ && visible (i, delta )) {

            velocity_diff_sum = velocity_diff_sum + (boids_[j].velocity() - boids_[i].velocity());

            ++neighbours;
        }
    }

    if (neighbours == 0) {
        return Vector3{0., 0., 0.};
    }

    double const n = static_cast<double>(neighbours);

    return velocity_diff_sum * a_ / n; //è n-1 perché non conta se stesso
}

Vector3 Flock::cohesion(std::size_t i) const {
    Vector3 position_sum{0., 0., 0.};
    std::size_t neighbours{0};

    for (std::size_t j = 0; j < boids_.size(); ++j) {
        if (j == i) {
            continue;
        }
        Vector3 delta = toroidal_displacement (boids_[i].position(), boids_[j].position());

        if (norm(delta) < d_ && visible (i, delta )) {

            position_sum = position_sum + delta;
            ++neighbours;
        }
    }

    if (neighbours == 0) {
        return Vector3{0., 0.,0.};
    }

    double const n = static_cast<double>(neighbours);
    Vector3 const centre = position_sum / n; 

    return centre  * c_;
}

void Flock::update(double dt)
{
    std::vector<Boid> new_boids = boids_;

    for (std::size_t i = 0; i < boids_.size(); ++i) {

        Vector3 new_velocity = boids_[i].velocity() + separation(i) + alignment(i) + cohesion(i);

        double const speed = norm(new_velocity);
        if (speed > max_speed_) { new_velocity = new_velocity * (max_speed_ / speed);
}

        Vector3 new_position = boids_[i].position() + new_velocity * dt;
        new_position = wrap_position (new_position);

        new_boids[i].set_velocity(new_velocity);
        new_boids[i].set_position(new_position);
    }

    boids_ = new_boids; 
}

double Flock::mean_speed() const
{
    if (boids_.empty()) {
        return 0.;
    }

    double sum{0.};

    for (std::size_t i = 0; i < boids_.size(); ++i) {
        sum += norm(boids_[i].velocity());
    }

    /* for (Boid const& boid : boids_) {
        sum += norm(boid.velocity());
    } */
     
    double const n = static_cast<double>(boids_.size());

    return sum / n;
}

double Flock::speed_stddev() const
{
    if (boids_.empty()) {
        return 0.;
    }

    double const mean = mean_speed();

    double sum{0.};

    for (std::size_t i = 0; i < boids_.size(); ++i) {

        double const difference = norm(boids_[i].velocity()) - mean;

        sum += difference * difference;
    }
    
    double const n = static_cast<double>(boids_.size());

    return std::sqrt(sum / n);
}

double Flock::mean_distance() const
{
    if (boids_.size() < 2) {
        return 0.;
    }

    double sum{0.};
    std::size_t pairs{0};

    for (std::size_t i = 0; i < boids_.size(); ++i) {

        for (std::size_t j = i + 1; j < boids_.size(); ++j) {
            Vector3 delta = toroidal_displacement (boids_[i].position(), boids_[j].position());

            sum += norm(delta);
            ++pairs;
        }
    }

    return sum / static_cast<double>(pairs);
}

double Flock::distance_stddev() const
{
    if (boids_.size() < 2) {
        return 0.;
    }

    double const mean = mean_distance();

    double sum{0.};
    std::size_t pairs{0};

    for (std::size_t i = 0; i < boids_.size(); ++i) {

        for (std::size_t j = i + 1; j < boids_.size(); ++j) {
            Vector3 delta = toroidal_displacement (boids_[i].position(), boids_[j].position());

            double const difference = norm(delta) - mean;

            sum += difference * difference;

            ++pairs;
        }
    }

    return std::sqrt(
        sum / static_cast<double>(pairs));
}

std::vector<Boid> const& Flock::boids() const
{
    return boids_;
}

}