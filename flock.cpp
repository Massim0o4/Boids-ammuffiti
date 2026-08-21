#include "flock.hpp"
#include <stdexcept>
#include <cmath>

namespace boids {

Flock::Flock(std::vector<Boid> boids, double d, double ds, double s, double a, double c, double max_speed, double Lx, double Ly, double Lz,)
    : boids_{boids}, d_{d}, ds_{ds}, s_{s}, a_{a}, c_{c}, max_speed_{max_speed}, Lx_ {Lx}, Ly_ {Ly},  Lz_ {Lz}, {} 

Vector3 Flock::wrap_position(Vector3 const& position) const {
    Vector3 result = position;

    if (result.x < 0.) {
        result.x += Lx_;
    } else if (result.x >= Lx_) {
        result.x -= Lx_;
    } 
    if (result.y < 0.) {
        result.x += Ly_;
    } else if (result.x >= Ly_) {
        result.y -= Ly_;
    } 
    if (result.z < 0.) {
        result.z += Lz_;
    } else if (result.z >= Lz_) {
        result.z -= Lz_;
    } 
    return result;
}

Vector3 Flock::separation(std::size_t i) const {
    Vector3 result{0., 0., 0.};

    for (std::size_t j = 0; j < boids_.size(); ++j) {
        if (j == i) {
            continue;
        }

        if (distance(boids_[i].position(),
                     boids_[j].position()) < ds_) {

            result = result - (boids_[j].position() - boids_[i].position()) * s_;
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

        if (distance(boids_[i].position(),
                     boids_[j].position()) < d_) {

            velocity_diff_sum = velocity_diff_sum + 
                (boids_[j].velocity() - boids_[i].velocity());

            ++neighbours;
        }
    }

    if (neighbours == 0) {
        return Vector3{0., 0., 0.};
    }

    double const n = static_cast<double>(neighbours);

    return velocity_diff_sum * a_ / n; //da chiedere a chat
}

Vector3 Flock::cohesion(std::size_t i) const {
    Vector3 position_sum{0., 0., 0.};
    std::size_t neighbours{0};

    for (std::size_t j = 0; j < boids_.size(); ++j) {

        if (j == i) {
            continue;
        }

        if (distance(boids_[i].position(),
                     boids_[j].position()) < d_) {

            position_sum = position_sum + boids_[j].position();

            ++neighbours;
        }
    }

    if (neighbours == 0) {
        return Vector3{0., 0.,0.};
    }

    double const n = static_cast<double>(neighbours);
    Vector3 const centre = position_sum / n; 

    return (centre - boids_[i].position()) * c_;
}

void Flock::update(double dt)
{
    std::vector<Boid> new_boids = boids_;

    for (std::size_t i = 0; i < boids_.size(); ++i) {

        Vector3 new_velocity = boids_[i].velocity() + separation(i) + alignment(i) + cohesion(i);

        double const speed = norm(new_velocity);
        if (speed > max_speed_) { new_velocity = new_velocity * (max_speed_ / speed);
}

        Vector3 const new_position = boids_[i].position() + new_velocity * dt;
        wrap_position (new_position);

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

            sum += distance(
                boids_[i].position(),
                boids_[j].position());

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

            double const difference =
                distance(
                    boids_[i].position(),
                    boids_[j].position()) - mean;

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