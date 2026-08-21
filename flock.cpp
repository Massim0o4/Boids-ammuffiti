#include "flock.hpp"

#include <cmath>

namespace boids {

Flock::Flock(std::vector<Boid> boids, double d, double ds, double s, double a, double c)
    : boids_{boids}, d_{d}, ds_{ds}, s_{s}, a_{a}, c_{c} {} 

Vector2 Flock::separation(std::size_t i) const {
    Vector2 result{0., 0.};

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

Vector2 Flock::alignment(std::size_t i) const
{
    Vector2 velocity_diff_sum{0., 0.};
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
        return Vector2{0., 0.};
    }

    double const n = static_cast<double>(neighbours);

    return velocity_diff_sum * a_ / n; //da chiedere a chat
}

Vector2 Flock::cohesion(std::size_t i) const {
    Vector2 position_sum{0., 0.};
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
        return Vector2{0., 0.};
    }

    double const n = static_cast<double>(neighbours);
    Vector2 const centre = position_sum / n; 

    return (centre - boids_[i].position()) * c_;
}

void Flock::update(double dt)
{
    std::vector<Boid> new_boids = boids_;

    for (std::size_t i = 0; i < boids_.size(); ++i) {

        Vector2 const new_velocity =
            boids_[i].velocity() + separation(i) + alignment(i) + cohesion(i);

        Vector2 const new_position =
            boids_[i].position() + new_velocity * dt;

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