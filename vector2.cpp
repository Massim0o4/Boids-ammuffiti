#include "vector2.hpp"
#include "flock.hpp"

#include <cmath>
namespace boids {

Vector2 operator+(Vector2 const& l, Vector2 const& r) {
  return Vector2{l.x + r.x, l.y + r.y};
}

Vector2 operator-(Vector2 const& l, Vector2 const& r) {
    return Vector2{l.x - r.x, l.y - r.y};
}

Vector2 operator*(Vector2 const& l, double scalar) {
    return Vector2{l.x * scalar , l.y * scalar };}

Vector2 operator/(Vector2 const& l, double scalar) {
    return Vector2{l.x / scalar, l.y / scalar};}


bool operator==(Vector2 const& a, Vector2 const& b){
    return a.x == b.x && a.y == b.y;}


double norm(Vector2 const& v) {
    return  (std::sqrt( v.x * v.x + v.y * v.y ));}

double distance(Vector2 const& a, Vector2 const& b) {
    Vector2 diff = a - b;
    return norm (diff);}

}
