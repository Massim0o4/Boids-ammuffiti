#include "vector3.hpp"
#include <cmath>

namespace boids {

Vector3 operator+(Vector3 const& l, Vector3 const& r) {
  return Vector3{l.x + r.x, l.y + r.y, l.z + r.z};
}

Vector3 operator-(Vector3 const& l, Vector3 const& r) {
    return Vector3{l.x - r.x, l.y - r.y, l.z - r.z};
}

Vector3 operator*(Vector3 const& l, double scalar) {
    return Vector3{l.x * scalar , l.y * scalar , l.z * scalar};}

Vector3 operator/(Vector3 const& l, double scalar) {
    return Vector3{l.x / scalar, l.y / scalar, l.z / scalar};}


bool operator==(Vector3 const& a, Vector3 const& b){
    return a.x == b.x && a.y == b.y && a.z == b.z;}


double norm(Vector3 const& v) {
    return  (std::sqrt( v.x * v.x + v.y * v.y + v.z * v.z));}

double distance(Vector3 const& a, Vector3 const& b) {
    Vector3 diff = a - b;
    return norm (diff);}

double scalar_product(Vector3 const& a, Vector3 const& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z ; 
}

}