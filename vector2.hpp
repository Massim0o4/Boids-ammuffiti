#ifndef VECTOR2_HPP
#define VECTOR2_HPP

namespace boids {
struct Vector2 {
  double x;
  double y;
};

Vector2 operator+(Vector2 const& l, Vector2 const& r);
Vector2 operator-(Vector2 const& l, Vector2 const& r);
Vector2 operator*(Vector2 const& v, double scalar);
Vector2 operator/(Vector2 const& v, double scalar);

bool operator==(Vector2 const& a, Vector2 const& b);

double norm(Vector2 const& v);
double distance(Vector2 const& a, Vector2 const& b);

}

#endif