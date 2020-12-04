// C++ program to find the Shortest
// Distance Between A line and a
// Given point.
#include<bits/stdc++.h>
#include<cmath>
using namespace std;

namespace sds
{
  class Vector {
  private:
      float x, y, z;
      // 3D Coordinates of the Vector

  public:
      Vector(float x, float y, float z)
      {
          // Constructor
          this->x = x;
          this->y = y;
          this->z = z;
      }
      Vector operator+(Vector v); // ADD 2 Vectors
      Vector operator-(Vector v); // Subtraction
      float operator^(Vector v); // Dot Product
      Vector operator*(Vector v); // Cross Product
      float magnitude()
      {
          return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      }
      friend ostream& operator<<(ostream& out, const Vector& v);
      // To output the Vector
  };

  // ADD 2 Vectors
  Vector Vector::operator+(Vector v)
  {
      float x1, y1, z1;
      x1 = x + v.x;
      y1 = y + v.y;
      z1 = z + v.z;
      return Vector(x1, y1, z1);
  }

  // Subtract 2 vectors
  Vector Vector::operator-(Vector v)
  {
      float x1, y1, z1;
      x1 = x - v.x;
      y1 = y - v.y;
      z1 = z - v.z;
      return Vector(x1, y1, z1);
  }

  // Dot product of 2 vectors
  float Vector::operator^(Vector v)
  {
      float x1, y1, z1;
      x1 = x * v.x;
      y1 = y * v.y;
      z1 = z * v.z;
      return (x1 + y1 + z1);
  }

  // Cross product of 2 vectors
  Vector Vector::operator*(Vector v)
  {
      float x1, y1, z1;
      x1 = y * v.z - z * v.y;
      y1 = z * v.x - x * v.z;
      z1 = x * v.y - y * v.x;
      return Vector(x1, y1, z1);
  }

  // Display Vector
  ostream& operator<<(ostream& out,
                      const Vector& v)
  {
      out << v.x << "i ";
      if (v.y >= 0)
          out << "+ ";
      out << v.y << "j ";
      if (v.z >= 0)
          out << "+ ";
      out << v.z << "k" << endl;
      return out;
  }

  // calculate shortest dist. from point to line
  float shortDistance(Vector line_point1, Vector line_point2,
                      Vector point)
  {
      Vector AB = line_point2 - line_point1;
      Vector AC = point - line_point1;
      float area = Vector(AB * AC).magnitude();
      float CD = area / AB.magnitude();
      return CD;
  }

  float angleToVector2D(Vector point, float angle, char axis)
  {

  }

}


// // Example Usage
// int main()
// {
//     // Taking point C as (2, 2, 0)
//     // Line Passes through A(4, 0, 0)
//     // and B(3, 1, 0).
//     Vector line_point1(4, 0, 0), line_point2(3, 1, 0);
//     Vector point(2, 2, 0);
//
//     cout << "Shortest Distance is : "
//          << shortDistance(line_point1, line_point2, point)
//          << endl;
//
//   return 0;
// }
