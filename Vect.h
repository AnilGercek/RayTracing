#ifndef _VECT_H
#define _VECT_H

#include "math.h"

class Vect
{
  public:
    float x, y, z;

    Vect(float _x = 0, float _y = 0, float _z = 0)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    float dotProduct(Vect v)
    {
        return x * v.x + y * v.y + z * v.z;
    };

    Vect crossProduct(Vect v)
    {
        return Vect(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    Vect vectorAdd(Vect v)
    {
        return Vect(x + v.x, y + v.y, z + v.z);
    }

    Vect vectorSubtract(Vect v)
    {
        return Vect(x - v.x, y - v.y, z - v.z);
    }

    Vect vectorNegative()
    {
        return Vect(-1 * x, -1 * y, -1 * z);
    }

    Vect vectorMult(float s)
    {
        return Vect(s * x, s * y, s * z);
    }

    float magnitude()
    {
        return sqrt((x * x) + (y * y) + (z * z));
    }

    Vect normalize()
    {
        float m = this->magnitude();
        return Vect(x / m, y / m, z / m);
    }

    float distance(Vect v)
    {
        return sqrt(pow((x - v.x), 2) + pow((y - v.y), 2) + pow((z - v.z), 2));
    }
};

#endif