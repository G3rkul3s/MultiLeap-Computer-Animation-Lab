#ifndef VECTORMATH_H
#define VECTORMATH_H

#pragma once

#include "pch.h"

namespace vecm
{
    // Function to calculate the angle between two vectors (in radians)
    float angleBetweenVectors(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2);

    // Function to calculate the distance between two vectors
    float distanceBetweenVectors(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2);

} // namespace VectorMath

#endif // VECTORMATH_H
