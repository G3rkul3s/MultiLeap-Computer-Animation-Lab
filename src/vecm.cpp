#include "vecm.h"

namespace vecm
{
    // Function to calculate the angle between two vectors (in radians)
    float angleBetweenVectors(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2)
    {
        float dotProduct = vec1.dot(vec2);
        float norms = vec1.norm() * vec2.norm();

        // Check for zero norm to avoid division by zero
        if (norms == 0.0f)
        {
            throw std::invalid_argument("One or both vectors have zero length.");
        }

        // Compute cosine of the angle
        float cosineTheta = dotProduct / norms;
        // Clamp cosineTheta to the range [-1, 1] to handle numerical precision issues
        cosineTheta = std::max(-1.0f, std::min(1.0f, cosineTheta));

        // Return the angle in radians
        return std::acos(cosineTheta);
    }

    // Function to calculate the distance between two vectors
    float distanceBetweenVectors(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2)
    {
        if (vec1.size() != vec2.size())
        {
            throw std::invalid_argument("Vectors must have the same size");
        }

        return (vec1 - vec2).norm();
    }

} // namespace vecm
