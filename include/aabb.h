
#ifndef AABB_H
#define AABB_H

#include <iostream>
#include <Eigen/Geometry>
#include <limits>
#include "types.h"

// #include "collision_object.h"

// template <typename S>
// using Vector3 = Eigen::Matrix<S, 3, 1>;

// template <typename S>
// using Matrix3 = Eigen::Matrix<S, 3, 3>;

// template <typename S>
// using Transform3 = Eigen::Transform<S, 3, Eigen::Isometry>;

// template <typename S>
// using Quaternion = Eigen::Quaternion<S>;

template <typename S_>
class AABB
{
public:
    using S = S_;

    Vector3<S> min_;
    Vector3<S> max_;

    AABB()
        : min_(Vector3<S>::Constant(std::numeric_limits<S>::max())),
          max_(Vector3<S>::Constant(-std::numeric_limits<S>::max()))
    {
    }

    // AABB(const Vector3<S>& v);
    AABB(const Vector3<S>& a, const Vector3<S>& b)
        : min_(a.cwiseMin(b)), max_(a.cwiseMax(b))
    {
    }

    /// @brief Check whether two AABB are overlap
    bool overlap(const AABB<S>& other) const;

    /// @brief Merge the AABB and another AABB
    AABB<S>& operator += (const AABB<S>& other);

    /// @brief Return the merged AABB of current AABB and the other one
    AABB<S> operator + (const AABB<S>& other) const;

    /// @brief Width of the AABB
    S width() const;

    /// @brief Height of the AABB
    S height() const;

    /// @brief Depth of the AABB
    S depth() const;

    S size() const;

    /// @brief Center of the AABB
    Vector3<S> center() const;
        
};

#endif
