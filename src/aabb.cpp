#include "aabb.h"

// template <typename S>
// AABB<S>::AABB() 
//     : min_(Vector3<S>::Constant(std::numeric_limits<S>::max())),
//     max_(Vector3<S>::Constant(-std::numeric_limits<S>::max()))
// {

// }

// template <typename S>
// AABB<S>::AABB(const Vector3<S>& a, const Vector3<S>& b)
//   : min_(a.cwiseMin(b)),
//     max_(a.cwiseMax(b))
// {
//   // Do nothing
// }

template <typename S>
bool AABB<S>::overlap(const AABB<S>& other) const
{
  if ((min_.array() > other.min_.array()).any())
    return false;

  if ((max_.array() < other.max_.array()).any())
    return false;

  return true;
}

template <typename S>
AABB<S>& AABB<S>::operator +=(const AABB<S>& other)
{
  min_ = min_.cwiseMin(other.min_);
  max_ = max_.cwiseMax(other.max_);
  return *this;
}

template <typename S>
AABB<S> AABB<S>::operator +(const AABB<S>& other) const
{
    AABB res(*this);
    return res += other;
}


//==============================================================================
template <typename S>
S AABB<S>::width() const
{
  return max_[0] - min_[0];
}

//==============================================================================
template <typename S>
S AABB<S>::height() const
{
  return max_[1] - min_[1];
}

//==============================================================================
template <typename S>
S AABB<S>::depth() const
{
  return max_[2] - min_[2];
}

//==============================================================================
template <typename S>
S AABB<S>::size() const
{
  return (max_ - min_).squaredNorm();
}


//==============================================================================
template <typename S>
Vector3<S> AABB<S>::center() const
{
  return (min_ + max_) * 0.5;
}


// 显式实例化模板类 以防万一可以加一个实例化类
template class AABB<double>;
// template class AABB<float>;