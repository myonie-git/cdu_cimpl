
#include <cstddef>
#include <cstdint>
#include <vector>
#include <map>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/StdVector>

template <typename S>
using Vector3 = Eigen::Matrix<S, 3, 1>;

template <typename S>
using Matrix3 = Eigen::Matrix<S, 3, 3>;