#ifndef UAV_INSPECTION_COMMON_HPP
#define UAV_INSPECTION_COMMON_HPP

#include <cmath>
#include <array>
#include <vector>

// ============ 坐标系转换 ============
// PX4: NED (North-East-Down)
// Gazebo: ENU (East-North-Up)

namespace CoordinateTransform {
    inline std::array<double, 3> enu_to_ned(const std::array<double, 3>& enu) {
        return {enu[1], enu[0], -enu[2]};
    }
    
    inline std::array<double, 3> ned_to_enu(const std::array<double, 3>& ned) {
        return {ned[1], ned[0], -ned[2]};
    }
}

// ============ 数学工具 ============
namespace MathUtils {
    inline double euclidean_distance(
        const std::array<double, 3>& p1,
        const std::array<double, 3>& p2) {
        double dx = p1[0] - p2[0];
        double dy = p1[1] - p2[1];
        double dz = p1[2] - p2[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    inline double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
}

#endif
