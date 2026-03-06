#ifndef UAV_INSPECTION_TYPES_HPP
#define UAV_INSPECTION_TYPES_HPP

#include <array>
#include <vector>

// ============ 基础数据结构 ============

struct Position {
    double x, y, z;
};

struct Velocity {
    double vx, vy, vz;
};

struct Attitude {
    double roll, pitch, yaw;
};

struct UAVState {
    Position position;
    Velocity velocity;
    Attitude attitude;
    double battery_level;
    bool is_armed;
    int8_t flight_mode;
};

struct GridCell {
    int x, y, z;
    bool occupied;
    int8_t distance_to_obstacle;
};

struct Waypoint {
    double x, y, z;
    double heading;
    double speed;
};

struct FormationConfig {
    int num_agents;
    std::vector<std::array<double, 3>> relative_positions;
    double formation_scale;
};

#endif
