#ifndef IWBC_BEHAVIOR_GENERIC_MOVE_HPP
#define IWBC_BEHAVIOR_GENERIC_MOVE_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            class Move : public Behavior {
            public:
                Move(const controller_ptr_t& controller, const YAML::Node& config);
                Move() = delete;
                Move(const Move&) = delete;
                virtual ~Move() {}

                void set_trajectory_target(const std::string& task_name, const pinocchio::SE3& target, float duration);
//                void set_immediate_target(const pinocchio::SE3& target, float duration);

                bool target_reached(const std::string& task_name) const {
                    // trajectories are removed once they have reached their last point
                    return trajectories_.find(task_name) != trajectories_.end();
                }

                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                struct Traj {
                    std::vector<pinocchio::SE3> points;
                    int time = 0;
                };
                std::shared_ptr<controllers::PosTracker> pos_tracker_;
                std::unordered_map<std::string,Traj> trajectories_;
            };
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc
#endif