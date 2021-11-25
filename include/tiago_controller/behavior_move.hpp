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

                void set_trajectory_target(const pinocchio::SE3& target);
                void set_immediate_target(const pinocchio::SE3& target);
                const pinocchio::SE3& current_target() const { return current_target_; }
                bool target_reached() const { return target_reached_; }

                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                std::shared_ptr<controllers::PosTracker> pos_tracker_;
                int time_ = 0;
                std::vector<pinocchio::SE3> trajectory_;
                float trajectory_duration_; // from YAML
                std::string task_name_; // from YAML
                pinocchio::SE3 current_target_; // form YAML
                bool target_reached_ = true;
            };
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc
#endif