#include "tiago_controller/behavior_move.hpp"

namespace inria_wbc
{
    namespace behaviors
    {
        namespace generic
        {
            static Register<Move> __talos_move_arm("generic::move");

            Move::Move(const controller_ptr_t &controller, const YAML::Node &config)
                : Behavior(controller, config)
            {
                auto c = IWBC_CHECK(config["BEHAVIOR"]);

                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                pos_tracker_ = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);

            }

            void Move::set_trajectory_target(const std::string &task_name, const pinocchio::SE3 &target, float duration)
            {
                // we assume that the target is valid (needs to be checked from outside!)
                auto task_init = pos_tracker_->get_se3_ref(task_name);
                auto pts = trajectory_handler::compute_traj(task_init, target, controller_->dt(), duration);
                trajectories_[task_name] = {.points = pts, .time = 0};
            }

            // void Move::set_immediate_target(const pinocchio::SE3 &target)
            // {
            //     // we assume a close point... otherwise the robot will move at full speed
            //     current_target_ = target;

            //     // switch to immediate target mode
            //     trajectory_.clear();
            //     time_ = 0;
            // }

            void Move::update(const controllers::SensorData &sensor_data)
            {
                for (auto it = trajectories_.begin(); it != trajectories_.end(); /*nothing*/)
                {
                    auto& traj = it->second;
                    IWBC_ASSERT(!traj.points.empty(), "invalid trajectory (empty)");
                    pos_tracker_->set_se3_ref(traj.points[traj.time], it->first);
                    traj.time++;
                    std::cout<<" time:"<<traj.time<<std::endl;

                    // increment or erase (end of trajectory)
                    if (traj.time == (int)traj.points.size())
                        it = trajectories_.erase(it);
                    else
                        it++;
                }
                controller_->update(sensor_data);
            }
        } // namespace generic
    }     // namespace behaviors
} // namespace inria_wbc