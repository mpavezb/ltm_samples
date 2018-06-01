#ifndef LTM_SAMPLES_ROBOT_ENTITY_PLUGIN_H_
#define LTM_SAMPLES_ROBOT_ENTITY_PLUGIN_H_

#include <ltm/plugin/entity_base.h>
#include <ros/ros.h>

namespace ltm_samples
{
    class RobotEntityPlugin : public ltm::plugin::EntityBase
    {
    public:
        RobotEntityPlugin(){}

        void initialize(const std::string& param_ns) {
            ROS_DEBUG_STREAM("LTM Robot Entity plugin initialized with ns: " << param_ns);
        }

        void register_episode(uint32_t uid) {
            ROS_DEBUG_STREAM("LTM Robot Entity plugin: registering episode " << uid);
        }

        void unregister_episode(uint32_t uid) {
            ROS_DEBUG_STREAM("LTM Robot Entity plugin: unregistering episode " << uid);
        }

        void collect(uint32_t uid, ltm::What& msg) {
            ROS_DEBUG_STREAM("LTM Robot Entity plugin: collecting episode " << uid);
        }
    };

};
#endif // LTM_SAMPLES_ROBOT_ENTITY_PLUGIN_H_
