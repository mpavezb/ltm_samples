#ifndef LTM_SAMPLES_ROBOT_ENTITY_PLUGIN_H_
#define LTM_SAMPLES_ROBOT_ENTITY_PLUGIN_H_
#include <ltm/plugins_base.h>
#include <ros/ros.h>


namespace ltm_samples
{

    class RobotEntityPlugin : public ltm::plugin::EntityBase
    {
    public:
        RobotEntityPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Robot Entities plugin initialized.");
        }

        bool get_entity() {
            return true;
        }

    private:
        double side_length_;
    };

};
#endif // LTM_SAMPLES_ROBOT_ENTITY_PLUGIN_H_
