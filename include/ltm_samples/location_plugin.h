#ifndef LTM_SAMPLES_LOCATION_PLUGIN_H_
#define LTM_SAMPLES_LOCATION_PLUGIN_H_
#include <ltm/plugins_base.h>
#include <ros/ros.h>

namespace ltm_samples
{

    class LocationPlugin : public ltm::plugin::LocationBase
    {
    public:
        LocationPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Location plugin initialized.");
        }

        bool get_location() {
            return true;
        }

    private:
        double side_length_;
    };

};
#endif // LTM_SAMPLES_LOCATION_PLUGIN_H_
