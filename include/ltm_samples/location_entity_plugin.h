#ifndef LTM_SAMPLES_LOCATION_ENTITY_PLUGIN_H_
#define LTM_SAMPLES_LOCATION_ENTITY_PLUGIN_H_
#include <ltm/plugins_base.h>
#include <ros/ros.h>


namespace ltm_samples
{

    class LocationEntityPlugin : public ltm::plugin::EntityBase
    {
    public:
        LocationEntityPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Location Entities plugin initialized.");
        }

        bool get_entity() {
            return true;
        }

    private:
        double side_length_;
    };

};
#endif // LTM_SAMPLES_LOCATION_ENTITY_PLUGIN_H_
