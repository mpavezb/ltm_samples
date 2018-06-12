#ifndef LTM_SAMPLES_LOCATION_PLUGIN_H_
#define LTM_SAMPLES_LOCATION_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/location_base.h>
#include <ltm/GetEpisode.h>

namespace ltm_samples
{
    class LocationPlugin : public ltm::plugin::LocationBase
    {
    public:
        LocationPlugin(){}

        void initialize(const std::string& param_ns);

        void register_episode(uint32_t uid);

        void unregister_episode(uint32_t uid);

        void collect(uint32_t uid, ltm::Where& msg);

        void reset();

    private:
        ros::ServiceClient client;
        std::map<uint32_t, ros::Time> registry;
        bool initialized;
        std::string log_prefix;
        bool reinitialize();
    };

};
#endif // LTM_SAMPLES_LOCATION_PLUGIN_H_
