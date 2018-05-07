#ifndef LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
#define LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
#include <ltm/plugins_base.h>
#include <ros/ros.h>

namespace ltm_samples
{
    class ImageStreamPlugin : public ltm::plugin::StreamBase
    {
    public:
        ImageStreamPlugin(){}

        void initialize(const std::string& param_ns) {
            ROS_INFO_STREAM("LTM Image Stream plugin initialized with ns: " << param_ns);
        }

        void register_episode(uint32_t uid) {
            ROS_INFO_STREAM("LTM Image Stream plugin: registering episode " << uid);
        }

        void unregister_episode(uint32_t uid) {
            ROS_INFO_STREAM("LTM Image Stream plugin: unregistering episode " << uid);
        }

        void collect(uint32_t uid, ltm::What& msg) {
            ROS_INFO_STREAM("LTM Image Stream plugin: collecting episode " << uid);
        }
    };

};
#endif // LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
