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

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Image Stream plugin initialized.");
        }

        bool get_stream() {
            return true;
        }

    private:
        double side_length_;
    };

};
#endif // LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
