#include <ltm_samples/image_stream_plugin.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm_samples
{
    void ImageStreamPlugin::initialize(const std::string& param_ns) {
        log_prefix = "[LTM][ImageStream plugin]: ";
        ROS_DEBUG_STREAM(log_prefix << "plugin initialized with ns: " << param_ns);

        // ROS Parameter Server
        double buffer_frequency;
        ltm::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "type", type, "images");
        psw.getParameter(param_ns + "collection", collection_name, "image_streams");
        psw.getParameter(param_ns + "topic", image_topic, "/robot/fake/sensors/camera/image_raw");
        psw.getParameter(param_ns + "buffer_frequency", buffer_frequency, 3.0);
        psw.getParameter(param_ns + "buffer_size", buffer_max_size, 100);

        // timing
        buffer_period = ros::Duration((1.0 / buffer_frequency));
        last_callback = ros::Time(0.0);

        // ROS API
        ros::NodeHandle priv("~");
        image_sub = priv.subscribe(image_topic, 2, &ImageStreamPlugin::image_callback, this,
                                   ros::TransportHints().unreliable().reliable());
    }

    std::string ImageStreamPlugin::get_type() {
        return type;
    }

    std::string ImageStreamPlugin::get_collection_name() {
        return collection_name;
    }

    void ImageStreamPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "LTM Image Stream plugin: registering episode " << uid);
    }

    void ImageStreamPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "LTM Image Stream plugin: unregistering episode " << uid);
    }

    void ImageStreamPlugin::collect(uint32_t uid, ltm::What& msg) {
        ROS_DEBUG_STREAM(log_prefix << "LTM Image Stream plugin: collecting episode " << uid);
    }

    void ImageStreamPlugin::degrade(uint32_t uid) {
        // TODO
    }

    void ImageStreamPlugin::image_callback(const sensor_msgs::Image::ConstPtr& msg) {
        // keep buffer period
        ros::Time now = ros::Time::now();
        if (now - last_callback < buffer_period) return;
        last_callback = now;

        // new msg
        ROS_WARN_STREAM(log_prefix << "Received new image.");
    }

};
