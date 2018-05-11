#include <ltm_samples/image_stream_plugin.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm_samples
{
    void ImageStreamPlugin::initialize(const std::string& param_ns) {
        _log_prefix = "[LTM][ImageStream plugin]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        // ROS Parameter Server
        double buffer_frequency;
        ltm::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "type", _type, "images");
        psw.getParameter(param_ns + "collection", _collection_name, "image_streams");
        psw.getParameter(param_ns + "topic", _image_topic, "/robot/fake/sensors/camera/image_raw");
        psw.getParameter(param_ns + "buffer_frequency", buffer_frequency, 3.0);
        psw.getParameter(param_ns + "buffer_size", _buffer_max_size, 100);
        if (_buffer_max_size < 10) {
            ROS_WARN_STREAM(_log_prefix << "'buffer_size' param is too low. Setting to minimum: 10");
            _buffer_max_size = 10;
        }
        if (_buffer_max_size > 1000) {
            ROS_WARN_STREAM(_log_prefix << "'buffer_size' param is too high. Setting to maximum: 1000");
            _buffer_max_size = 1000;
        }
        if (buffer_frequency > 10.0) {
            ROS_WARN_STREAM(_log_prefix << "'buffer_frequency' param is too high. The server might slow down because of this.");
        }

        // timing
        _buffer_period = ros::Duration((1.0 / buffer_frequency));
        _last_callback = ros::Time(0.0);

        // setup
        _buffer.reserve((size_t)_buffer_max_size);
        _buffer.resize((size_t)_buffer_max_size);
        _last_idx = (size_t)(_buffer_max_size - 1);
        _buffer_size = 0;
    }

    std::string ImageStreamPlugin::get_type() {
        return _type;
    }

    std::string ImageStreamPlugin::get_collection_name() {
        return _collection_name;
    }

    void ImageStreamPlugin::subscribe() {
        ros::NodeHandle priv("~");
        _image_sub = priv.subscribe(_image_topic, 2, &ImageStreamPlugin::image_callback, this,
                                    ros::TransportHints().unreliable().reliable());
    }

    void ImageStreamPlugin::unsubscribe() {
        _image_sub.shutdown();
    }

    void ImageStreamPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: registering episode " << uid);

        // subscribe on demand
        if (registry.empty()) subscribe();

        // register in cache
        std::vector<uint32_t>::const_iterator it = std::find(registry.begin(), registry.end(), uid);
        if (it == registry.end()) registry.push_back(uid);
    }

    void ImageStreamPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: unregistering episode " << uid);
        // unregister from cache
        std::vector<uint32_t>::iterator it = std::find(registry.begin(), registry.end(), uid);
        if (it != registry.end()) registry.erase(it);

        // unsubscribe on demand
        if (registry.empty()) unsubscribe();
    }

    void ImageStreamPlugin::collect(uint32_t uid, ltm::What& msg) {
        ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: collecting episode " << uid);

        // save into Image Stream collection
        // TODO

        // append to msg
        msg.streams.push_back(get_type());

        // unregister
        // TODO: redundant calls to (un)register methods?
        unregister_episode(uid);
    }

    void ImageStreamPlugin::degrade(uint32_t uid) {
        // TODO
    }

    void ImageStreamPlugin::image_callback(const sensor_msgs::ImageConstPtr& msg) {
        // keep buffer period
        ros::Time now = ros::Time::now();
        if (now - _last_callback < _buffer_period) return;
        _last_callback = now;

        // fill buffer
        _buffer_size = std::min((size_t)_buffer_max_size, _buffer_size + 1);
        _last_idx = (_last_idx + 1) % _buffer_max_size;
        _buffer[_last_idx] = msg;
        ROS_DEBUG_STREAM(_log_prefix << "Received new image (" << (_last_idx + 1) << "/" << _buffer_max_size << ").");
    }

};
