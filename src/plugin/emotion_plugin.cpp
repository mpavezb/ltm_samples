#include <ltm_samples/plugin/emotion_plugin.h>

namespace ltm_samples
{
    void EmotionPlugin::initialize(const std::string &param_ns) {
        log_prefix = "[LTM][Emotion]: ";
        ROS_DEBUG_STREAM(log_prefix << "Initializing with ns: " << param_ns);
        ros::NodeHandle priv("~");
        client = priv.serviceClient<ltm::GetEpisode>("/robot/fake/get_emotion");

        initialized = false;
        reinitialize();
    }

    void EmotionPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "registering episode " << uid);
        registry[uid] = ros::Time::now();

        if (!initialized) reinitialize();
    }

    void EmotionPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "unregistering episode " << uid);
        registry.erase(uid);
    }

    void EmotionPlugin::collect(uint32_t uid, ltm::EmotionalRelevance &msg) {
        ROS_DEBUG_STREAM(log_prefix << "collecting episode " << uid);
        if (!initialized && !reinitialize()) {
            ROS_WARN_STREAM(log_prefix << "Required service is not present (" << client.getService() << ").");
            return;
        }
        ltm::GetEpisode srv;
        if (client.call(srv)) {
            msg = srv.response.episode.relevance.emotional;
        } else {
            ROS_WARN_STREAM(log_prefix << "service call failed.(" << client.getService() << ").");
        }
    }


    bool EmotionPlugin::reinitialize() {
        if (!client.waitForExistence(ros::Duration(2.0))) {
            ROS_WARN_STREAM(log_prefix << "Couldn't find service " << client.getService() << ".");
            return false;
        }
        ROS_INFO_STREAM(log_prefix << "initialized.");
        initialized = true;
        return true;
    }

    void EmotionPlugin::reset() {

    }

}
