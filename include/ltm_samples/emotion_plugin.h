#ifndef LTM_SAMPLES_EMOTION_PLUGIN_H_
#define LTM_SAMPLES_EMOTION_PLUGIN_H_

#include <ltm/plugins_base.h>
#include <ros/ros.h>

namespace ltm_samples {
    class EmotionPlugin : public ltm::plugin::EmotionBase {
    public:
        EmotionPlugin() {}

        void initialize(const std::string &param_ns) {
            ROS_INFO_STREAM("LTM Emotion plugin initialized with ns: " << param_ns);
        }

        void register_episode(uint32_t uid) {
            ROS_INFO_STREAM("LTM Emotion plugin: registering episode " << uid);
        }

        void unregister_episode(uint32_t uid) {
            ROS_INFO_STREAM("LTM Emotion plugin: unregistering episode " << uid);
        }

        void collect(uint32_t uid, ltm::EmotionalRelevance &msg) {
            ROS_INFO_STREAM("LTM Emotion plugin: collecting episode " << uid);
        }
    };

};
#endif // LTM_SAMPLES_EMOTION_PLUGIN_H_
