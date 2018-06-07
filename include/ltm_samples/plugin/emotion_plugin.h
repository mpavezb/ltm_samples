#ifndef LTM_SAMPLES_EMOTION_PLUGIN_H_
#define LTM_SAMPLES_EMOTION_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/emotion_base.h>
#include <ltm/GetEpisode.h>

namespace ltm_samples
{
    class EmotionPlugin : public ltm::plugin::EmotionBase
    {
    public:
        EmotionPlugin() {}

        void initialize(const std::string &param_ns);

        void register_episode(uint32_t uid);

        void unregister_episode(uint32_t uid);

        void collect(uint32_t uid, ltm::EmotionalRelevance &msg);

    private:
        ros::ServiceClient client;
        std::map<uint32_t, ros::Time> registry;
        bool initialized;
        std::string log_prefix;
        bool reinitialize();
    };

};
#endif // LTM_SAMPLES_EMOTION_PLUGIN_H_
