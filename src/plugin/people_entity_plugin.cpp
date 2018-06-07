#include <ltm_samples/plugin/people_entity_plugin.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm_samples
{
    // =================================================================================================================
    // Public API
    // =================================================================================================================

    void PeopleEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) {
        _log_prefix = "[LTM][People Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);


    }

    PeopleEntityPlugin::~PeopleEntityPlugin() {

    }

    void PeopleEntityPlugin::register_episode(uint32_t uid) {

    }

    void PeopleEntityPlugin::unregister_episode(uint32_t uid) {

    }

    void PeopleEntityPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) {

    }

    // =================================================================================================================
    // Private API
    // =================================================================================================================

    MetadataPtr PeopleEntityPlugin::make_metadata(const PersonEntity &entity) {

    }

}