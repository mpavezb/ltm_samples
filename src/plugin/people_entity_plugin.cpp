#include <ltm_samples/plugin/people_entity_plugin.h>
#include <ltm/parameter_server_wrapper.h>
#include <sensor_msgs/Image.h>

namespace ltm_samples
{
    // =================================================================================================================
    // Public API
    // =================================================================================================================

    void PeopleEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name) {
        _log_prefix = "[LTM][People Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        ltm::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "topic", _stm_topic, "/robot/fake_short_term_memory/person/updates");

        ros::NodeHandle priv("~");
        _sub = priv.subscribe(_stm_topic, 1, &PeopleEntityPlugin::callback, this);

        // DB connection
        this->ltm_setup(param_ns, db_ptr, db_name);

        // init ROS interface
        this->ltm_init();
    }

    PeopleEntityPlugin::~PeopleEntityPlugin() {

    }

    void PeopleEntityPlugin::register_episode(uint32_t uid) {
        this->ltm_register_episode(uid);
    }

    void PeopleEntityPlugin::unregister_episode(uint32_t uid) {

    }

    void PeopleEntityPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) {

    }

    // =================================================================================================================
    // Private API
    // =================================================================================================================

    MetadataPtr PeopleEntityPlugin::make_metadata(const EntityType &entity) {

    }

    void PeopleEntityPlugin::callback(const EntityType& msg) {
        ROS_WARN_STREAM("Received person msg: uid: " << msg.uid);

        EntityType null_e;
        build_null(null_e);
    }

    void PeopleEntityPlugin::build_null(EntityType &entity) {
        entity.uid = 0;
        entity.name = "";
        entity.last_name = "";
        entity.genre = 2;
        entity.country = "";
        entity.city = "";
        entity.birthday.year = 0;
        entity.birthday.month = 0;
        entity.birthday.day = 0;
        entity.age = 0;
        entity.body = sensor_msgs::Image();
        entity.face = sensor_msgs::Image();
        entity.emotion = "";
        entity.stance = "";
        entity.last_seen = ros::Time(0);
        entity.last_interacted = ros::Time(0);
    }

}