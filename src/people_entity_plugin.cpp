#include <ltm_samples/people_entity_plugin.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm_samples
{

    void PeopleEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) {
        _log_prefix = "[LTM][People Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        // ROS Parameter Server
        ltm::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "type", _type, "images");
        psw.getParameter(param_ns + "collection", _collection_name, "image_streams");

        // DB connection
        _conn = ptr;
        _db_name = db_name;
        setup_db();

        // ROS API
        ros::NodeHandle priv("~");
        _status_service = priv.advertiseService("entity/" + _type + "/status", &PeopleEntityPlugin::status_service, this);
        _drop_db_service = priv.advertiseService("entity/" + _type + "/drop_db", &PeopleEntityPlugin::drop_db_service, this);
//        _add_stream_service = priv.advertiseService("entity/" + _type + "/add", &PeopleEntityPlugin::add_service, this);
//        _get_stream_service = priv.advertiseService("entity/" + _type + "/get", &PeopleEntityPlugin::get_service, this);
//        _delete_stream_service = priv.advertiseService("entity/" + _type + "/delete", &PeopleEntityPlugin::delete_service, this);
    }

    PeopleEntityPlugin::~PeopleEntityPlugin() {

    }

    MetadataPtr PeopleEntityPlugin::make_metadata(const PersonEntity &entity) {

    }

    bool PeopleEntityPlugin::insert(const PersonEntity &entity) {

    }

    bool PeopleEntityPlugin::get(uint32_t uid, PersonEntityWithMetadataPtr &entity_ptr) {

    }

    bool PeopleEntityPlugin::update(uint32_t uid, const PersonEntity &stream) {

    }

    void PeopleEntityPlugin::register_episode(uint32_t uid) {

    }

    void PeopleEntityPlugin::unregister_episode(uint32_t uid) {

    }

    void PeopleEntityPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) {

    }

    std::string PeopleEntityPlugin::get_type() {
        return _type;
    }

    std::string PeopleEntityPlugin::get_collection_name() {
        return _collection_name;
    }

    // - - - - - - - - -  - DB API - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    bool PeopleEntityPlugin::remove(uint32_t uid) {

    }

    int PeopleEntityPlugin::count() {

    }

    bool PeopleEntityPlugin::has(int uid) {

    }

    bool PeopleEntityPlugin::is_reserved(int uid) {

    }

    bool PeopleEntityPlugin::drop_db() {

    }

    void PeopleEntityPlugin::setup_db() {

    }

    // ROS API
    bool PeopleEntityPlugin::status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

    }

    bool PeopleEntityPlugin::drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

    }

//        bool PeopleEntityPlugin::add_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);
//        bool PeopleEntityPlugin::get_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);
//        bool PeopleEntityPlugin::delete_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);

}