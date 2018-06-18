#include <ltm_samples/plugin/objects_entity_plugin.h>
#include <ltm/util/parameter_server_wrapper.h>

namespace ltm_samples
{

    void ObjectsEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) {
        _log_prefix = "[LTM][Objects Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        // ROS Parameter Server
        ltm::util::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "type", _type, "images");
        psw.getParameter(param_ns + "collection", _collection_name, "image_streams");

        // DB connection
        _conn = ptr;
        _db_name = db_name;
        setup_db();

        // ROS API
        ros::NodeHandle priv("~");
        _status_service = priv.advertiseService("entity/" + _type + "/status", &ObjectsEntityPlugin::status_service, this);
        _drop_db_service = priv.advertiseService("entity/" + _type + "/drop_db", &ObjectsEntityPlugin::drop_db_service, this);
//        _add_stream_service = priv.advertiseService("entity/" + _type + "/add", &ObjectsEntityPlugin::add_service, this);
//        _get_stream_service = priv.advertiseService("entity/" + _type + "/get", &ObjectsEntityPlugin::get_service, this);
//        _delete_stream_service = priv.advertiseService("entity/" + _type + "/delete", &ObjectsEntityPlugin::delete_service, this);
    }

    ObjectsEntityPlugin::~ObjectsEntityPlugin() {

    }

    MetadataPtr ObjectsEntityPlugin::make_metadata(const ObjectEntity &entity) {

    }

    bool ObjectsEntityPlugin::insert(const ObjectEntity &entity) {

    }

    bool ObjectsEntityPlugin::get(uint32_t uid, ObjectEntityWithMetadataPtr &entity_ptr) {

    }

    bool ObjectsEntityPlugin::update(uint32_t uid, const ObjectEntity &stream) {

    }

    void ObjectsEntityPlugin::register_episode(uint32_t uid) {

    }

    void ObjectsEntityPlugin::unregister_episode(uint32_t uid) {

    }

    void ObjectsEntityPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) {

    }

    std::string ObjectsEntityPlugin::get_type() {
        return _type;
    }

    std::string ObjectsEntityPlugin::get_collection_name() {
        return _collection_name;
    }

    // - - - - - - - - -  - DB API - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    bool ObjectsEntityPlugin::remove(uint32_t uid) {

    }

    int ObjectsEntityPlugin::count() {

    }

    bool ObjectsEntityPlugin::has(int uid) {

    }

    bool ObjectsEntityPlugin::is_reserved(int uid) {

    }

    bool ObjectsEntityPlugin::drop_db() {

    }

    void ObjectsEntityPlugin::setup_db() {

    }

    // ROS API
    bool ObjectsEntityPlugin::status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

    }

    bool ObjectsEntityPlugin::drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

    }

//        bool ObjectsEntityPlugin::add_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);
//        bool ObjectsEntityPlugin::get_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);
//        bool ObjectsEntityPlugin::delete_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);

}