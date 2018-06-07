#ifndef LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_
#define LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/entity_base.h>

#include <ltm_samples/PersonEntity.h>
#include <std_srvs/Empty.h>

#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/database_connection.h>

typedef warehouse_ros::MessageCollection<ltm_samples::PersonEntity> PersonEntityCollection;
typedef boost::shared_ptr<PersonEntityCollection> PersonEntityCollectionPtr;

typedef warehouse_ros::MessageWithMetadata<ltm_samples::PersonEntity> PersonEntityWithMetadata;
typedef boost::shared_ptr<const PersonEntityWithMetadata> PersonEntityWithMetadataPtr;

namespace ltm_samples
{
    class PeopleEntityPlugin : public ltm::plugin::EntityBase
    {
    private:

        // plugin
        std::string _log_prefix;
        std::string _type;
        std::string _collection_name;
        std::vector<uint32_t> registry;

        // ROS API
        ros::ServiceServer _status_service;
        ros::ServiceServer _drop_db_service;
        ros::ServiceServer _add_stream_service;
        ros::ServiceServer _get_stream_service;
        ros::ServiceServer _delete_stream_service;

        // database
        DBConnectionPtr _conn;
        PersonEntityCollectionPtr _coll;
        std::string _db_name;

        // DB API
        MetadataPtr make_metadata(const PersonEntity &entity);
        bool insert(const PersonEntity &entity);
        bool get(uint32_t uid, PersonEntityWithMetadataPtr &entity_ptr);
        bool update(uint32_t uid, const PersonEntity &stream);

    public:
        PeopleEntityPlugin(){}
        ~PeopleEntityPlugin();

        void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name);
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end);
        std::string get_type();
        std::string get_collection_name();

        // - - - - - - - - -  - DB API - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        bool remove(uint32_t uid);
        int count();
        bool has(int uid);
        bool is_reserved(int uid);
        bool drop_db();
        void setup_db();

        // ROS API
        bool status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
        bool drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
//        bool add_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);
//        bool get_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);
//        bool delete_service(ltm_samples::ImageStreamSrv::Request  &req, ltm_samples::ImageStreamSrv::Response &res);
    };

};
#endif // LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_
