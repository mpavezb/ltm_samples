#ifndef LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_
#define LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/util/util.h>
#include <ltm/plugin/entity_base.h>
#include <ltm/plugin/entity_default.h>
#include <ltm_samples/PersonEntity.h>
#include <ltm_samples/PersonEntitySrv.h>
#include <ltm/QueryServer.h>
#include <std_srvs/Empty.h>
#include <ltm/plugin/entity_util.h>

namespace ltm_samples
{
    struct RegisterItem {
        ros::Time timestamp;
        uint32_t log_uid;
        uint32_t entity_uid;

        RegisterItem(ros::Time timestamp, uint32_t log_uid, uint32_t entity_uid) {
            this->timestamp = timestamp;
            this->log_uid = log_uid;
            this->entity_uid = entity_uid;
        }

    };
    struct RegisterItemComp {
        bool operator() (const RegisterItem& lhs, const RegisterItem& rhs) const
        {return lhs.timestamp < rhs.timestamp;}
    };

    class PeopleEntityPlugin :
            public ltm::plugin::EntityBase,
            public ltm::plugin::EntityDefault<ltm_samples::PersonEntity, ltm_samples::PersonEntitySrv>
    {
    private:
        // Entity Types
        typedef ltm_samples::PersonEntity EntityMsg;
        typedef ltm_db::MessageWithMetadata<EntityMsg> EntityWithMetadata;
        typedef boost::shared_ptr<const EntityWithMetadata> EntityWithMetadataPtr;

        // Log Message Types
        typedef ltm::EntityLog LogType;
        typedef ltm_db::MessageWithMetadata<LogType> LogWithMetadata;
        typedef boost::shared_ptr<const LogWithMetadata> LogWithMetadataPtr;

        // plugin specific variables
        EntityMsg _null_e;
        std::set<std::string> _field_names;

        // ROS API
        std::string _log_prefix;
        std::string _stm_topic;
        ros::Subscriber _sub;

        // timestamp registry (keep sorted by timestamp)
        typedef std::multiset<RegisterItem, RegisterItemComp> Registry;
        Registry _registry;

    public:
        PeopleEntityPlugin(){}
        ~PeopleEntityPlugin();

    private:
        void callback(const EntityMsg &msg);
        void build_null(EntityMsg &entity);

    public:
        std::string get_type();
        void initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name);
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end);
        void query(const std::string &json, ltm::QueryServer::Response &res, bool trail);
        void update(const EntityMsg &msg);
        void drop_db();
        void reset(const std::string &db_name);
        void append_status(std::stringstream &status);
        MetadataPtr make_metadata(const EntityMsg &entity);
        void retrace(EntityMsg &entity, const std::vector<uint32_t> &logs);
        void retrace_retrieve_field(const std::string& name, EntityWithMetadataPtr &in, EntityMsg &out);
    };


};
#endif // LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_
