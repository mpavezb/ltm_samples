#ifndef LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_
#define LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/entity_base.h>
#include <ltm/plugin/entity_default.h>
#include <ltm_samples/PersonEntity.h>
#include <ltm_samples/PersonEntitySrv.h>
#include <std_srvs/Empty.h>

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
        typedef ltm_samples::PersonEntity EntityType;
        typedef warehouse_ros::MessageWithMetadata<EntityType> EntityWithMetadata;
        typedef boost::shared_ptr<const EntityWithMetadata> EntityWithMetadataPtr;

        // Log Message Types
        typedef ltm::EntityLog LogType;
        typedef warehouse_ros::MessageWithMetadata<LogType> LogWithMetadata;
        typedef boost::shared_ptr<const LogWithMetadata> LogWithMetadataPtr;

        // plugin specific variables
        EntityType _null_e;

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

        std::string build_log_vector(const std::vector<std::string> &v);

        MetadataPtr make_metadata(const EntityType &entity);
        MetadataPtr make_log_metadata(const LogType &log);

        void callback(const EntityType &msg);

        void build_null(EntityType &entity);

        template <typename T> bool field_equals(const T &A, const T &B);
        template <typename T> void update_field(LogType& log, const std::string &field, T &curr_e, T &log_e,  const T &new_e, const T &null_e);


    public:
        void initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name);
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end);
        void drop_db();
        void append_status(std::stringstream &status);
    };


};
#endif // LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_
