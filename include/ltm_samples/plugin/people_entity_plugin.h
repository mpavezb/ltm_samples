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
    class PeopleEntityPlugin :
            public ltm::plugin::EntityBase,
            public ltm::plugin::EntityDefault<ltm_samples::PersonEntity, ltm_samples::PersonEntitySrv>
    {
    private:
        typedef ltm_samples::PersonEntity EntityType;

        // plugin specific variables

        // ROS API
        std::string _log_prefix;
        std::string _stm_topic;
        ros::Subscriber _sub;


    public:
        PeopleEntityPlugin(){}
        ~PeopleEntityPlugin();

    private:
        // DB API
        MetadataPtr make_metadata(const EntityType &entity);

        void callback(const EntityType &msg);

        void build_null(EntityType &entity);

    public:
        void initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name);
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end);

    };

};
#endif // LTM_SAMPLES_PEOPLE_ENTITY_PLUGIN_H_
