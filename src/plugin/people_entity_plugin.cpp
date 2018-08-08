#include <ltm_samples/plugin/people_entity_plugin.h>
#include <ltm/util/parameter_server_wrapper.h>
#include <sensor_msgs/Image.h>

/**
 * TODO: EL PLUGIN ES EL ENCARGADO DE DEFINIR EL UID DE LA ENTIDAD!, NO EL SERVER.
 *
 */

namespace ltm_samples
{
    using namespace ltm::plugin;

    // =================================================================================================================
    // Public API
    // =================================================================================================================

    std::string PeopleEntityPlugin::get_type() {
        return this->ltm_get_type();
    }

    void PeopleEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name) {
        _log_prefix = "[LTM][People Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        build_null(_null_e);

        ltm::util::ParameterServerWrapper psw("~");
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
        // TODO: WE CAN USE THIS TO DELETE OLD TIMESTAMPS FROM THE REGISTRY
    }

    void PeopleEntityPlugin::unregister_episode(uint32_t uid) {
        this->ltm_unregister_episode(uid);
        // TODO: WE CAN USE THIS TO DELETE OLD TIMESTAMPS FROM THE REGISTRY
    }

    void PeopleEntityPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) {
        ROS_WARN_STREAM(_log_prefix << "Collecting people entities for episode " << uid << ".");

        // TODO: mutex (it is not required, because we are using a single-threaded Spinner for callbacks)

        // write entities matching initial and ending times.
        std::map<uint32_t, ltm::EntityRegister> episode_registry;
        std::map<uint32_t, ltm::EntityRegister>::iterator m_it;

        // COMPUTE EPISODE REGISTRY
        int logcnt = 0;
        Registry::const_iterator it;
        for (it = _registry.lower_bound(RegisterItem(_start, 0, 0)); it != _registry.end(); ++it) {
            if (it->timestamp > _end) break;

            // entity is in registry
            m_it = episode_registry.find(it->entity_uid);
            if (m_it != episode_registry.end()) {
                // update register
                m_it->second.log_uids.push_back(it->log_uid);
            } else {
                // new register
                ltm::EntityRegister reg;
                reg.type = ltm_get_type();
                reg.uid = it->entity_uid;
                reg.log_uids.push_back(it->log_uid);
                episode_registry.insert(std::pair<uint32_t, ltm::EntityRegister>(it->entity_uid, reg));
            }
            logcnt++;
        }

        // SAVE IT
        ROS_WARN_STREAM(_log_prefix << "Collected (" << episode_registry.size() << ") people entities and (" << logcnt << ") logs for episode " << uid << ".");
        for (m_it = episode_registry.begin(); m_it != episode_registry.end(); ++m_it) {
            msg.entities.push_back(m_it->second);
        }

        // unregister
        // TODO: redundant calls to (un)register methods?
        unregister_episode(uid);
    }

    void PeopleEntityPlugin::query(const std::string &json, ltm::QueryServer::Response &res, bool trail) {
        this->ltm_query(json, res, trail);
    }

    void PeopleEntityPlugin::drop_db() {
        this->reset(this->ltm_get_db_name());
        this->ltm_drop_db();
    }

    void PeopleEntityPlugin::reset(const std::string &db_name) {
        this->_registry.clear();
        this->ltm_resetup_db(db_name);
    }

    void PeopleEntityPlugin::append_status(std::stringstream &status) {
        status << this->ltm_get_status();
    }

    MetadataPtr PeopleEntityPlugin::make_metadata(const EntityMsg &entity) {
        MetadataPtr meta = this->ltm_create_metadata(entity);
        meta->append("name", entity.name);
        meta->append("last_name", entity.last_name);
        meta->append("age", entity.age);
        meta->append("genre", entity.genre);
        meta->append("country", entity.country);
        meta->append("city", entity.city);
        std::stringstream birthday;
        birthday << entity.birthday.year << "/" << entity.birthday.month << "/" << entity.birthday.day;
        meta->append("birthday", birthday.str());
        meta->append("emotion", entity.emotion);
        meta->append("stance", entity.stance);
        meta->append("is_nerd", entity.is_nerd);

        double last_seen = entity.last_seen.sec + entity.last_seen.nsec * pow10(-9);
        meta->append("last_seen", last_seen);

        double last_interacted = entity.last_interacted.sec + entity.last_interacted.nsec * pow10(-9);
        meta->append("last_interacted", last_interacted);

        return meta;
    }

    // =================================================================================================================
    // Private API
    // =================================================================================================================

    void PeopleEntityPlugin::callback(const EntityMsg &msg) {
        this->update(msg);
    }

    void PeopleEntityPlugin::update(const EntityMsg& msg) {
        // KEYS
        LogType log;
        log.entity_uid = msg.meta.uid;
        log.log_uid = (uint32_t) this->ltm_reserve_log_uid();

        // WHEN
        log.timestamp = ros::Time::now();
        log.prev_log = (uint32_t) this->ltm_get_last_log_uid(log.entity_uid);
        // log.next_log = 0;

        // WHO
        this->ltm_get_registry(log.episode_uids);

        // TODO: WE CAN USE A CACHE FOR RECENT ENTITIES
        EntityMsg curr;
        EntityWithMetadataPtr curr_with_md;
        curr.meta.uid = msg.meta.uid;
        curr.meta.log_uid = log.log_uid;
        bool uid_exists = this->ltm_get(msg.meta.uid, curr_with_md);
        if (uid_exists) {
            curr.name = curr_with_md->name;
            curr.last_name = curr_with_md->last_name;
            curr.age = curr_with_md->age;
            curr.genre = curr_with_md->genre;
            curr.country = curr_with_md->country;
            curr.city = curr_with_md->city;
            curr.birthday = curr_with_md->birthday;
            curr.face = curr_with_md->face;
            curr.body = curr_with_md->body;
            curr.emotion = curr_with_md->emotion;
            curr.stance = curr_with_md->stance;
            curr.is_nerd = curr_with_md->is_nerd;
            curr.last_seen = curr_with_md->last_seen;
            curr.last_interacted = curr_with_md->last_interacted;
        }

        EntityMsg diff;
        diff.meta.uid = msg.meta.uid;
        diff.meta.log_uid = log.log_uid;
        entity::update_field<std::string>(log, "name", curr.name, diff.name, msg.name, _null_e.name);
        entity::update_field<std::string>(log, "last_name", curr.last_name, diff.last_name, msg.last_name, _null_e.last_name);
        entity::update_field<uint8_t>(log, "age", curr.age, diff.age, msg.age, _null_e.age);
        entity::update_field<uint8_t>(log, "genre", curr.genre, diff.genre, msg.genre, _null_e.genre);
        entity::update_field<std::string>(log, "country", curr.country, diff.country, msg.country, _null_e.country);
        entity::update_field<std::string>(log, "city", curr.city, diff.city, msg.city, _null_e.city);
        entity::update_field<ltm::Date>(log, "birthday", curr.birthday, diff.birthday, msg.birthday, _null_e.birthday);
        entity::update_field<sensor_msgs::Image>(log, "face", curr.face, diff.face, msg.face, _null_e.face);
        entity::update_field<sensor_msgs::Image>(log, "body", curr.body, diff.body, msg.body, _null_e.body);
        entity::update_field<std::string>(log, "emotion", curr.emotion, diff.emotion, msg.emotion, _null_e.emotion);
        entity::update_field<std::string>(log, "stance", curr.stance, diff.stance, msg.stance, _null_e.stance);
        entity::update_field<uint8_t>(log, "is_nerd", curr.is_nerd, diff.is_nerd, msg.is_nerd, _null_e.is_nerd);
        entity::update_field<ros::Time>(log, "last_seen", curr.last_seen, diff.last_seen, msg.last_seen, _null_e.last_seen);
        entity::update_field<ros::Time>(log, "last_interacted", curr.last_interacted, diff.last_interacted, msg.last_interacted, _null_e.last_interacted);

        size_t n_added = log.new_f.size();
        size_t n_updated = log.updated_f.size();
        size_t n_removed = log.removed_f.size();
        if ((n_added + n_updated + n_removed) == 0) {
            ROS_DEBUG_STREAM(_log_prefix << "Received update for entity (" << msg.meta.uid << ") does not apply any changes.");
            return;
        }
        ROS_DEBUG_STREAM(_log_prefix << "Received update for entity (" << msg.meta.uid << ") info: add=" << n_added << ", update=" << n_updated << ", removed=" << n_removed << ".");
        ROS_DEBUG_STREAM_COND(n_added > 0, _log_prefix << " - ADD fields for (" << msg.meta.uid << "): " << entity::build_log_vector(log.new_f) << ".");
        ROS_DEBUG_STREAM_COND(n_updated > 0, _log_prefix << " - UPDATE fields for (" << msg.meta.uid << "): " << entity::build_log_vector(log.updated_f) << ".");
        ROS_DEBUG_STREAM_COND(n_removed > 0, _log_prefix << " - REMOVE fields for (" << msg.meta.uid << "): " << entity::build_log_vector(log.removed_f) << ".");

        // SAVE LOG AND DIFF INTO COLLECTION
        this->ltm_log_insert(log);
        this->ltm_diff_insert(diff);

        // UPDATE CURRENT ENTITY
        this->ltm_update(curr.meta.uid, curr);

        // ADD ENTITIES/LOG TO REGISTRY BY TIMESTAMP
        // TODO: MUTEX HERE?
        RegisterItem reg(log.timestamp, log.log_uid, log.entity_uid);
        this->_registry.insert(reg);
    }

    void PeopleEntityPlugin::build_null(EntityMsg &entity) {
        entity.meta.uid = 0;
        entity.meta.log_uid = 0;
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
        entity.is_nerd = 0;
        entity.last_seen = ros::Time(0);
        entity.last_interacted = ros::Time(0);
    }

}