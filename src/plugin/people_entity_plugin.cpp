#include <ltm_samples/plugin/people_entity_plugin.h>
#include <ltm/parameter_server_wrapper.h>
#include <sensor_msgs/Image.h>

/**
 * TODO: EL PLUGIN ES EL ENCARGADO DE DEFINIR EL UID DE LA ENTIDAD!, NO EL SERVER.
 *
 */

namespace ltm_samples
{
    // =================================================================================================================
    // Public API
    // =================================================================================================================

    void PeopleEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name) {
        _log_prefix = "[LTM][People Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        build_null(_null_e);

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

    void PeopleEntityPlugin::drop_db() {
        this->_registry.clear();
        this->ltm_drop_db();
    }

    void PeopleEntityPlugin::append_status(std::stringstream &status) {
        status << this->ltm_get_status();
    }

    // =================================================================================================================
    // Private API
    // =================================================================================================================

    MetadataPtr PeopleEntityPlugin::make_metadata(const EntityType &entity) {
        MetadataPtr meta = this->ltm_create_metadata();
        meta->append("uid", (int) entity.uid);
        meta->append("log_uid", (int) entity.log_uid);
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

        double last_seen = entity.last_seen.sec + entity.last_seen.nsec * pow10(-9);
        meta->append("last_seen", last_seen);

        double last_interacted = entity.last_interacted.sec + entity.last_interacted.nsec * pow10(-9);
        meta->append("last_interacted", last_interacted);

        return meta;
    }

    MetadataPtr PeopleEntityPlugin::make_log_metadata(const LogType &log) {
        MetadataPtr meta = this->ltm_create_metadata();
        // KEYS
        meta->append("entity_uid", (int) log.entity_uid);
        meta->append("log_uid", (int) log.log_uid);

        // WHO
        // meta->append("episode_uids", log.episode_uids);

        // WHEN
        double timestamp = log.timestamp.sec + log.timestamp.nsec * pow10(-9);
        meta->append("timestamp", timestamp);
        meta->append("prev_log", (int) log.prev_log);
        meta->append("next_log", (int) log.next_log);

        // WHAT
        // meta->append("new_f", log.new_f);
        // meta->append("updated_f", log.updated_f);
        // meta->append("removed_f", log.removed_f);

        return meta;
    }

    void PeopleEntityPlugin::callback(const EntityType& msg) {
        // KEYS
        LogType log;
        log.entity_uid = msg.uid;
        log.log_uid = (uint32_t) this->ltm_reserve_log_uid();

        // WHEN
        log.timestamp = ros::Time::now();
        log.prev_log = (uint32_t) this->ltm_get_last_log_uid(log.entity_uid);
        // log.next_log = 0;

        // WHO
        this->ltm_get_registry(log.episode_uids);

        // TODO: WE CAN USE A CACHE FOR RECENT ENTITIES
        EntityType curr;
        EntityWithMetadataPtr curr_with_md;
        curr.uid = msg.uid;
        curr.log_uid = 0;
        bool uid_exists = this->ltm_get(msg.uid, curr_with_md);
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
            curr.last_seen = curr_with_md->last_seen;
            curr.last_interacted = curr_with_md->last_interacted;
        }

        EntityType diff;
        diff.uid = msg.uid;
        diff.log_uid = log.log_uid;
        this->update_field<std::string>(log, "name", curr.name, diff.name, msg.name, _null_e.name);
        this->update_field<std::string>(log, "last_name", curr.last_name, diff.last_name, msg.last_name, _null_e.last_name);
        this->update_field<uint8_t>(log, "age", curr.age, diff.age, msg.age, _null_e.age);
        this->update_field<uint8_t>(log, "genre", curr.genre, diff.genre, msg.genre, _null_e.genre);
        this->update_field<std::string>(log, "country", curr.country, diff.country, msg.country, _null_e.country);
        this->update_field<std::string>(log, "city", curr.city, diff.city, msg.city, _null_e.city);
        this->update_field<ltm::Date>(log, "birthday", curr.birthday, diff.birthday, msg.birthday, _null_e.birthday);
        this->update_field<sensor_msgs::Image>(log, "face", curr.face, diff.face, msg.face, _null_e.face);
        this->update_field<sensor_msgs::Image>(log, "body", curr.body, diff.body, msg.body, _null_e.body);
        this->update_field<std::string>(log, "emotion", curr.emotion, diff.emotion, msg.emotion, _null_e.emotion);
        this->update_field<std::string>(log, "stance", curr.stance, diff.stance, msg.stance, _null_e.stance);
        this->update_field<ros::Time>(log, "last_seen", curr.last_seen, diff.last_seen, msg.last_seen, _null_e.last_seen);
        this->update_field<ros::Time>(log, "last_interacted", curr.last_interacted, diff.last_interacted, msg.last_interacted, _null_e.last_interacted);

        size_t n_added = log.new_f.size();
        size_t n_updated = log.updated_f.size();
        size_t n_removed = log.removed_f.size();
        if ((n_added + n_updated + n_removed) == 0) {
            ROS_DEBUG_STREAM(_log_prefix << "Received update for entity (" << msg.uid << ") does not apply any changes.");
            return;
        }
        ROS_DEBUG_STREAM(_log_prefix << "Received update for entity (" << msg.uid << ") info: add=" << n_added << ", update=" << n_updated << ", removed=" << n_removed << ".");
        ROS_DEBUG_STREAM_COND(n_added > 0, _log_prefix << " - ADD fields for (" << msg.uid << "): " << build_log_vector(log.new_f) << ".");
        ROS_DEBUG_STREAM_COND(n_updated > 0, _log_prefix << " - UPDATE fields for (" << msg.uid << "): " << build_log_vector(log.updated_f) << ".");
        ROS_DEBUG_STREAM_COND(n_removed > 0, _log_prefix << " - REMOVE fields for (" << msg.uid << "): " << build_log_vector(log.removed_f) << ".");

        // SAVE LOG AND DIFF INTO COLLECTION
        this->ltm_log_insert(log, make_log_metadata(log));
        this->ltm_diff_insert(diff, make_metadata(diff));

        // UPDATE CURRENT ENTITY
        this->ltm_update(curr.uid, curr, make_metadata(curr));

        // ADD ENTITIES/LOG TO REGISTRY BY TIMESTAMP
        // TODO: MUTEX HERE?
        RegisterItem reg(log.timestamp, log.log_uid, log.entity_uid);
        this->_registry.insert(reg);
    }

    std::string PeopleEntityPlugin::build_log_vector(const std::vector<std::string> &v) {
        std::string s = "[";
        std::vector<std::string>::const_iterator it;
        for (it = v.begin(); it != v.end(); ++it) {
            s += *it + ", ";
        }
        s = (s.size() > 2) ? s = s.substr(0, s.size()-2) + "]" : "[]";
        return s;
    }

    template <typename T>
    bool PeopleEntityPlugin::field_equals(const T &A, const T &B) {
        return A == B;
    }

    //explicit specialization for ltm::Date
    template <>
    bool PeopleEntityPlugin::field_equals<ltm::Date>(const ltm::Date &A, const ltm::Date &B) {
        return A.day == B.day && A.month == B.month && A.year == B.year;
    }

    //explicit specialization for ros::Time
    template <>
    bool PeopleEntityPlugin::field_equals<ros::Time>(const ros::Time &A, const ros::Time &B) {
        return A.sec == B.sec && A.nsec == B.nsec;
    }

    //explicit specialization for sensor_msgs::Image
    template <>
    bool PeopleEntityPlugin::field_equals<sensor_msgs::Image>(const sensor_msgs::Image &A, const sensor_msgs::Image &B) {
        return A.header.stamp.sec == B.header.stamp.sec
               && A.header.stamp.nsec == B.header.stamp.nsec
               && A.header.frame_id == B.header.frame_id
               && A.header.seq == B.header.seq
               && A.height == B.height
               && A.width == B.width
               && A.step == B.step;
    }

    template <typename T>
    void PeopleEntityPlugin::update_field(ltm_samples::PeopleEntityPlugin::LogType &log, const std::string &field, T &curr_e,
                                          T &log_e, const T &new_e, const T &null_e) {
        if (field_equals<T>(new_e, null_e)) {            // new field is null
            if (field_equals<T>(curr_e, null_e)) {       // - and curr field is null     ---> NO CHANGES
                log_e = null_e;
            } else {                                     // - and curr field is not null ---> REMOVE FIELD
                curr_e = null_e;
                log_e = new_e;
                log.removed_f.push_back(field);
            }
        } else {                                         // new field is not null
            if (field_equals<T>(curr_e, null_e)) {       // - and curr field is null     ---> NEW FIELD
                curr_e = new_e;
                log_e = new_e;
                log.new_f.push_back(field);
            } else if (field_equals<T>(curr_e, new_e)) { // - and curr field is the same ---> NO CHANGES
                log_e = null_e;
            } else {                                     // - and fields are different   ---> UPDATE FIELD
                curr_e = new_e;
                log_e = new_e;
                log.updated_f.push_back(field);
            }
        }
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