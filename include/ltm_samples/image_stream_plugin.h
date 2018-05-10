#ifndef LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
#define LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugins_base.h>
#include <ltm_samples/ImageStream.h>
#include <sensor_msgs/Image.h>

#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/database_connection.h>

typedef warehouse_ros::MessageCollection<ltm_samples::ImageStream> ImageStreamCollection;
typedef boost::shared_ptr<EpisodeCollection> ImageStreamCollectionPtr;

typedef warehouse_ros::MessageWithMetadata<ltm::Episode> ImageStreamWithMetadata;
typedef boost::shared_ptr<const EpisodeWithMetadata> ImageStreamWithMetadataPtr;


namespace ltm_samples
{
    class ImageStreamPlugin : public ltm::plugin::StreamBase
    {
    public:
        ImageStreamPlugin(){}

        void initialize(const std::string& param_ns) {
            ROS_DEBUG_STREAM("LTM Image Stream plugin initialized with ns: " << param_ns);

        }

        void register_episode(uint32_t uid) {
            ROS_DEBUG_STREAM("LTM Image Stream plugin: registering episode " << uid);
        }

        void unregister_episode(uint32_t uid) {
            ROS_DEBUG_STREAM("LTM Image Stream plugin: unregistering episode " << uid);
        }

        void collect(uint32_t uid, ltm::What& msg) {
            ROS_DEBUG_STREAM("LTM Image Stream plugin: collecting episode " << uid);
        }

        void degrade(uint32_t uid) {
            // TODO
        }

        std::string get_type() {
            return "images";
        }

        std::string get_collection_name() {
            return "image_streams";
        }
    private:
        warehouse_ros_mongo::MongoDatabaseConnection _conn;
        ImageStreamCollectionPtr _coll;
        std::string log_prefix;
    };

};
#endif // LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
