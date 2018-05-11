#ifndef LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
#define LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugins_base.h>
#include <ltm_samples/ImageStream.h>
#include <sensor_msgs/Image.h>

#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/database_connection.h>

typedef warehouse_ros::MessageCollection<ltm_samples::ImageStream> ImageStreamCollection;
typedef boost::shared_ptr<ImageStreamCollection> ImageStreamCollectionPtr;

typedef warehouse_ros::MessageWithMetadata<ltm_samples::ImageStream> ImageStreamWithMetadata;
typedef boost::shared_ptr<const ImageStreamWithMetadata> ImageStreamWithMetadataPtr;

namespace ltm_samples
{
    class ImageStreamPlugin : public ltm::plugin::StreamBase
    {
    // TODO: (un)subscribe on demand.

    private:
        // plugin
        std::string _log_prefix;
        std::string _type;
        std::string _collection_name;
        std::vector<sensor_msgs::ImageConstPtr> _buffer;
        std::vector<uint32_t> registry;
        size_t _last_idx;
        size_t _buffer_size;

        // timing
        int _buffer_max_size;
        ros::Duration _buffer_period;
        ros::Time _last_callback;

        // subscriber
        std::string _image_topic;
        ros::Subscriber _image_sub;

        // database
        warehouse_ros_mongo::MongoDatabaseConnection _conn;
        ImageStreamCollectionPtr _coll;


        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void subscribe();
        void unsubscribe();

    public:
        ImageStreamPlugin(){}

        void initialize(const std::string& param_ns);

        void register_episode(uint32_t uid);

        void unregister_episode(uint32_t uid);

        void collect(uint32_t uid, ltm::What& msg);

        void degrade(uint32_t uid);

        std::string get_type();

        std::string get_collection_name();
    };

};
#endif // LTM_SAMPLES_IMAGE_STREAM_PLUGIN_H_
