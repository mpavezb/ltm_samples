#include <pluginlib/class_list_macros.h>
#include <ltm/plugins_base.h>
#include <ltm_samples/emotion_plugin.h>
#include <ltm_samples/location_plugin.h>
#include <ltm_samples/image_stream_plugin.h>
#include <ltm_samples/people_entity_plugin.h>
#include <ltm_samples/objects_entity_plugin.h>
//#include <ltm_samples/robot_entity_plugin.h>
//#include <ltm_samples/location_entity_plugin.h>


PLUGINLIB_EXPORT_CLASS(ltm_samples::EmotionPlugin, ltm::plugin::EmotionBase)
PLUGINLIB_EXPORT_CLASS(ltm_samples::LocationPlugin, ltm::plugin::LocationBase)
PLUGINLIB_EXPORT_CLASS(ltm_samples::ImageStreamPlugin, ltm::plugin::StreamBase)
PLUGINLIB_EXPORT_CLASS(ltm_samples::PeopleEntityPlugin, ltm::plugin::EntityBase)
PLUGINLIB_EXPORT_CLASS(ltm_samples::ObjectsEntityPlugin, ltm::plugin::EntityBase)
//PLUGINLIB_EXPORT_CLASS(ltm_samples::RobotEntityPlugin, ltm::plugin::EntityBase)
//PLUGINLIB_EXPORT_CLASS(ltm_samples::LocationEntityPlugin, ltm::plugin::EntityBase)

