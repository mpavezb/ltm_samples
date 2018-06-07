#include <pluginlib/class_list_macros.h>

#include <ltm_samples/plugin/emotion_plugin.h>
#include <ltm_samples/plugin/location_plugin.h>
#include <ltm_samples/plugin/people_entity_plugin.h>
//#include <ltm_samples/plugin/objects_entity_plugin.h>


PLUGINLIB_EXPORT_CLASS(ltm_samples::EmotionPlugin, ltm::plugin::EmotionBase)
PLUGINLIB_EXPORT_CLASS(ltm_samples::LocationPlugin, ltm::plugin::LocationBase)
PLUGINLIB_EXPORT_CLASS(ltm_samples::PeopleEntityPlugin, ltm::plugin::EntityBase)
//PLUGINLIB_EXPORT_CLASS(ltm_samples::ObjectsEntityPlugin, ltm::plugin::EntityBase)

