#ifndef LTM_SAMPLES__LTM_PLUGINS_H_
#define LTM_SAMPLES__LTM_PLUGINS_H_
#include <ltm/plugins_base.h>
#include <ros/ros.h>


namespace ltm_samples
{
    class EmotionPlugin : public ltm::plugin::EmotionBase
    {
    public:
        EmotionPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Emotion plugin initialized.");
        }

        bool get_emotion() {
            return true;
        }

    private:
        double side_length_;
    };


    class LocationPlugin : public ltm::plugin::LocationBase
    {
    public:
        LocationPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Location plugin initialized.");
        }

        bool get_location() {
            return true;
        }

    private:
        double side_length_;
    };


    class ImageStreamPlugin : public ltm::plugin::StreamBase
    {
    public:
        ImageStreamPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Image Stream plugin initialized.");
        }

        bool get_stream() {
            return true;
        }

    private:
        double side_length_;
    };


    class PeopleEntityPlugin : public ltm::plugin::EntityBase
    {
    public:
        PeopleEntityPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM People Entities plugin initialized.");
        }

        bool get_entity() {
            return true;
        }

    private:
        double side_length_;
    };


    class ObjectsEntityPlugin : public ltm::plugin::EntityBase
    {
    public:
        ObjectsEntityPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Objects Entities plugin initialized.");
        }

        bool get_entity() {
            return true;
        }

    private:
        double side_length_;
    };

    class RobotEntityPlugin : public ltm::plugin::EntityBase
    {
    public:
        RobotEntityPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Robot Entities plugin initialized.");
        }

        bool get_entity() {
            return true;
        }

    private:
        double side_length_;
    };


    class LocationEntityPlugin : public ltm::plugin::EntityBase
    {
    public:
        LocationEntityPlugin(){}

        void initialize(double side_length) {
            side_length_ = side_length;
            ROS_INFO("LTM Location Entities plugin initialized.");
        }

        bool get_entity() {
            return true;
        }

    private:
        double side_length_;
    };
};
#endif // LTM_SAMPLES__LTM_PLUGINS_H_
