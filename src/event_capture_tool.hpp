#ifndef EVENT_CAPTURE_TOOL_HPP
#define EVENT_CAPTURE_TOOL_HPP

#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/default_plugin/tools/move_tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <memory>


namespace rviz_plugins {

class EventCapture: public rviz::Tool
{
    Q_OBJECT

    public:

        EventCapture();
        ~EventCapture();

        void onInitialize() override;
        void activate() override;
        void deactivate() override;
        int processMouseEvent(rviz::ViewportMouseEvent& event) override;
        int processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel );

    private Q_SLOTS:

        void updateTopic();

    private:

        rviz::MoveTool move_tool_;
        ros::NodeHandle nh_;
        ros::Publisher  pub_mouse_, pub_key_;
        std::unique_ptr<rviz::StringProperty> mouse_topic_property_;
        std::unique_ptr<rviz::StringProperty> key_topic_property_;
        std::unique_ptr<rviz::BoolProperty> use_move_tool_property_;

        void publishMouseEvent(rviz::ViewportMouseEvent &event);
        void publishKeyEvent(QKeyEvent* event);
};

}

#endif
