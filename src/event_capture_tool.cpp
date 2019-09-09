
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <rviz/display_context.h>
#include <rviz/render_panel.h>

#include "event_capture_tool.hpp"
#include "event_capture/MouseEventCaptureStamped.h"

namespace rviz_plugins {

EventCapture::EventCapture()
{
    shortcut_key_ = 'c';
    topic_property_.reset(new rviz::StringProperty( "Topic", "/rviz/event_capture/mouse",
                                        "The topic on which to publish event capture",
                                        getPropertyContainer(), SLOT( updateTopic() ), this ));
}

EventCapture::~EventCapture()
{
}

void EventCapture::onInitialize()
{
    updateTopic();
}

void EventCapture::activate()
{
    //printf("activate\n");
}

void EventCapture::deactivate()
{
    //printf("deactivate\n");
}

void EventCapture::updateTopic()
{
  pub_ = nh_.advertise<event_capture::MouseEventCaptureStamped>( topic_property_->getStdString(), 1 );
}

int EventCapture::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    publishMouseEvent(event);
    return Render;
}

/*int EventCapture::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  return Render;
}*/

void EventCapture::publishMouseEvent(rviz::ViewportMouseEvent &event)
{
  const auto viewport_x = event.x / static_cast<double>(event.viewport->getActualWidth());
  const auto viewport_y = event.y / static_cast<double>(event.viewport->getActualHeight());
  const auto camera = event.viewport->getCamera();
  const auto campos = camera->getPosition();
  const auto camvec = camera->getDirection();
  const auto ray = camera->getCameraToViewportRay(viewport_x, viewport_y);
  const auto raypos = ray.getOrigin();
  const auto rayvec = ray.getDirection();

  auto OgreVector3ToGeometryVector3 = [](const Ogre::Vector3 &ov){
    geometry_msgs::Vector3 gv;
    gv.x = ov.x;
    gv.y = ov.y;
    gv.z = ov.z;
    return gv;
  };

  event_capture::MouseEventCaptureStamped mecs;
  event_capture::MouseEventCapture mec;
  mec.ray = [raypos, rayvec, OgreVector3ToGeometryVector3]{
    event_capture::Ray r;
    r.origin = OgreVector3ToGeometryVector3(raypos);
    r.direction = OgreVector3ToGeometryVector3(rayvec);
    return r;
  }();

  mec.camera = [campos, camvec, OgreVector3ToGeometryVector3]{
    event_capture::Camera c;
    c.origin = OgreVector3ToGeometryVector3(campos);
    c.direction = OgreVector3ToGeometryVector3(camvec);
    return c;
  }();

  mec.left = event.left();
  mec.left_down = event.leftDown();
  mec.left_up = event.leftUp();
  mec.middle = event.middle();
  mec.middle_down = event.middleDown();
  mec.middle_up = event.middleUp();
  mec.right = event.right();
  mec.right_down = event.rightDown();
  mec.right_up = event.rightUp();
  mec.alt = event.alt();
  mec.ctrl = event.control();
  mec.shift = event.shift();

  mecs.capture = mec;
  mecs.header.stamp = ros::Time::now();
  mecs.header.frame_id = context_->getFixedFrame().toUtf8().constData();

  pub_.publish(mecs);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::EventCapture, rviz::Tool)
