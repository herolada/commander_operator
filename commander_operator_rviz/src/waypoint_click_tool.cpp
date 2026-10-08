#include "commander_operator_rviz/waypoint_click_tool.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace commander_operator_rviz
{

WaypointClickTool::WaypointClickTool()
: projection_finder_(std::make_shared<rviz_rendering::ViewportProjectionFinder>())
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", kDefaultClickTopic,
    "Topic the clicked points are published to (geometry_msgs/PointStamped).",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

void WaypointClickTool::onInitialize()
{
  updateTopic();
}

void WaypointClickTool::activate()
{
  setStatus("Left click on the ground plane to add a waypoint.");
}

void WaypointClickTool::deactivate() {}

void WaypointClickTool::updateTopic()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    topic_property_->getStdString(), rclcpp::QoS(10));
  clock_ = node->get_clock();
}

int WaypointClickTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  auto [hit, position] = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

  if (!hit) {
    setStatus("Point the cursor at the ground plane.");
    return Render;
  }

  const QString frame = context_->getFixedFrame();
  setStatus(
    QString("[%1, %2] in '%3'. Left click to add a waypoint.")
    .arg(position.x, 0, 'f', 2).arg(position.y, 0, 'f', 2).arg(frame));

  if (event.leftUp()) {
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = frame.toStdString();
    msg.header.stamp = clock_->now();
    msg.point.x = position.x;
    msg.point.y = position.y;
    msg.point.z = position.z;
    publisher_->publish(msg);
  }

  return Render;
}

}  // namespace commander_operator_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(commander_operator_rviz::WaypointClickTool, rviz_common::Tool)
