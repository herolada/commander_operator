#ifndef COMMANDER_OPERATOR_RVIZ__WAYPOINT_CLICK_TOOL_HPP_
#define COMMANDER_OPERATOR_RVIZ__WAYPOINT_CLICK_TOOL_HPP_

#include <memory>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

namespace rviz_common::properties
{
class StringProperty;
}

namespace commander_operator_rviz
{

/// Topic the tool publishes to and the WaypointRecorderPanel listens on by default.
constexpr char kDefaultClickTopic[] = "/waypoint_recorder/clicked_point";

/**
 * Publishes the point where the ground plane (z = 0 of the fixed frame) was clicked.
 *
 * Unlike the default "Publish Point" tool this works anywhere in the view, not only
 * on rendered geometry, and unlike "2D Goal Pose" it does not publish on a topic the
 * robot may act on. The tool stays active after a click, so routes can be clicked in
 * one go.
 */
class WaypointClickTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  WaypointClickTool();
  ~WaypointClickTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private Q_SLOTS:
  void updateTopic();

private:
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
  rviz_common::properties::StringProperty * topic_property_;
};

}  // namespace commander_operator_rviz

#endif  // COMMANDER_OPERATOR_RVIZ__WAYPOINT_CLICK_TOOL_HPP_
