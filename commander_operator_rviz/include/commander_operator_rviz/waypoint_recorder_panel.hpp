#ifndef COMMANDER_OPERATOR_RVIZ__WAYPOINT_RECORDER_PANEL_HPP_
#define COMMANDER_OPERATOR_RVIZ__WAYPOINT_RECORDER_PANEL_HPP_

#include <string>
#include <vector>

#include "commander_operator_interfaces/srv/save_waypoints.hpp"
#include "crl_commander_interfaces/msg/operator_goal.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QRadioButton;

namespace commander_operator_rviz
{

/**
 * Panel for recording commander_operator waypoints and routes.
 *
 * While "Record" is on, points clicked with the WaypointClick tool are staged
 * (a single one in waypoint mode, a sequence in route mode) and shown as markers.
 * "Save" sends them to the waypoint_recorder node, which transforms them and
 * writes them into the commander_operator data files.
 */
class WaypointRecorderPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointRecorderPanel(QWidget * parent = nullptr);
  ~WaypointRecorderPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onModeChanged();
  void onRecordToggled(bool checked);
  void onUndo();
  void onClear();
  void onSave();

private:
  using SaveWaypoints = commander_operator_interfaces::srv::SaveWaypoints;
  using OperatorGoal = crl_commander_interfaces::msg::OperatorGoal;

  struct PendingPoint
  {
    std::string name;  // empty in waypoint mode, the name is read on save
    OperatorGoal goal;
  };

  bool routeMode() const;
  void createConnections();
  void handleClick(const geometry_msgs::msg::PointStamped & msg);
  void handleResponse(const SaveWaypoints::Response & response, int request_id);
  OperatorGoal goalFromFields() const;
  void activateClickTool();
  void updateWidgets();
  void publishMarkers();
  void setStatus(const QString & text, bool error = false);

  // Widgets
  QRadioButton * waypoint_mode_button_;
  QRadioButton * route_mode_button_;
  QRadioButton * local_frame_button_;
  QRadioButton * wgs_frame_button_;
  QLineEdit * route_name_edit_;
  QLineEdit * point_name_edit_;
  QCheckBox * overwrite_check_;
  QCheckBox * max_linear_vel_check_;
  QDoubleSpinBox * max_linear_vel_spin_;
  QCheckBox * max_angular_vel_check_;
  QDoubleSpinBox * max_angular_vel_spin_;
  QCheckBox * lookahead_distance_check_;
  QDoubleSpinBox * lookahead_distance_spin_;
  QCheckBox * mandatory_check_;
  QPushButton * record_button_;
  QPushButton * undo_button_;
  QPushButton * clear_button_;
  QPushButton * save_button_;
  QListWidget * pending_list_;
  QLabel * status_label_;

  // ROS
  QString click_topic_;
  QString save_service_;
  QString marker_topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Client<SaveWaypoints>::SharedPtr save_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::vector<PendingPoint> pending_;
  int request_id_ = 0;
  bool saving_ = false;
};

}  // namespace commander_operator_rviz

#endif  // COMMANDER_OPERATOR_RVIZ__WAYPOINT_RECORDER_PANEL_HPP_
