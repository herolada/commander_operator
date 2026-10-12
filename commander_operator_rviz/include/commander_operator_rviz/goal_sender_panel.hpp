#ifndef COMMANDER_OPERATOR_RVIZ__GOAL_SENDER_PANEL_HPP_
#define COMMANDER_OPERATOR_RVIZ__GOAL_SENDER_PANEL_HPP_

#include "commander_operator_interfaces/msg/operator_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace commander_operator_rviz
{

/**
 * Panel for requesting a stored waypoint or route from commander_operator.
 *
 * "Send" publishes a single OperatorRequest (goal name, route and wgs flags)
 * on the goal topic of the commander_operator node.
 */
class GoalSenderPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GoalSenderPanel(QWidget * parent = nullptr);
  ~GoalSenderPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onSend();

private:
  using OperatorRequest = commander_operator_interfaces::msg::OperatorRequest;

  void createPublisher();
  void updateWidgets();
  void setStatus(const QString & text, bool error = false);

  // Widgets
  QLineEdit * goal_edit_;
  QCheckBox * route_check_;
  QCheckBox * wgs_check_;
  QPushButton * send_button_;
  QLabel * status_label_;

  // ROS
  QString goal_topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<OperatorRequest>::SharedPtr goal_pub_;
};

}  // namespace commander_operator_rviz

#endif  // COMMANDER_OPERATOR_RVIZ__GOAL_SENDER_PANEL_HPP_
