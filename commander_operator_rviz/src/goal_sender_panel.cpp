#include "commander_operator_rviz/goal_sender_panel.hpp"

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "rviz_common/display_context.hpp"

namespace commander_operator_rviz
{

namespace
{

constexpr char kDefaultGoalTopic[] = "/commander_operator/goal";

}  // namespace

GoalSenderPanel::GoalSenderPanel(QWidget * parent)
: rviz_common::Panel(parent),
  goal_topic_(kDefaultGoalTopic)
{
  goal_edit_ = new QLineEdit;
  goal_edit_->setPlaceholderText("waypoint or route name");
  route_check_ = new QCheckBox("Route");
  route_check_->setToolTip("Request a route instead of a single waypoint.");
  wgs_check_ = new QCheckBox("WGS");
  wgs_check_->setToolTip("Look the goal up among the WGS waypoints / routes.");
  send_button_ = new QPushButton("Send");
  send_button_->setToolTip("Publish the request once (also Enter in the name field).");

  auto grid = new QGridLayout;
  grid->addWidget(new QLabel("Goal:"), 0, 0);
  grid->addWidget(goal_edit_, 0, 1, 1, 3);
  grid->addWidget(route_check_, 1, 1);
  grid->addWidget(wgs_check_, 1, 2);
  grid->addWidget(send_button_, 1, 3);
  grid->setColumnStretch(2, 1);

  status_label_ = new QLabel;
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  auto layout = new QVBoxLayout;
  layout->addLayout(grid);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(send_button_, &QPushButton::clicked, this, &GoalSenderPanel::onSend);
  connect(goal_edit_, &QLineEdit::returnPressed, this, &GoalSenderPanel::onSend);
  connect(goal_edit_, &QLineEdit::textChanged, this, &GoalSenderPanel::updateWidgets);

  updateWidgets();
}

GoalSenderPanel::~GoalSenderPanel()
{
  goal_pub_.reset();
}

void GoalSenderPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  createPublisher();
  updateWidgets();
}

void GoalSenderPanel::createPublisher()
{
  if (!node_) {
    return;
  }
  // Matches the subscription of commander_operator (reliable, volatile).
  goal_pub_ = node_->create_publisher<OperatorRequest>(
    goal_topic_.toStdString(), rclcpp::QoS(10).reliable());
}

void GoalSenderPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  bool flag;
  QString text;
  if (config.mapGetString("Goal", &text)) {
    goal_edit_->setText(text);
  }
  if (config.mapGetBool("Route", &flag)) {
    route_check_->setChecked(flag);
  }
  if (config.mapGetBool("WGS", &flag)) {
    wgs_check_->setChecked(flag);
  }
  if (config.mapGetString("GoalTopic", &text)) {
    goal_topic_ = text;
  }

  createPublisher();
  updateWidgets();
}

void GoalSenderPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  config.mapSetValue("Goal", goal_edit_->text());
  config.mapSetValue("Route", route_check_->isChecked());
  config.mapSetValue("WGS", wgs_check_->isChecked());
  config.mapSetValue("GoalTopic", goal_topic_);
}

void GoalSenderPanel::onSend()
{
  const QString goal = goal_edit_->text().trimmed();
  if (goal.isEmpty() || !goal_pub_) {
    return;
  }

  OperatorRequest msg;
  msg.goal = goal.toStdString();
  msg.route = route_check_->isChecked();
  msg.wgs = wgs_check_->isChecked();
  goal_pub_->publish(msg);

  QString text = QString("Sent %1 %2 '%3' on %4.")
    .arg(msg.wgs ? "WGS" : "local")
    .arg(msg.route ? "route" : "waypoint")
    .arg(goal)
    .arg(goal_topic_);
  const bool no_subscriber = goal_pub_->get_subscription_count() == 0;
  if (no_subscriber) {
    text += " No subscriber, is commander_operator running?";
  }
  setStatus(text, no_subscriber);
}

void GoalSenderPanel::updateWidgets()
{
  send_button_->setEnabled(goal_pub_ && !goal_edit_->text().trimmed().isEmpty());
}

void GoalSenderPanel::setStatus(const QString & text, bool error)
{
  status_label_->setStyleSheet(error ? "color: #d03030;" : "");
  status_label_->setText(text);
}

}  // namespace commander_operator_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(commander_operator_rviz::GoalSenderPanel, rviz_common::Panel)
