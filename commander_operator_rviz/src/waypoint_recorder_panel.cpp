#include "commander_operator_rviz/waypoint_recorder_panel.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPointer>
#include <QPushButton>
#include <QRadioButton>
#include <QRegularExpressionValidator>
#include <QTimer>
#include <QVBoxLayout>

#include <memory>
#include <tuple>
#include <utility>

#include "commander_operator_rviz/waypoint_click_tool.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"

namespace commander_operator_rviz
{

namespace
{

constexpr char kClickToolClass[] = "commander_operator_rviz/WaypointClick";
constexpr char kDefaultSaveService[] = "/waypoint_recorder/save_waypoints";
constexpr char kDefaultMarkerTopic[] = "/waypoint_recorder/pending_markers";
constexpr int kSaveTimeoutMs = 5000;

// Same rule as commander_operator.waypoints.is_valid_name (which also rejects
// YAML keywords such as 'true' or 'null').
const QRegularExpression kNameRegex("[A-Za-z_][A-Za-z0-9_\\-]*");

QDoubleSpinBox * makeSpinBox(double max, double value)
{
  auto spin = new QDoubleSpinBox;
  spin->setRange(0.0, max);
  spin->setDecimals(2);
  spin->setSingleStep(0.1);
  spin->setValue(value);
  return spin;
}

}  // namespace

WaypointRecorderPanel::WaypointRecorderPanel(QWidget * parent)
: rviz_common::Panel(parent),
  click_topic_(kDefaultClickTopic),
  save_service_(kDefaultSaveService),
  marker_topic_(kDefaultMarkerTopic)
{
  // Mode and frame
  waypoint_mode_button_ = new QRadioButton("Waypoint");
  route_mode_button_ = new QRadioButton("Route");
  waypoint_mode_button_->setChecked(true);
  auto mode_group = new QButtonGroup(this);
  mode_group->addButton(waypoint_mode_button_);
  mode_group->addButton(route_mode_button_);

  local_frame_button_ = new QRadioButton("Local");
  wgs_frame_button_ = new QRadioButton("WGS");
  local_frame_button_->setChecked(true);
  local_frame_button_->setToolTip("Save x, y in the map frame of the waypoint_recorder node.");
  wgs_frame_button_->setToolTip("Save lat, lon (via the ECEF frame of the waypoint_recorder node).");
  auto frame_group = new QButtonGroup(this);
  frame_group->addButton(local_frame_button_);
  frame_group->addButton(wgs_frame_button_);

  // Names
  auto validator = new QRegularExpressionValidator(kNameRegex, this);
  route_name_edit_ = new QLineEdit;
  route_name_edit_->setValidator(validator);
  route_name_edit_->setPlaceholderText("required");
  point_name_edit_ = new QLineEdit;
  point_name_edit_->setValidator(validator);
  overwrite_check_ = new QCheckBox("Overwrite existing");

  auto top_layout = new QGridLayout;
  top_layout->addWidget(new QLabel("Mode:"), 0, 0);
  top_layout->addWidget(waypoint_mode_button_, 0, 1);
  top_layout->addWidget(route_mode_button_, 0, 2);
  top_layout->addWidget(new QLabel("Frame:"), 1, 0);
  top_layout->addWidget(local_frame_button_, 1, 1);
  top_layout->addWidget(wgs_frame_button_, 1, 2);
  top_layout->addWidget(new QLabel("Route name:"), 2, 0);
  top_layout->addWidget(route_name_edit_, 2, 1, 1, 2);
  top_layout->addWidget(new QLabel("Point name:"), 3, 0);
  top_layout->addWidget(point_name_edit_, 3, 1, 1, 2);
  top_layout->addWidget(overwrite_check_, 4, 1, 1, 2);
  top_layout->setColumnStretch(2, 1);

  // Waypoint parameters
  max_linear_vel_check_ = new QCheckBox("max_linear_vel");
  max_linear_vel_spin_ = makeSpinBox(10.0, 1.2);
  max_angular_vel_check_ = new QCheckBox("max_angular_vel");
  max_angular_vel_spin_ = makeSpinBox(10.0, 1.2);
  lookahead_distance_check_ = new QCheckBox("lookahead_distance");
  lookahead_distance_spin_ = makeSpinBox(50.0, 1.5);
  mandatory_check_ = new QCheckBox("mandatory");
  mandatory_check_->setChecked(true);
  mandatory_check_->setToolTip("Whether the robot must reach this route waypoint.");

  auto params_box = new QGroupBox("Waypoint parameters");
  auto params_layout = new QGridLayout(params_box);
  params_layout->addWidget(max_linear_vel_check_, 0, 0);
  params_layout->addWidget(max_linear_vel_spin_, 0, 1);
  params_layout->addWidget(max_angular_vel_check_, 1, 0);
  params_layout->addWidget(max_angular_vel_spin_, 1, 1);
  params_layout->addWidget(lookahead_distance_check_, 2, 0);
  params_layout->addWidget(lookahead_distance_spin_, 2, 1);
  params_layout->addWidget(mandatory_check_, 3, 0);

  // Buttons
  record_button_ = new QPushButton("Record");
  record_button_->setCheckable(true);
  record_button_->setToolTip(
    "Accept points clicked with the WaypointClick tool (shortcut 'w').");
  undo_button_ = new QPushButton("Undo last");
  clear_button_ = new QPushButton("Clear");
  save_button_ = new QPushButton("Save");
  auto buttons_layout = new QHBoxLayout;
  buttons_layout->addWidget(record_button_);
  buttons_layout->addWidget(undo_button_);
  buttons_layout->addWidget(clear_button_);
  buttons_layout->addWidget(save_button_);

  pending_list_ = new QListWidget;
  pending_list_->setMaximumHeight(120);
  status_label_ = new QLabel;
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  auto layout = new QVBoxLayout;
  layout->addLayout(top_layout);
  layout->addWidget(params_box);
  layout->addLayout(buttons_layout);
  layout->addWidget(pending_list_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(route_mode_button_, &QRadioButton::toggled, this, &WaypointRecorderPanel::onModeChanged);
  connect(record_button_, &QPushButton::toggled, this, &WaypointRecorderPanel::onRecordToggled);
  connect(undo_button_, &QPushButton::clicked, this, &WaypointRecorderPanel::onUndo);
  connect(clear_button_, &QPushButton::clicked, this, &WaypointRecorderPanel::onClear);
  connect(save_button_, &QPushButton::clicked, this, &WaypointRecorderPanel::onSave);
  // In waypoint mode the name is shown on the pending point, keep it up to date.
  connect(point_name_edit_, &QLineEdit::textChanged, this, [this]() {
      if (!routeMode()) {
        updateWidgets();
        publishMarkers();
      }
    });
  for (auto [check, spin] : {
      std::pair{max_linear_vel_check_, max_linear_vel_spin_},
      std::pair{max_angular_vel_check_, max_angular_vel_spin_},
      std::pair{lookahead_distance_check_, lookahead_distance_spin_}})
  {
    spin->setEnabled(false);
    connect(check, &QCheckBox::toggled, spin, &QDoubleSpinBox::setEnabled);
  }

  updateWidgets();
}

WaypointRecorderPanel::~WaypointRecorderPanel()
{
  // Drop the ROS entities first, so that no callback can reach a half destroyed panel.
  click_sub_.reset();
  save_client_.reset();
  marker_pub_.reset();
}

void WaypointRecorderPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  createConnections();
}

void WaypointRecorderPanel::createConnections()
{
  if (!node_) {
    return;
  }

  QPointer<WaypointRecorderPanel> self(this);

  // Callbacks run in the thread spinning the rviz node, Qt widgets have to be
  // touched from the GUI thread, hence the queued invocations.
  click_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
    click_topic_.toStdString(), rclcpp::QoS(10),
    [self](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
      if (self) {
        QMetaObject::invokeMethod(
          self.data(), [self, msg]() {
            if (self) {self->handleClick(*msg);}
          }, Qt::QueuedConnection);
      }
    });

  save_client_ = node_->create_client<SaveWaypoints>(save_service_.toStdString());

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_.toStdString(), rclcpp::QoS(1).transient_local());
}

void WaypointRecorderPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  bool flag;
  float value;
  QString text;
  if (config.mapGetBool("RouteMode", &flag)) {
    route_mode_button_->setChecked(flag);
    waypoint_mode_button_->setChecked(!flag);
  }
  if (config.mapGetBool("WGS", &flag)) {
    wgs_frame_button_->setChecked(flag);
    local_frame_button_->setChecked(!flag);
  }
  for (auto [key, check, spin] : {
      std::tuple{"MaxLinearVel", max_linear_vel_check_, max_linear_vel_spin_},
      std::tuple{"MaxAngularVel", max_angular_vel_check_, max_angular_vel_spin_},
      std::tuple{"LookaheadDistance", lookahead_distance_check_, lookahead_distance_spin_}})
  {
    if (config.mapGetBool(QString("Use") + key, &flag)) {
      check->setChecked(flag);
    }
    if (config.mapGetFloat(key, &value)) {
      spin->setValue(value);
    }
  }
  if (config.mapGetBool("Mandatory", &flag)) {
    mandatory_check_->setChecked(flag);
  }
  if (config.mapGetString("ClickTopic", &text)) {
    click_topic_ = text;
  }
  if (config.mapGetString("SaveService", &text)) {
    save_service_ = text;
  }
  if (config.mapGetString("MarkerTopic", &text)) {
    marker_topic_ = text;
  }

  createConnections();
  updateWidgets();
}

void WaypointRecorderPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  config.mapSetValue("RouteMode", routeMode());
  config.mapSetValue("WGS", wgs_frame_button_->isChecked());
  config.mapSetValue("UseMaxLinearVel", max_linear_vel_check_->isChecked());
  config.mapSetValue("MaxLinearVel", max_linear_vel_spin_->value());
  config.mapSetValue("UseMaxAngularVel", max_angular_vel_check_->isChecked());
  config.mapSetValue("MaxAngularVel", max_angular_vel_spin_->value());
  config.mapSetValue("UseLookaheadDistance", lookahead_distance_check_->isChecked());
  config.mapSetValue("LookaheadDistance", lookahead_distance_spin_->value());
  config.mapSetValue("Mandatory", mandatory_check_->isChecked());
  config.mapSetValue("ClickTopic", click_topic_);
  config.mapSetValue("SaveService", save_service_);
  config.mapSetValue("MarkerTopic", marker_topic_);
}

bool WaypointRecorderPanel::routeMode() const
{
  return route_mode_button_->isChecked();
}

void WaypointRecorderPanel::onModeChanged()
{
  if (!pending_.empty()) {
    pending_.clear();
    setStatus("Mode changed, pending points cleared.");
  }
  updateWidgets();
  publishMarkers();
}

void WaypointRecorderPanel::onRecordToggled(bool checked)
{
  record_button_->setText(checked ? "Recording..." : "Record");
  if (checked) {
    activateClickTool();
    return;
  }

  // Give the mouse back to the default tool when the click tool is active.
  auto tool_manager = getDisplayContext()->getToolManager();
  auto current = tool_manager->getCurrentTool();
  if (current && current->getClassId() == kClickToolClass) {
    tool_manager->setCurrentTool(tool_manager->getDefaultTool());
  }
}

void WaypointRecorderPanel::activateClickTool()
{
  auto tool_manager = getDisplayContext()->getToolManager();
  rviz_common::Tool * tool = nullptr;
  for (int i = 0; i < tool_manager->numTools(); ++i) {
    if (tool_manager->getTool(i)->getClassId() == kClickToolClass) {
      tool = tool_manager->getTool(i);
      break;
    }
  }
  if (!tool) {
    tool = tool_manager->addTool(kClickToolClass);
  }
  if (tool) {
    tool_manager->setCurrentTool(tool);
    setStatus("Click points on the ground plane.");
  } else {
    setStatus("Could not load the WaypointClick tool.", true);
  }
}

void WaypointRecorderPanel::onUndo()
{
  if (!pending_.empty()) {
    pending_.pop_back();
  }
  updateWidgets();
  publishMarkers();
}

void WaypointRecorderPanel::onClear()
{
  pending_.clear();
  updateWidgets();
  publishMarkers();
}

WaypointRecorderPanel::OperatorGoal WaypointRecorderPanel::goalFromFields() const
{
  OperatorGoal goal;
  goal.use_max_linear_vel = max_linear_vel_check_->isChecked();
  goal.max_linear_vel = goal.use_max_linear_vel ? max_linear_vel_spin_->value() : -1.0;
  goal.use_max_angular_vel = max_angular_vel_check_->isChecked();
  goal.max_angular_vel = goal.use_max_angular_vel ? max_angular_vel_spin_->value() : -1.0;
  goal.use_lookahead_distance = lookahead_distance_check_->isChecked();
  goal.lookahead_distance =
    goal.use_lookahead_distance ? lookahead_distance_spin_->value() : -1.0;
  goal.mandatory = mandatory_check_->isChecked();
  return goal;
}

void WaypointRecorderPanel::handleClick(const geometry_msgs::msg::PointStamped & msg)
{
  if (!record_button_->isChecked()) {
    setStatus("Not recording, press 'Record' to add points.", true);
    return;
  }
  if (saving_) {
    setStatus("Saving in progress, click ignored.", true);
    return;
  }

  PendingPoint point;
  point.goal.goal.header = msg.header;
  point.goal.goal.pose.position = msg.point;
  point.goal.goal.pose.orientation.w = 1.0;

  if (!routeMode()) {
    // A single waypoint, a new click replaces it. Name and parameters are read on save.
    pending_ = {point};
    updateWidgets();
    publishMarkers();
    return;
  }

  if (!pending_.empty() && pending_.front().goal.goal.header.frame_id != msg.header.frame_id) {
    setStatus(
      QString("Point is in '%1' but the route in '%2'. Save or clear the route first.")
      .arg(QString::fromStdString(msg.header.frame_id))
      .arg(QString::fromStdString(pending_.front().goal.goal.header.frame_id)), true);
    return;
  }

  QString name = point_name_edit_->text().trimmed();
  if (name.isEmpty()) {
    name = QString("waypoint_%1").arg(pending_.size() + 1);
  }
  for (const auto & other : pending_) {
    if (other.name == name.toStdString()) {
      setStatus(QString("Name '%1' is already used in this route.").arg(name), true);
      return;
    }
  }

  const auto position = point.goal.goal;
  point.goal = goalFromFields();
  point.goal.goal = position;
  point.name = name.toStdString();
  pending_.push_back(point);

  // Next point gets an automatic name unless one is typed again.
  point_name_edit_->clear();
  setStatus(QString("Added '%1'.").arg(name));
  updateWidgets();
  publishMarkers();
}

void WaypointRecorderPanel::onSave()
{
  if (pending_.empty() || saving_) {
    return;
  }
  if (!save_client_ || !save_client_->service_is_ready()) {
    setStatus(
      QString("Service '%1' is not available, is the waypoint_recorder node running?")
      .arg(save_service_), true);
    return;
  }

  auto request = std::make_shared<SaveWaypoints::Request>();
  request->wgs = wgs_frame_button_->isChecked();
  request->overwrite = overwrite_check_->isChecked();

  if (routeMode()) {
    if (!route_name_edit_->hasAcceptableInput()) {
      setStatus("Fill in the route name.", true);
      return;
    }
    request->route_name = route_name_edit_->text().toStdString();
    for (const auto & point : pending_) {
      request->names.push_back(point.name);
      request->goals.push_back(point.goal);
    }
  } else {
    if (!point_name_edit_->hasAcceptableInput()) {
      setStatus("Fill in the waypoint name.", true);
      return;
    }
    auto goal = goalFromFields();
    goal.goal = pending_.front().goal.goal;
    request->names.push_back(point_name_edit_->text().toStdString());
    request->goals.push_back(goal);
  }

  saving_ = true;
  const int request_id = ++request_id_;
  setStatus("Saving...");
  updateWidgets();

  QPointer<WaypointRecorderPanel> self(this);
  save_client_->async_send_request(
    request, [self, request_id](rclcpp::Client<SaveWaypoints>::SharedFuture future) {
      auto response = future.get();
      if (self) {
        QMetaObject::invokeMethod(
          self.data(), [self, response, request_id]() {
            if (self) {self->handleResponse(*response, request_id);}
          }, Qt::QueuedConnection);
      }
    });

  QTimer::singleShot(
    kSaveTimeoutMs, this, [this, request_id]() {
      if (saving_ && request_id == request_id_) {
        saving_ = false;
        save_client_->prune_pending_requests();
        setStatus("No response from the waypoint_recorder node.", true);
        updateWidgets();
      }
    });
}

void WaypointRecorderPanel::handleResponse(
  const SaveWaypoints::Response & response, int request_id)
{
  if (request_id != request_id_ || !saving_) {
    return;  // timed out already
  }
  saving_ = false;

  if (response.success) {
    pending_.clear();
    if (!routeMode()) {
      point_name_edit_->clear();
    }
    publishMarkers();
  }
  setStatus(QString::fromStdString(response.message), !response.success);
  updateWidgets();
}

void WaypointRecorderPanel::updateWidgets()
{
  const bool route = routeMode();
  route_name_edit_->setEnabled(route);
  mandatory_check_->setEnabled(route);
  point_name_edit_->setPlaceholderText(route ? "empty: waypoint_N" : "required");
  overwrite_check_->setToolTip(
    route ? "Replace an existing route file of the same name." :
    "Replace an existing waypoint of the same name.");

  const bool has_points = !pending_.empty();
  undo_button_->setEnabled(has_points && !saving_);
  clear_button_->setEnabled(has_points && !saving_);
  save_button_->setEnabled(has_points && !saving_);

  pending_list_->clear();
  for (size_t i = 0; i < pending_.size(); ++i) {
    const auto & p = pending_[i].goal.goal;
    QString name = route ? QString::fromStdString(pending_[i].name) :
      point_name_edit_->text().isEmpty() ? QString("<name>") : point_name_edit_->text();
    pending_list_->addItem(
      QString("%1. %2  (%3, %4) in '%5'")
      .arg(i + 1).arg(name)
      .arg(p.pose.position.x, 0, 'f', 2).arg(p.pose.position.y, 0, 'f', 2)
      .arg(QString::fromStdString(p.header.frame_id)));
  }
  pending_list_->scrollToBottom();
}

void WaypointRecorderPanel::publishMarkers()
{
  if (!marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  if (!pending_.empty()) {
    // Zero stamp: rviz uses the latest transform, the points are static.
    std_msgs::msg::Header header;
    header.frame_id = pending_.front().goal.goal.header.frame_id;

    visualization_msgs::msg::Marker line;
    line.header = header;
    line.ns = "route";
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.scale.x = 0.08;
    line.color.r = 1.0f;
    line.color.g = 0.6f;
    line.color.a = 0.8f;
    line.pose.orientation.w = 1.0;

    for (size_t i = 0; i < pending_.size(); ++i) {
      const auto & position = pending_[i].goal.goal.pose.position;

      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "points";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.pose.position = position;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4;
      sphere.color.r = 1.0f;
      sphere.color.g = 0.4f;
      sphere.color.a = 1.0f;
      array.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header = header;
      label.ns = "labels";
      label.id = static_cast<int>(i);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position = position;
      label.pose.position.z += 0.6;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.4;
      label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
      label.text = routeMode() ? pending_[i].name :
        (point_name_edit_->text().isEmpty() ? "<name>" : point_name_edit_->text().toStdString());
      array.markers.push_back(label);

      line.points.push_back(position);
    }

    if (line.points.size() > 1) {
      array.markers.push_back(line);
    }
  }

  marker_pub_->publish(array);
}

void WaypointRecorderPanel::setStatus(const QString & text, bool error)
{
  status_label_->setStyleSheet(error ? "color: #d03030;" : "");
  status_label_->setText(text);
}

}  // namespace commander_operator_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(commander_operator_rviz::WaypointRecorderPanel, rviz_common::Panel)
