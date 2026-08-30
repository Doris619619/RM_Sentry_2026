#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr int kDefaultTickHz = 20;
std::string upper(std::string value) { for (auto& c : value) c = static_cast<char>(std::toupper(c)); return value; }

struct RefereeState {
  int game_progress{}, remain_hp{400}, bullet_remain{999}, friendly_score{}, enemy_score{}, occupy_status{};
  int robot_id{}, robot_color{}, self_hp{400}, self_max_hp{400}, red_1_hp{400}, red_3_hp{400}, red_7_hp{400};
  int blue_1_hp{400}, blue_3_hp{400}, blue_7_hp{400}, red_dead{}, blue_dead{}; uint8_t suggested_target{};
  double hero_x{}, hero_y{}, engineer_x{}, engineer_y{}, std3_x{}, std3_y{}, std4_x{}, std4_y{}, sentry_x{}, sentry_y{};
};

class StrategyNode final : public rclcpp::Node
{
public:
  StrategyNode() : Node("strategy_node")
  {
    tick_hz_ = declare_parameter<int>("tick_hz", kDefaultTickHz);
    publish_on_change_only_ = declare_parameter<bool>("publish_on_change_only", true);
    danger_hp_ = declare_parameter<int>("danger_hp", 100); sufficient_bullet_ = declare_parameter<int>("sufficient_bullet", 10);
    max_bullet_ = declare_parameter<int>("max_bullet", 150); fixed_supply_ = declare_parameter<int>("fixed_supply", 50);
    central_threshold_ = declare_parameter<int>("central_threshold", 20);  // ROS1 effective value, not XML occupy_threshold.
    occupy_threshold_ = declare_parameter<int>("occupy_threshold", 30); aggressive_threshold_ = declare_parameter<int>("aggressive_threshold", 50);
    attack_threshold_ = declare_parameter<int>("attack_threshold", 5); harm_on_ = declare_parameter<int>("harm_threshold_on", 50); harm_off_ = declare_parameter<int>("harm_threshold_off", 10);
    for (int i = 0; i != 4; ++i) {
      occupy_x_[i] = declare_parameter<double>("goals.occupy.point_" + std::to_string(i) + ".x", 0.0);
      occupy_y_[i] = declare_parameter<double>("goals.occupy.point_" + std::to_string(i) + ".y", 0.0);
    }
    supply_ = pointParameter("supply"); waitforop_ = pointParameter("waitforop"); retreat_ = pointParameter("retreat"); radical_ = pointParameter("radical");
    const auto default_xml = ament_index_cpp::get_package_share_directory("decision_node") + "/config/strategy_tree.xml";
    xml_path_ = declare_parameter<std::string>("bt_xml", default_xml);

    const auto state_qos = rclcpp::QoS(1).reliable();
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/clicked_point", state_qos);
    motion_pub_ = create_publisher<std_msgs::msg::UInt8>("/motion", state_qos);
    recover_pub_ = create_publisher<std_msgs::msg::UInt8>("/recover", state_qos);
    bullet_up_pub_ = create_publisher<std_msgs::msg::UInt8>("/bullet_up", state_qos);
    bullet_num_pub_ = create_publisher<std_msgs::msg::UInt8>("/bullet_num", state_qos);
    subscribeInputs(state_qos);
    configureTree();
    publishUInt(motion_pub_, 0); publishUInt(recover_pub_, 0); publishUInt(bullet_up_pub_, 0); publishUInt(bullet_num_pub_, 0); last_motion_pub_ = 0;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / std::max(1, tick_hz_)));
    tick_timer_ = create_wall_timer(period, [this] { tree_.tickRoot(); });
  }

private:
  using Point = std::pair<double, double>;
  Point pointParameter(const std::string& name) {
    return {declare_parameter<double>("goals." + name + ".x", 0.0), declare_parameter<double>("goals." + name + ".y", 0.0)};
  }
  template<typename Msg, typename Callback> void subscription(const std::string& topic, Callback&& callback, const rclcpp::QoS& qos) {
    std::function<void(typename Msg::ConstSharedPtr)> typed_callback(std::forward<Callback>(callback));
    subscriptions_.push_back(create_subscription<Msg>(topic, qos, typed_callback));
  }
  static void publishUInt(const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr& publisher, const int value) {
    std_msgs::msg::UInt8 message; message.data = static_cast<uint8_t>(std::clamp(value, 0, 255)); publisher->publish(message);
  }
  void subscribeInputs(const rclcpp::QoS& qos) {
    subscription<std_msgs::msg::UInt8>("/referee/game_progress", [this](auto m){ ref_.game_progress=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/remain_hp", [this](auto m){ ref_.remain_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/bullet_remain", [this](auto m){ ref_.bullet_remain=m->data; }, qos);
    subscription<std_msgs::msg::Int32>("/referee/friendly_score", [this](auto m){ ref_.friendly_score=m->data; }, qos);
    subscription<std_msgs::msg::Int32>("/referee/enemy_score", [this](auto m){ ref_.enemy_score=m->data; }, qos);
    subscription<std_msgs::msg::UInt8>("/referee/occupy_status", [this](auto m){ ref_.occupy_status=m->data; }, qos);
    subscription<std_msgs::msg::UInt8>("/robot/robot_id", [this](auto m){ ref_.robot_id=m->data; }, qos);
    subscription<std_msgs::msg::UInt8>("/robot/robot_color", [this](auto m){ ref_.robot_color=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/robot/self_hp", [this](auto m){ ref_.self_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/robot/self_max_hp", [this](auto m){ ref_.self_max_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/red_1_hp", [this](auto m){ ref_.red_1_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/red_3_hp", [this](auto m){ ref_.red_3_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/red_7_hp", [this](auto m){ ref_.red_7_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/blue_1_hp", [this](auto m){ ref_.blue_1_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/blue_3_hp", [this](auto m){ ref_.blue_3_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/blue_7_hp", [this](auto m){ ref_.blue_7_hp=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/red_dead", [this](auto m){ ref_.red_dead=m->data; }, qos);
    subscription<std_msgs::msg::UInt16>("/referee/blue_dead", [this](auto m){ ref_.blue_dead=m->data; }, qos);
    subscription<geometry_msgs::msg::Point>("/enemy/hero_position", [this](auto m){ ref_.hero_x=m->x; ref_.hero_y=m->y; }, qos);
    subscription<geometry_msgs::msg::Point>("/enemy/engineer_position", [this](auto m){ ref_.engineer_x=m->x; ref_.engineer_y=m->y; }, qos);
    subscription<geometry_msgs::msg::Point>("/enemy/standard_3_position", [this](auto m){ ref_.std3_x=m->x; ref_.std3_y=m->y; }, qos);
    subscription<geometry_msgs::msg::Point>("/enemy/standard_4_position", [this](auto m){ ref_.std4_x=m->x; ref_.std4_y=m->y; }, qos);
    subscription<geometry_msgs::msg::Point>("/enemy/sentry_position", [this](auto m){ ref_.sentry_x=m->x; ref_.sentry_y=m->y; }, qos);
    subscription<std_msgs::msg::UInt8>("/radar/suggested_target", [this](auto m){ ref_.suggested_target=m->data; }, qos);
    subscription<std_msgs::msg::Bool>("/dstar_status", [this](auto m){ arrived_=m->data; }, qos);
  }
  static BT::NodeStatus boolStatus(const bool value) { return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }
  Point pointForTarget(const uint8_t target) const {
    switch (target) { case 0:return {hero_x_,hero_y_}; case 1:return {engineer_x_,engineer_y_}; case 2:return {std3_x_,std3_y_}; case 3:return {std4_x_,std4_y_}; case 4:return {sentry_x_,sentry_y_}; default:return {0.0,0.0}; }
  }
  void setGoal(BT::TreeNode& node, const Point goal) {
    geometry_msgs::msg::PointStamped stamped; stamped.header.frame_id="map"; stamped.header.stamp=now(); stamped.point.x=goal.first; stamped.point.y=goal.second;
    node.config().blackboard->set("goal.point", stamped); node.config().blackboard->set("goal.valid", true);
  }
  void publishGoal(BT::TreeNode& node) {
    if (!node.config().blackboard->get<bool>("goal.valid")) return;
    const auto goal=node.config().blackboard->get<geometry_msgs::msg::PointStamped>("goal.point");
    if (publish_on_change_only_ && have_last_goal_ && goal.point.x==last_goal_x_ && goal.point.y==last_goal_y_) return;
    have_last_goal_=true; last_goal_x_=goal.point.x; last_goal_y_=goal.point.y; goal_pub_->publish(goal);
  }
  void updateReferee(BT::TreeNode& node) {
    auto bb=node.config().blackboard; bb->set("ref.game_progress",ref_.game_progress); bb->set("ref.remain_hp",ref_.remain_hp); bb->set("ref.bullet_remain",ref_.bullet_remain);
    bb->set("ref.friendly_score",ref_.friendly_score); bb->set("ref.enemy_score",ref_.enemy_score); bb->set("ref.occupy_status",ref_.occupy_status); bb->set("ref.robot_id",ref_.robot_id); bb->set("ref.robot_color",ref_.robot_color);
    bb->set("ref.self_hp",ref_.self_hp); bb->set("ref.self_max_hp",ref_.self_max_hp); bb->set("ref.red_1_hp",ref_.red_1_hp); bb->set("ref.red_3_hp",ref_.red_3_hp); bb->set("ref.red_7_hp",ref_.red_7_hp); bb->set("ref.blue_1_hp",ref_.blue_1_hp); bb->set("ref.blue_3_hp",ref_.blue_3_hp); bb->set("ref.blue_7_hp",ref_.blue_7_hp); bb->set("ref.red_dead",ref_.red_dead); bb->set("ref.blue_dead",ref_.blue_dead);
    auto cache=[&](double input,double& value){if(input!=-8888.0)value=input;}; cache(ref_.hero_x,hero_x_);cache(ref_.hero_y,hero_y_);cache(ref_.engineer_x,engineer_x_);cache(ref_.engineer_y,engineer_y_);cache(ref_.std3_x,std3_x_);cache(ref_.std3_y,std3_y_);cache(ref_.std4_x,std4_x_);cache(ref_.std4_y,std4_y_);cache(ref_.sentry_x,sentry_x_);cache(ref_.sentry_y,sentry_y_);
    bb->set("ref.enemy_hero_x",static_cast<float>(hero_x_));bb->set("ref.enemy_hero_y",static_cast<float>(hero_y_));bb->set("ref.enemy_engineer_x",static_cast<float>(engineer_x_));bb->set("ref.enemy_engineer_y",static_cast<float>(engineer_y_));bb->set("ref.enemy_standard_3_x",static_cast<float>(std3_x_));bb->set("ref.enemy_standard_3_y",static_cast<float>(std3_y_));bb->set("ref.enemy_standard_4_x",static_cast<float>(std4_x_));bb->set("ref.enemy_standard_4_y",static_cast<float>(std4_y_));bb->set("ref.enemy_sentry_x",static_cast<float>(sentry_x_));bb->set("ref.enemy_sentry_y",static_cast<float>(sentry_y_));bb->set("ref.suggested_target",ref_.suggested_target);
  }
  void updateDerived(BT::TreeNode& node) {
    auto bb=node.config().blackboard; const auto stamp=now().seconds(); if(last_hp_<0) last_hp_=ref_.remain_hp; if(ref_.remain_hp<last_hp_) damage_.emplace_back(stamp,last_hp_-ref_.remain_hp); last_hp_=ref_.remain_hp;
    while (!damage_.empty() && stamp - damage_.front().first > 2.0) {
      damage_.pop_front();
    }
    int total{};
    for (const auto& value : damage_) total += value.second;
    bb->set("derived.damage_2s", total);
    bb->set("is_dead", ref_.remain_hp <= 0);
    bb->set("is_in_danger", ref_.remain_hp > 0 && ref_.remain_hp < danger_hp_);
    bb->set("bullet_sufficient", ref_.bullet_remain >= sufficient_bullet_);
  }
  void configureTree();

  RefereeState ref_{}; bool arrived_{}; int tick_hz_{},danger_hp_{},sufficient_bullet_{},max_bullet_{},fixed_supply_{},central_threshold_{},occupy_threshold_{},aggressive_threshold_{},attack_threshold_{},harm_on_{},harm_off_{}; bool publish_on_change_only_{}; std::string xml_path_;
  Point occupy_x_y_placeholder_; double occupy_x_[4]{},occupy_y_[4]{}; Point supply_,waitforop_,retreat_,radical_; int cycle_index_{},last_hp_{-1},last_attack_hp_{-1},central_count_{},motion_flag_{},last_motion_pub_{-1},last_recover_pub_{-1},last_bullet_up_pub_{-1},last_bullet_num_pub_{-1}; double cooldown_end_{},wait_end_{}; bool intense_harm_{},central_triggered_{},chase_initialized_{},have_last_goal_{}; uint8_t chase_id_{}; double chase_x_{},chase_y_{},last_goal_x_{},last_goal_y_{},hero_x_{},hero_y_{},engineer_x_{},engineer_y_{},std3_x_{},std3_y_{},std4_x_{},std4_y_{},sentry_x_{},sentry_y_{}; std::deque<std::pair<double,int>> damage_;
  BT::BehaviorTreeFactory factory_; BT::Tree tree_; rclcpp::TimerBase::SharedPtr tick_timer_; std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_; rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr motion_pub_,recover_pub_,bullet_up_pub_,bullet_num_pub_;
};

void StrategyNode::configureTree()
{
  auto action=[this](const std::string& name, auto callback, BT::PortsList ports={}) { factory_.registerSimpleAction(name, callback, ports); };
  action("UpdateRefereeBB",[this](BT::TreeNode& n){updateReferee(n);return BT::NodeStatus::SUCCESS;}); action("UpdateNavigationBB",[this](BT::TreeNode& n){n.config().blackboard->set("nav.arrived",arrived_);return BT::NodeStatus::SUCCESS;});
  action("UpdateVisionBB",[](BT::TreeNode&){return BT::NodeStatus::SUCCESS;}); action("UpdateTimersBB",[](BT::TreeNode&){return BT::NodeStatus::SUCCESS;}); action("UpdateDerivedFlags",[this](BT::TreeNode& n){updateDerived(n);return BT::NodeStatus::SUCCESS;},{BT::InputPort<int>("danger_hp",100,""),BT::InputPort<int>("sufficient_bullet",10,"")});
  action("IsGameStarted",[this](BT::TreeNode& n){return boolStatus((ref_.game_progress==4)==n.getInput<bool>("expect_started").value_or(true));},{BT::InputPort<bool>("expect_started",true,"")});
  action("IsSentryDead",[this](BT::TreeNode&){return boolStatus(ref_.remain_hp<=0);}); action("IsSentryAlive",[this](BT::TreeNode&){return boolStatus(ref_.remain_hp>0);}); action("IsSentryInDanger",[this](BT::TreeNode&){return boolStatus(ref_.remain_hp>0&&ref_.remain_hp<danger_hp_);}); action("NotBulletSufficient",[this](BT::TreeNode&){return boolStatus(ref_.bullet_remain<sufficient_bullet_);});
  action("AggressiveAdvantage",[this](BT::TreeNode& n){return boolStatus(ref_.friendly_score-ref_.enemy_score>=n.getInput<int>("threshold").value_or(50));},{BT::InputPort<int>("threshold",50,"")});
  action("IntenseHarm",[this](BT::TreeNode& n){const auto damage=n.config().blackboard->get<int>("derived.damage_2s");const int on=n.getInput<int>("threshold_activate").value_or(100),off=n.getInput<int>("threshold_deactivate").value_or(50);if(!intense_harm_&&damage>on)intense_harm_=true;else if(intense_harm_&&damage<off)intense_harm_=false;return boolStatus(intense_harm_);},{BT::InputPort<int>("threshold_activate",100,""),BT::InputPort<int>("threshold_deactivate",50,"")});
  action("IsAction",[](BT::TreeNode& n){return boolStatus(upper(n.config().blackboard->get<std::string>("action"))==upper(n.getInput<std::string>("value").value_or("INIT")));},{BT::InputPort<std::string>("value","INIT","")}); action("SetAction",[](BT::TreeNode& n){n.config().blackboard->set("action",upper(n.getInput<std::string>("action").value_or("INIT")));return BT::NodeStatus::SUCCESS;},{BT::InputPort<std::string>("action")}); action("ClearGoal",[](BT::TreeNode& n){n.config().blackboard->set("goal.valid",false);return BT::NodeStatus::SUCCESS;});
  action("Wait",[this](BT::TreeNode& n){const auto t=now().seconds();if(wait_end_==0){wait_end_=t+n.getInput<double>("duration").value_or(1.0);return BT::NodeStatus::RUNNING;}if(t>=wait_end_){wait_end_=0;return BT::NodeStatus::SUCCESS;}return BT::NodeStatus::RUNNING;},{BT::InputPort<double>("duration",1.0,"")});
  action("SetGoalFromParams",[this](BT::TreeNode& n){const auto ns=n.getInput<std::string>("ns").value_or("push");if(ns=="supply")setGoal(n,supply_);else if(ns=="waitforop")setGoal(n,waitforop_);else if(ns=="retreat")setGoal(n,retreat_);else setGoal(n,radical_);return BT::NodeStatus::SUCCESS;},{BT::InputPort<std::string>("ns","push","")}); action("SetGoalFromParamsCyclic",[this](BT::TreeNode& n){setGoal(n,{occupy_x_[cycle_index_],occupy_y_[cycle_index_]});return BT::NodeStatus::SUCCESS;},{BT::InputPort<std::string>("ns","push",""),BT::InputPort<int>("point_count",4,"")}); action("AdvanceCycleIndex",[this](BT::TreeNode& n){cycle_index_=(cycle_index_+1)%std::max(1,n.getInput<int>("point_count").value_or(4));return BT::NodeStatus::SUCCESS;},{BT::InputPort<int>("point_count",4,"")}); action("PublishGoalPoint",[this](BT::TreeNode& n){publishGoal(n);return BT::NodeStatus::SUCCESS;},{BT::InputPort<std::string>("topic","clicked_point","")});
  action("CheckArrived",[this](BT::TreeNode&){return boolStatus(arrived_);}); action("CheckAttacked",[this](BT::TreeNode& n){const int last=last_attack_hp_<0?ref_.remain_hp:last_attack_hp_;last_attack_hp_=ref_.remain_hp;return boolStatus(last-ref_.remain_hp>=n.getInput<int>("attack_threshold").value_or(5));},{BT::InputPort<int>("attack_threshold",5,"")}); action("SetMotionFlag",[this](BT::TreeNode& n){const int target=n.getInput<int>("target_motion").value_or(0);const bool cool=motion_flag_!=target&&motion_flag_!=3&&target!=3;if(!cool||now().seconds()>=cooldown_end_){if(cool)cooldown_end_=now().seconds()+5.0;else cooldown_end_=0;motion_flag_=target;}n.config().blackboard->set("motion_flag",motion_flag_);return BT::NodeStatus::SUCCESS;},{BT::InputPort<int>("target_motion")}); action("PublishMotion",[this](BT::TreeNode&){if(!publish_on_change_only_||last_motion_pub_!=motion_flag_){publishUInt(motion_pub_,motion_flag_);last_motion_pub_=motion_flag_;}return BT::NodeStatus::SUCCESS;});
  action("IsHealthFull",[this](BT::TreeNode& n){return boolStatus(ref_.remain_hp>=n.getInput<int>("max_hp").value_or(400));},{BT::InputPort<int>("max_hp",400,"")}); action("SetRecover",[](BT::TreeNode& n){n.config().blackboard->set("recover",n.getInput<int>("value").value_or(0));return BT::NodeStatus::SUCCESS;},{BT::InputPort<int>("value",0,"")}); action("PublishRecover",[this](BT::TreeNode& n){const int v=n.config().blackboard->get<int>("recover");if(!publish_on_change_only_||v!=last_recover_pub_){publishUInt(recover_pub_,v);last_recover_pub_=v;}return BT::NodeStatus::SUCCESS;}); action("IsBulletFull",[this](BT::TreeNode& n){return boolStatus(ref_.bullet_remain>=n.getInput<int>("max_bullet").value_or(999));},{BT::InputPort<int>("max_bullet",999,"")}); action("SetBulletUp",[](BT::TreeNode& n){n.config().blackboard->set("bullet_up",n.getInput<int>("value").value_or(0));return BT::NodeStatus::SUCCESS;},{BT::InputPort<int>("value",0,"")}); action("PublishBulletUp",[this](BT::TreeNode& n){const int v=n.config().blackboard->get<int>("bullet_up");if(!publish_on_change_only_||v!=last_bullet_up_pub_){publishUInt(bullet_up_pub_,v);last_bullet_up_pub_=v;}return BT::NodeStatus::SUCCESS;}); action("SetBulletNum",[this](BT::TreeNode& n){const auto mode=upper(n.getInput<std::string>("mode").value_or("DELTA"));const int value=mode=="FIXED"?n.getInput<int>("fixed_supply").value_or(50):std::max(0,n.getInput<int>("expected_bullet").value_or(999)-ref_.bullet_remain);n.config().blackboard->set("bullet_num",value);return BT::NodeStatus::SUCCESS;},{BT::InputPort<std::string>("mode","DELTA",""),BT::InputPort<int>("expected_bullet",999,""),BT::InputPort<int>("fixed_supply",50,"")}); action("PublishBulletNum",[this](BT::TreeNode& n){const int v=n.config().blackboard->get<int>("bullet_num");if(!publish_on_change_only_||v!=last_bullet_num_pub_){publishUInt(bullet_num_pub_,v);last_bullet_num_pub_=v;}return BT::NodeStatus::SUCCESS;});
  action("AccumulateCentralOccupiable",[this](BT::TreeNode& n){if(n.getInput<int>("occupy_status").value_or(0)!=2&&++central_count_>=central_threshold_){central_count_=0;n.setOutput("reached_threshold",true);}else n.setOutput("reached_threshold",false);n.setOutput("accumulated_count",central_count_);return BT::NodeStatus::SUCCESS;},{BT::InputPort<int>("occupy_status"),BT::InputPort<int>("threshold"),BT::OutputPort<int>("accumulated_count"),BT::OutputPort<bool>("reached_threshold")}); action("TriggerOnThreshold",[this](BT::TreeNode& n){if(n.getInput<bool>("reset_condition").value_or(false))central_triggered_=false;const bool result=n.getInput<bool>("reached_threshold").value_or(false)&&!central_triggered_;central_triggered_|=result;return boolStatus(result);},{BT::InputPort<bool>("reached_threshold"),BT::InputPort<bool>("reset_condition")}); action("ResetAccumulator",[this](BT::TreeNode&){central_count_=0;return BT::NodeStatus::SUCCESS;}); action("ResetCentralOccupiable",[this](BT::TreeNode&){central_count_=0;central_triggered_=false;return BT::NodeStatus::SUCCESS;});
  action("InitChase",[this](BT::TreeNode& n){chase_id_=ref_.suggested_target;if(chase_id_>4)return BT::NodeStatus::FAILURE;const auto p=pointForTarget(chase_id_);chase_x_=p.first;chase_y_=p.second;chase_initialized_=true;n.config().blackboard->set("chase.target_id",chase_id_);return BT::NodeStatus::SUCCESS;}); action("UpdateChaseTarget",[this](BT::TreeNode& n){if(!chase_initialized_)return BT::NodeStatus::FAILURE;const auto p=pointForTarget(chase_id_);chase_x_=p.first;chase_y_=p.second;n.config().blackboard->set("chase.target_x",static_cast<float>(chase_x_));n.config().blackboard->set("chase.target_y",static_cast<float>(chase_y_));return BT::NodeStatus::SUCCESS;}); action("PublishChaseGoal",[this](BT::TreeNode& n){setGoal(n,{chase_x_,chase_y_});if(!have_last_goal_||std::hypot(chase_x_-last_goal_x_,chase_y_-last_goal_y_)>=0.01)publishGoal(n);return BT::NodeStatus::SUCCESS;}); action("ResetChase",[this](BT::TreeNode&){chase_initialized_=false;return BT::NodeStatus::SUCCESS;});
  auto bb=BT::Blackboard::create(); bb->set("action",std::string("INIT"));bb->set("goal.valid",false);bb->set("motion_flag",0);bb->set("recover",0);bb->set("bullet_up",0);bb->set("bullet_num",0);bb->set("is_enemy_occupied",false);
  bb->set("danger_hp",danger_hp_);bb->set("sufficient_bullet",sufficient_bullet_);bb->set("max_bullet",max_bullet_);bb->set("fixed_supply",fixed_supply_);bb->set("occupy_threshold",occupy_threshold_);bb->set("aggressive_threshold",aggressive_threshold_);bb->set("attack_threshold",attack_threshold_);bb->set("harm_threshold_on",harm_on_);bb->set("harm_threshold_off",harm_off_);
  tree_=factory_.createTreeFromFile(xml_path_,bb);
}

}  // namespace

int main(int argc,char** argv) { rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<StrategyNode>());rclcpp::shutdown();return 0; }

