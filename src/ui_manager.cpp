#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <quirkd/Alert.h>
#include <quirkd/AlertArray.h>
#include <visualization_msgs/Marker.h>

class UIManager
{
public:
  UIManager()
  {
    this->n = new ros::NodeHandle();
    lowPub = this->n->advertise<geometry_msgs::PolygonStamped>("/low_alert", 1);
    warnPub = this->n->advertise<geometry_msgs::PolygonStamped>("/warn_alert", 1);
    maxPub = this->n->advertise<geometry_msgs::PolygonStamped>("/max_alert", 1);
    linePub = this->n->advertise<visualization_msgs::Marker>("/max_line_alert", 1);
    alertSub = this->n->subscribe("/quirkd/alert/notification", 1, &UIManager::alertCB, this);
    alertArraySub = this->n->subscribe("/quirkd/alert_array/notification", 1, &UIManager::alertArrayCB, this);
    ROS_INFO("Done with UIManager constructor");
  }
  void alertCB(const quirkd::Alert &alert)
  {
    ROS_DEBUG("Alert CB");
    if (alert.level < 10)
    {
      publishPolygon(&lowPub, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
    }
    else if (alert.level > 100)
    {
      publishPolygon(&maxPub, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
    }
    else
    {
      publishPolygon(&warnPub, alert.min_x, alert.max_x, alert.min_y, alert.max_y);
    }
  }
  void alertArrayCB(const quirkd::AlertArray &msg)
  {
    ROS_DEBUG("Alerts CB");
    std::vector<geometry_msgs::Point> points;
    for (size_t i = 0; i < msg.alerts.size(); i++)
    {
      quirkd::Alert alert = msg.alerts[i];
      this->extendLineList(&points, &alert);
    }
    this->publishLineList(&maxPub, &points);
  }
  void extendLineList(std::vector<geometry_msgs::Point> *points, quirkd::Alert *msg)
  {
    geometry_msgs::Point minmin;
    minmin.x = msg->min_x;
    minmin.y = msg->min_y;
    geometry_msgs::Point minmax;
    minmin.x = msg->min_x;
    minmax.y = msg->max_y;
    geometry_msgs::Point maxmax;
    maxmax.x = msg->max_x;
    maxmax.y = msg->max_y;
    geometry_msgs::Point maxmin;
    maxmin.x = msg->max_x;
    maxmin.y = msg->min_y;

    points->push_back(minmin);
    points->push_back(minmax);

    points->push_back(minmax);
    points->push_back(maxmax);

    points->push_back(maxmax);
    points->push_back(maxmin);

    points->push_back(maxmin);
    points->push_back(minmin);
  }
  void publishLineList(ros::Publisher *pub, std::vector<geometry_msgs::Point> *points)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.id = 1;
    marker.action = visualization_msgs::Marker::LINE_LIST;
    marker.type = visualization_msgs::Marker::MODIFY;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    // Chance color based on level
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.frame_locked = false;

    marker.points = *points;
    pub->publish(marker);
  }
  void publishPolygon(ros::Publisher *pub, float min_x, float max_x, float min_y, float max_y)
  {
    if (max_x - min_x < .01)
    {
      max_x += .05;
      min_x -= .05;
    }
    if (max_y - min_y < .01)
    {
      max_y += .05;
      min_y -= .05;
    }
    geometry_msgs::PolygonStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "map";

    geometry_msgs::Point32 p1;
    p1.x = min_x;
    p1.y = min_y;
    p1.z = 0;
    ps.polygon.points.push_back(p1);
    geometry_msgs::Point32 p2;
    p2.x = max_x;
    p2.y = min_y;
    p2.z = 0;
    ps.polygon.points.push_back(p2);
    geometry_msgs::Point32 p3;
    p3.x = max_x;
    p3.y = max_y;
    p3.z = 0;
    ps.polygon.points.push_back(p3);
    geometry_msgs::Point32 p4;
    p4.x = min_x;
    p4.y = max_y;
    p4.z = 0;
    ps.polygon.points.push_back(p4);

    ROS_INFO("Publish polygon %.2f %.2f %.2f %.2f", min_x, max_x, min_y, max_y);
    pub->publish(ps);
  }

private:
  ros::NodeHandle *n;
  ros::Publisher lowPub;
  ros::Publisher warnPub;
  ros::Publisher maxPub;
  ros::Publisher linePub;
  ros::Subscriber alertSub;
  ros::Subscriber alertArraySub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "UIManager");
  ROS_INFO("Init in UIManager");
  UIManager ui;

  ros::spin();
}
