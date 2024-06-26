#include "miv_rviz_plugin/multiview_panel.h"
#include <QVBoxLayout>
#include <QPainter>
#include <QLabel>
#include <gazebo_msgs/LinkStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace miv_rviz_plugin
{
    MultiViewPanel::MultiViewPanel(QWidget* parent)
        : rviz::Panel(parent)
    {
        QVBoxLayout* layout = new QVBoxLayout(this);
        compositeArrowLabel_ = new QLabel(this);
        layout->addWidget(compositeArrowLabel_);
        setLayout(layout);

        linkStatesSub_ = nh_.subscribe("/gazebo/link_states", 1, &MultiViewPanel::linkStatesCallback, this);

        jointStatesSub_ = nh_.subscribe("/joint_states",1, &MultiViewPanel::jointStatesCallback, this);
        ArrowPub_ = nh_.advertise<visualization_msgs::Marker>("arrow_marker", 1);
        updateCompositeImage(0,0);
    }

    void MultiViewPanel::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
        int cameraIndex = -1;
        int wristYawIndex = -1;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "robot::link_head_pan") {
                cameraIndex = i;
            } else if (msg->name[i] == "robot::link_wrist_yaw") {
                wristYawIndex = i;
            }
        }

        if (cameraIndex != -1) {
            publishArrowMarker(msg->pose[cameraIndex], cameraIndex, "head_pan_direction", 0.0, 1.0, 0.0, false); // Red arrow
        }

        if (wristYawIndex != -1) {
            publishArrowMarker(msg->pose[wristYawIndex], wristYawIndex, "wrist_yaw_direction", 0.0, 0.0, 1.0, true); //Blue arrow
        }
    }

    void MultiViewPanel::publishArrowMarker(const geometry_msgs::Pose& pose, int id, const std::string& ns, float r, float g, float b, bool alignWithYAxis = false) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom"; //  reference frame
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;


        if (alignWithYAxis) {
            tf2::Quaternion orientationAdjustment;
            orientationAdjustment.setRPY(0, 0, M_PI / 2);
            tf2::Quaternion originalOrientation;
            tf2::fromMsg(pose.orientation, originalOrientation);
            tf2::Quaternion newOrientation = originalOrientation * orientationAdjustment;
            newOrientation.normalize();
            marker.pose.orientation = tf2::toMsg(newOrientation);
        }

        // Marker scale and color
        marker.scale.x = 0.3; // Length
        marker.scale.y = 0.03; // Width
        marker.scale.z = 0.05; // Height
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        ArrowPub_.publish(marker); // Use the specific publisher for each marker
    }



    void MultiViewPanel::updateCompositeImage(double cameraYaw, double wristYaw) {
        QPixmap compositePixmap(250, 250); // Adjusted for additional drawing space
        compositePixmap.fill(Qt::transparent);

        QPainter painter(&compositePixmap);
        painter.setRenderHint(QPainter::Antialiasing);

        // Draw robot base first
        QRectF baseRect(40, 80, 120, 60);
        painter.setBrush(Qt::lightGray);
        painter.drawRoundedRect(baseRect, 10, 10);


        QPointF baseCenter = baseRect.center();

        QPointF redArrowStartPoint = baseCenter - QPointF(0, 30);


        QPointF blueArrowStartPoint = baseCenter + QPointF(70, 0);




        drawArrow(&painter, redArrowStartPoint, 0, Qt::red, 60);

        drawArrow(&painter, baseCenter, -static_cast<int>(cameraYaw), Qt::green, 40);


        drawArrow(&painter, blueArrowStartPoint, -static_cast<int>(wristYaw), Qt::blue, 30);

        compositeArrowLabel_->setPixmap(compositePixmap);
    }


    void MultiViewPanel::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        double headPanAngleRadians = 0.0;
        double wristYawAngleRadians = 0.0;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "joint_head_pan") {
                headPanAngleRadians = msg->position[i];
            } else if (msg->name[i] == "joint_wrist_yaw") {
                wristYawAngleRadians = msg->position[i];
            }
        }

        // Convert radians to degrees
        double headPanAngleDegrees = headPanAngleRadians * (180.0 / M_PI);
        double wristYawAngleDegrees = (wristYawAngleRadians * (180.0 / M_PI))-90;
        updateCompositeImage(headPanAngleDegrees, wristYawAngleDegrees);
    }
    void MultiViewPanel::drawArrow(QPainter *painter, QPointF center, int angle, const QColor &color, int length) {
        painter->save();
        painter->translate(center);
        painter->rotate(angle);

        // Arrow stick
        painter->setPen(QPen(color, 3)); // Set the color and thickness of the line
        painter->drawLine(0, 0, 0, -length); // Draw line upwards from center

        // Arrow head
        int headSize = 10; // Size of the arrow head
        QPolygon arrowHead;
        arrowHead << QPoint(-headSize, -length) << QPoint(0, -length-headSize)
                  << QPoint(headSize, -length);
        painter->setBrush(color); // Fill color for the arrow head
        painter->drawPolygon(arrowHead);

        painter->restore();
    }



    double MultiViewPanel::quaternionToYaw(const geometry_msgs::Quaternion& q) {
        // Calculate yaw (rotation around z-axis)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        return yaw;
    }




    MultiViewPanel::~MultiViewPanel() {}
}




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(miv_rviz_plugin::MultiViewPanel, rviz::Panel)
