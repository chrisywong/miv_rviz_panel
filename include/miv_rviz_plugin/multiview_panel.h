#ifndef MULTIVIEW_PANEL_H
#define MULTIVIEW_PANEL_H

// Include the tf2 headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Other includes remain unchanged
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <QPainter>
#include <QPushButton>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>



namespace miv_rviz_plugin
{
    class MultiViewPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        explicit MultiViewPanel(QWidget* parent = nullptr);
        virtual ~MultiViewPanel();

    protected:
        QLabel* compositeArrowLabel_;
        QPushButton* yesButton_;
        QPushButton* noButton_;
        QPushButton* repeatButton_;
        ros::NodeHandle nh_;
        ros::Subscriber linkStatesSub_;
        ros::Subscriber jointStatesSub_;
        ros::Publisher ArrowPub_;
        ros::Subscriber tfSub_;
        ros::Publisher sound_pub_;


        void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

        void publishArrowMarkerForJoint(const std::string& joint_frame_id, int id, const std::string& ns, float r, float g, float b, const geometry_msgs::Pose& pose);
        //void publishArrowMarkerForJoint(const geometry_msgs::Pose& pose, int id, const std::string& ns, float r, float g, float b, const tf2::Vector3& direction);

        void publishArrowMarker(const std::string& joint_frame_id, const geometry_msgs::Pose& pose, int id, const std::string& ns, float r, float g, float b, bool alignWithYAxis);

        void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void updateCompositeImage(double cameraYaw, double wristYaw);
        void drawArrow(QPainter *painter, QPointF center, int angle, const QColor &color, int length);
        void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
        double quaternionToYaw(const geometry_msgs::Quaternion& q);
        void publishArrow(const std::string& frame_id, const std::string& ns, int id, double position, const std::string& color);
        //void publishArrowMarker(const geometry_msgs::Pose& pose, int id, const std::string& ns, float r, float g, float b, bool alignWithYAxis = false);


    protected Q_SLOTS:
            void onYesButtonClicked();
            void onNoButtonClicked();
            void onRepeatButtonClicked();

    private:
        // Optional: Add any additional helper methods or variables here
    };
}

#endif // MULTIVIEW_PANEL_H

