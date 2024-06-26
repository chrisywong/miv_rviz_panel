#include "miv_rviz_plugin/multiview_panel.h"
#include <QVBoxLayout>
#include <QPainter>
#include <QLabel>
#include <gazebo_msgs/LinkStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <QPushButton>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sound_play/SoundRequest.h>



namespace miv_rviz_plugin
{
    MultiViewPanel::MultiViewPanel(QWidget* parent)
        : rviz::Panel(parent)
    {
        QVBoxLayout* layout = new QVBoxLayout(this);
        compositeArrowLabel_ = new QLabel(this);
        layout->addWidget(compositeArrowLabel_);

        QPushButton* yesButton = new QPushButton("Yes", this);
        layout->addWidget(yesButton);
        connect(yesButton, &QPushButton::clicked, this, &MultiViewPanel::onYesButtonClicked);


        QPushButton* noButton = new QPushButton("No", this);
        layout->addWidget(noButton);
        connect(noButton, &QPushButton::clicked, this, &MultiViewPanel::onNoButtonClicked);


        QPushButton* repeatButton = new QPushButton("Repeat Question", this);
        layout->addWidget(repeatButton);
        connect(repeatButton, &QPushButton::clicked, this, &MultiViewPanel::onRepeatButtonClicked);


        setLayout(layout);

        tfSub_ = nh_.subscribe("/tf", 1, &MultiViewPanel::tfCallback, this);

        //linkStatesSub_ = nh_.subscribe("/gazebo/link_states", 1, &MultiViewPanel::linkStatesCallback, this);

        jointStatesSub_ = nh_.subscribe("/joint_states",1, &MultiViewPanel::jointStatesCallback, this);
        ArrowPub_ = nh_.advertise<visualization_msgs::Marker>("arrow_marker", 1);
        sound_pub_ = nh_.advertise<sound_play::SoundRequest>("robotsound", 1);

        updateCompositeImage(0,0);
    }


    void MultiViewPanel::updateCompositeImage(double cameraYaw, double wristYaw) {
        QPixmap compositePixmap(240, 180); // Adjusted for additional drawing space
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

    void MultiViewPanel::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
        for (const auto& transform : msg->transforms) {
            if (transform.child_frame_id == "link_head_pan" || transform.child_frame_id == "link_wrist_yaw") {
                // Extract the pose
                geometry_msgs::Pose pose;
                pose.position.x = transform.transform.translation.x;
                pose.position.y = transform.transform.translation.y;
                pose.position.z = transform.transform.translation.z;
                //pose.orientation = transform.transform.rotation;


                // Now call publishArrowMarkerForJoint with this new pose
                int id = (transform.child_frame_id == "link_head_pan") ? 1 : 2;
                std::string ns = (transform.child_frame_id == "link_head_pan") ? "head_pan_direction" : "wrist_yaw_direction";
                if (transform.child_frame_id =="link_wrist_yaw"){
                  publishArrowMarker(transform.child_frame_id, pose, id, ns, 0.0, 0.0, 1.0, true);
                }
                else{
                publishArrowMarker(transform.child_frame_id, pose, id, ns, 0.0, 1.0, 0.0, false);
            }
                }
        }
    }



    void MultiViewPanel::publishArrowMarker(const std::string& joint_frame_id, const geometry_msgs::Pose& pose, int id, const std::string& ns, float r, float g, float b, bool alignWithYAxis = false) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = joint_frame_id; // Use a consistent reference frame, like "base_link"
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        if (alignWithYAxis) {
            // Set the orientation to align with the Y-axis directly
            tf2::Quaternion orientationAlignedWithY;
            orientationAlignedWithY.setRPY(0, 0, M_PI / 2); // Rotate 90 degrees around Z-axis
            marker.pose.orientation = tf2::toMsg(orientationAlignedWithY);
        } else {
            // Use the original orientation
            marker.pose = pose;
        }

        marker.scale.x = 0.3; // Length
        marker.scale.y = 0.03; // Width
        marker.scale.z = 0.05; // Height
        marker.color.a = 1.0; // Opacity
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        ArrowPub_.publish(marker);
    }





    void MultiViewPanel::publishArrowMarkerForJoint(const std::string& joint_frame_id, int id, const std::string& ns, float r, float g, float b, const geometry_msgs::Pose& pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id =joint_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // Set pose directly from the argument
        marker.pose = pose;

        marker.scale.x = 0.1; // Arrow length
        marker.scale.y = 0.02; // Arrow width
        marker.scale.z = 0.02; // Arrow height

        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        ArrowPub_.publish(marker);
    }



    void MultiViewPanel::onNoButtonClicked() {
      ROS_INFO("No button clicked");
      std::string s = "No";
      std::string voice = "voice_cmu_us_slt_arctic_hts"; //voice_us1_mbrola
      float volume = 1.0;

    //   ROS_INFO("Saying: %s", s.c_str());
    //   ROS_INFO("Voice: %s", voice.c_str());
    //   ROS_INFO("Volume: %f", volume);

      if (sound_pub_ && sound_pub_.getNumSubscribers() > 0) {
              sound_play::SoundRequest sound;
              sound.sound = sound_play::SoundRequest::SAY;
              sound.command = sound_play::SoundRequest::PLAY_ONCE;
              sound.volume = volume;
              sound.arg = s;
              sound.arg2 = voice;

              sound_pub_.publish(sound);
              ROS_INFO("Sound message published.");
          } else {
              ROS_WARN("No subscribers found on the sound topic, not publishing sound.");
          }
    }

    void MultiViewPanel::onYesButtonClicked() {
        ROS_INFO("Yes button clicked");
        std::string s = "Yes";
        std::string voice = "voice_cmu_us_slt_arctic_hts"; //voice_us1_mbrola
        float volume = 1.0;

        // ROS_INFO("Saying: %s", s.c_str());
        // ROS_INFO("Voice: %s", voice.c_str());
        // ROS_INFO("Volume: %f", volume);

        if (sound_pub_ && sound_pub_.getNumSubscribers() > 0) {
                sound_play::SoundRequest sound;
                sound.sound = sound_play::SoundRequest::SAY;
                sound.command = sound_play::SoundRequest::PLAY_ONCE;
                sound.volume = volume;
                sound.arg = s;
                sound.arg2 = voice;

                sound_pub_.publish(sound);
                ROS_INFO("Sound message published.");
            } else {
                ROS_WARN("No subscribers found on the sound topic, not publishing sound.");
            }
    }


    void MultiViewPanel::onRepeatButtonClicked() {
      ROS_INFO("Repeat button clicked");
      std::string s = "Can you please repeat the question";
      std::string voice = "voice_cmu_us_slt_arctic_hts"; //voice_us1_mbrola
      float volume = 1.0;

    //   ROS_INFO("Saying: %s", s.c_str());
    //   ROS_INFO("Voice: %s", voice.c_str());
    //   ROS_INFO("Volume: %f", volume);

      if (sound_pub_ && sound_pub_.getNumSubscribers() > 0) {
              sound_play::SoundRequest sound;
              sound.sound = sound_play::SoundRequest::SAY;
              sound.command = sound_play::SoundRequest::PLAY_ONCE;
              sound.volume = volume;
              sound.arg = s;
              sound.arg2 = voice;

              sound_pub_.publish(sound);
              ROS_INFO("Sound message published.");
          } else {
              ROS_WARN("No subscribers found on the sound topic, not publishing sound.");
          }
    }



      MultiViewPanel::~MultiViewPanel() {}
}




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(miv_rviz_plugin::MultiViewPanel, rviz::Panel)
