#ifndef LISTENER_H
#define LISTENER_H

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/singleton.hpp>
#include <csapex_ros/ros_handler.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <tf/transform_listener.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

namespace csapex
{
class LockedTFListener;

class TFListener : public Singleton<TFListener>
{
    friend class LockedTFListener;
    friend class Singleton<TFListener>;

public:
    std::shared_ptr<tf::TransformListener> tfl;

    static LockedTFListener getLocked();

    static void start();
    static void stop();

    void reset()
    {
        tfl->clear();
        std::cout << "reset tf listener" << std::endl;
        tfl.reset(new tf::TransformListener);
    }
    bool ok();

    void addTransforms(const tf2_msgs::TFMessage& msg);

private:
    TFListener();

    void cb(const tf::tfMessage::ConstPtr& msg);
    bool tryFrameAsReference(const tf::tfMessage::ConstPtr& msg, const std::string& frame);

    int retries;
    std::string reference_frame;
    bool init;
    ros::Time last_;
    ros::Subscriber tf_sub;

    std::mutex m;
};

class LockedTFListener
{
public:
    TFListener* l;

    LockedTFListener(TFListener* ll) : l(nullptr)
    {
        if (ll) {
            ll->m.lock();
            l = ll;
        }
    }

    ~LockedTFListener()
    {
        if (l) {
            l->m.unlock();
        }
    }
};

}  // namespace csapex

#endif  // LISTENER_H
