#ifndef LISTENER_H
#define LISTENER_H

/// PROJECT
#include <csapex_ros/ros_handler.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <tf/transform_listener.h>

namespace csapex
{

struct LockedListener;

struct Listener {
    friend class LockedListener;

public:
    boost::shared_ptr<tf::TransformListener> tfl;

    static LockedListener getLocked();

    static void start();
    static void stop();

    void reset() {
        tfl->clear();
        std::cout << "reset tf listener" << std::endl;
        tfl.reset(new tf::TransformListener);
    }
    bool ok();

private:
    static Listener* raw_instance() {
        static Listener l;
        apex_assert_hard(&l);
        return &l;
    }

    Listener();

    void cb(const tf::tfMessage::ConstPtr& msg);
    bool tryFrameAsReference(const tf::tfMessage::ConstPtr &msg, const std::string& frame);

    int retries;
    std::string reference_frame;
    bool init;
    ros::Time last_;
    ros::Subscriber tf_sub;

    QMutex m;
};

struct LockedListener {
public:
    Listener* l;

    LockedListener(Listener* ll)
        : l(NULL)
    {
        if(ll) {
            ll->m.lock();
            l = ll;
        }
    }

    ~LockedListener()
    {
        if(l) {
            l->m.unlock();
        }
    }
};

}

#endif // LISTENER_H
