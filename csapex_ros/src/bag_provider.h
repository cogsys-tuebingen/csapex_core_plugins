#ifndef BAG_PROVIDER_H
#define BAG_PROVIDER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>

/// SYSTEM
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace csapex
{

//class BagProvider : public MessageProvider
//{
//public:
//    BagProvider();
//    void load(const std::string& file);

//    ~BagProvider();

//public:
//    virtual bool hasNext();
//    virtual connection_types::Message::Ptr next();

//    virtual std::vector<std::string> getExtensions() const;

//    virtual Memento::Ptr getState() const;
//    virtual void setParameterState(Memento::Ptr memento);

//private:
//    std::string file_;
//    int frames_;

//    rosbag::Bag bag;
//    rosbag::View* view_;
//    rosbag::View::iterator view_it;

//    bool initiated;
//};

} /// NAMESPACE


#endif // BAG_PROVIDER_H
