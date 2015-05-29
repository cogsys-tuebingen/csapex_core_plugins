#ifndef ASSIGN_CLUSTER_CLASS_H
#define ASSIGN_CLUSTER_CLASS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_vision/roi_message.h>

/// SYSTEM
#include <QImage>
#include <QSharedPointer>


namespace vision_plugins {
class AssignClusterClass : public csapex::InteractiveNode
{
    friend class AssignClusterClassAdapter;

public:
    AssignClusterClass();
    virtual ~AssignClusterClass();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);
    virtual void process() override;

    void setResult(std::vector<int> &result);
    void setActiveClassColor(const int r, const int g, const int b);

private:
    void submit();
    void drop();
    void setColor();
    void setClass();
    void display();

protected:
    csapex::Input*    in_image_;
    csapex::Input*    in_clusters_;
    csapex::Output*   out_labels_;

    cv::Mat           image_;
    cv::Mat           mask_;
    cv::Mat           clusters_;

    std::shared_ptr<std::vector<int>> result_;


public:
    boost::signals2::signal<void(QSharedPointer<QImage>, const cv::Mat&)> display_request;
    boost::signals2::signal<void()>                                       submit_request;
    boost::signals2::signal<void()>                                       drop_request;
    boost::signals2::signal<void(int)>                                    set_class;
    boost::signals2::signal<void(int,int,int)>                            set_color;
};

}

#endif // ASSIGN_CLUSTER_CLASS_H
