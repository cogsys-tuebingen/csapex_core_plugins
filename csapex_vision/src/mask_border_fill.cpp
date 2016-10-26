
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class MaskBorderFill : public Node
{
public:
    MaskBorderFill()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<connection_types::CvMatMessage>("mask");

        out_ = modifier.addOutput<connection_types::CvMatMessage>("filled mask");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        auto image = msg::getMessage<connection_types::CvMatMessage>(in_);
        cv::Mat mask = image->value;

        apex_assert(image->getEncoding().matches(enc::mono));
        apex_assert(image->value.type() == CV_8UC1);

        std::deque<cv::Point> Q;
        for(int c = 0, cols = mask.cols; c < cols; ++c)
        {
            {
                cv::Point pt(c, 0);
                if(mask.at<uchar>(pt)) {
                    Q.push_back(pt);
                }
            }
            {
                cv::Point pt(c, mask.rows-1);
                if(mask.at<uchar>(pt)) {
                    Q.push_back(pt);
                }
            }
        }
        for(int r = 1, rows = mask.rows-1; r < rows; ++r)
        {
            {
                cv::Point pt(0, r);
                if(mask.at<uchar>(pt)) {
                    Q.push_back(pt);
                }
            }
            {
                cv::Point pt(mask.cols-1, r);
                if(mask.at<uchar>(pt)) {
                    Q.push_back(pt);
                }
            }
        }

        cv::Rect rect(0, 0, mask.cols, mask.rows);

        while(!Q.empty()) {
            cv::Point front = Q.front();
            Q.pop_front();

            mask.at<uchar>(front) = 0;

            static const int dx[] = {-1, 0, 1, 0};
            static const int dy[] = {0, -1, 0, 1};

            for(int i = 0; i < 4; ++i) {
                cv::Point n = front;
                n.x += dx[i];
                n.y += dy[i];

                if(rect.contains(n)) {
                    uchar val = mask.at<uchar>(n);
                    if(val) {
                        mask.at<uchar>(n) = 0;
                        Q.push_back(n);
                    }
                }
            }

        }

        auto result = std::make_shared<connection_types::CvMatMessage>(image->getEncoding(), image->stamp_micro_seconds);
        result->value = mask;
        msg::publish(out_, result);
    }

private:
    Input* in_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::MaskBorderFill, csapex::Node)

