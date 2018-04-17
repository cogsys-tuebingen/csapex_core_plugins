/// HEADER
#include "pcd_provider.h"


/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>

#include <pcl/io/pcd_io.h>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/for_each.hpp>

CSAPEX_REGISTER_CLASS(csapex::PCDPointCloudMessageProvider, csapex::MessageProvider)

using namespace csapex;
using namespace connection_types;

struct try_convert
{
    try_convert(const pcl::PCLPointCloud2 &pcl_blob, typename connection_types::PointCloudMessage::Ptr& out, bool full_match, bool& success)
        : pcl_blob_(pcl_blob), out_(out), full_match_(full_match), success_(success)
    {
        success_ = false;
    }

    template<typename PointT>
    void operator()(PointT& pt)
    {
        if(success_) {
            return;
        }

        std::vector<pcl::PCLPointField> fields;
        pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

        const std::vector< ::pcl::PCLPointField> &available_fields = pcl_blob_.fields;


        if(full_match_) {
            if(fields.size() != available_fields.size()) {
                return;
            }
        }

        for(size_t d = 0; d < fields.size (); ++d) {
            bool found = false;
            for(size_t f = 0; f < available_fields.size() && !found; ++f) {
                if(fields[d].name == available_fields[f].name) {
                    found = true;
                }
            }

            if(!found) {
                return;
            }
        }

        success_ = true;
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromPCLPointCloud2(pcl_blob_, *cloud);
        out_->value = cloud;
        out_->frame_id = pcl_blob_.header.frame_id;
        out_->stamp_micro_seconds = pcl_blob_.header.stamp;
    }

    const pcl::PCLPointCloud2 &pcl_blob_;
    typename connection_types::PointCloudMessage::Ptr& out_;

    bool full_match_;
    bool& success_;
};


std::map<std::string, PCDPointCloudMessageProvider::ProviderConstructor> PCDPointCloudMessageProvider::plugins;

PCDPointCloudMessageProvider::PCDPointCloudMessageProvider() :
    sent_(false)
{
    setType(makeEmpty<PointCloudMessage>());
}

void PCDPointCloudMessageProvider::load(const std::string &file)
{
    pcl::PCLPointCloud2 pcl_blob;
    pcl::io::loadPCDFile(file, pcl_blob);

    if(pcl_blob.header.frame_id == "") {
        pcl_blob.header.frame_id = "cloud_frame";
    }

    point_cloud_.reset(new PointCloudMessage(pcl_blob.header.frame_id, pcl_blob.header.stamp));
    bool success;
    try_convert full_match_converter(pcl_blob, point_cloud_, true, success);
    boost::mpl::for_each<connection_types::PointCloudPointTypes>( full_match_converter );

    if(!success) {
        try_convert partial_converter(pcl_blob, point_cloud_, false, success);
        boost::mpl::for_each<connection_types::PointCloudPointTypes>( partial_converter );
    }

    if(!success) {
        std::cerr << "cannot convert message, type is not known. Fields:";
        for(const auto& field : pcl_blob.fields) {
            std::cerr << field.name << " ";
        }
        std::cerr << std::endl;
        point_cloud_.reset();
    }
}

std::vector<std::string> PCDPointCloudMessageProvider::getExtensions() const
{
    return {".pcd"};
}

bool PCDPointCloudMessageProvider::hasNext()
{
    return point_cloud_ && (!sent_ || state.readParameter<bool>("playback/resend"));
}

connection_types::Message::Ptr PCDPointCloudMessageProvider::next(std::size_t slot)
{
    sent_ = true;
    return point_cloud_;
}

Memento::Ptr PCDPointCloudMessageProvider::getState() const
{
    GenericState::Ptr r(new GenericState(state));
    return r;
}

void PCDPointCloudMessageProvider::setParameterState(Memento::Ptr memento)
{
    std::shared_ptr<GenericState> m = std::dynamic_pointer_cast<GenericState> (memento);
    if(m) {
        state.setFrom(*m);
    }
}
