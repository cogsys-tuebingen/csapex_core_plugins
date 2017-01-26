#ifndef CREATE_MAP_MESSAGE_H
#define CREATE_MAP_MESSAGE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_core_plugins/map_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/serialization/serialization.h>

namespace csapex
{


class CSAPEX_EXPORT_PLUGIN CreateMapMessage : public Node, public VariadicInputs
{
    friend class CreateMapMessageAdapter;
public:
    CreateMapMessage();


    void setup(csapex::NodeModifier& modifier) override;


    void setupParameters(csapex::Parameterizable& params) override;

    void process() override;

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override;


private:
    Input* in_;
    Output* out_;

//    std::vector<param::Parameter::Ptr> params_keys_;
//    std::vector<std::string> keys_;

};

} // csapex


#endif // CREATE_MAP_MESSAGE_H
