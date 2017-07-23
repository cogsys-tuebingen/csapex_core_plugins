#include "ros_message_bool_2_event.h"
using namespace csapex;
using namespace csapex::connection_types;


CSAPEX_REGISTER_CLASS(csapex::RosMessageBool2Event, csapex::Node)

RosMessageBool2Event::RosMessageBool2Event():
    last_(false)
{

}

void RosMessageBool2Event::setup(csapex::NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericValueMessage<bool>>("input");
    event_false_ = node_modifier.addEvent("false");
    event_true_ = node_modifier.addEvent("true");
    event_changed_ = node_modifier.addEvent("changed");

}


void RosMessageBool2Event::setupParameters(csapex::Parameterizable& parameters)
{

}

void RosMessageBool2Event::process()
{



    GenericValueMessage<bool>::ConstPtr in = msg::getMessage<connection_types::GenericValueMessage<bool>>(in_);
      if( in->value != last_)
      {
          event_changed_->trigger();
          ainfo<< "changed" <<std::endl;
          if(in->value){
              event_true_->trigger();
          }
          else{
              event_false_->trigger();
          }
      }
      last_ = in->value;
}
