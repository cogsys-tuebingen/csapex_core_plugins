/// HEADER
#include "foreach.h"

/// COMPONENT
#include <csapex_core_plugins/vector_message.h>
#include <csapex/utility/assert.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex/utility/timer.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Foreach, csapex::Node)

using namespace csapex;
using namespace connection_types;

Foreach::Foreach()
    : in_sub(NULL), out_sub(NULL), msg_received_(false)
{
}

Foreach::~Foreach()
{
    if(in_sub){
        in_sub->deleteLater();
    }
    if(out_sub) {
        out_sub->deleteLater();
    }
}

void Foreach::setup()
{
    input_ = modifier_->addInput<VectorMessage>("Vector");
    output_ = modifier_->addOutput<VectorMessage>("Content");


    out_sub = new ConnectorOut(getSettings(), UUID::make_sub(getUUID(), "out_sub"));
    in_sub = new ConnectorIn(getSettings(), UUID::make_sub(getUUID(), "in_sub"));

    out_sub->setType(AnyMessage::make());
    in_sub->setType(AnyMessage::make());

    out_sub->enable();
    in_sub->enable();

    manageInput(in_sub);
    manageOutput(out_sub);

    QObject::connect(in_sub, SIGNAL(connectionRemoved()), output_, SLOT(disable()));
    QObject::connect(in_sub, SIGNAL(connectionDone()), output_, SLOT(enable()));
    QObject::connect(in_sub, SIGNAL(connectionEnabled(bool)), output_, SLOT(setEnabled(bool)));

    QObject::connect(in_sub, SIGNAL(messageArrived(Connectable*)), this, SLOT(messageProcessed()), Qt::DirectConnection);

    checkIO();
}

void Foreach::process()
{
    VectorMessage::Ptr vec = input_->getMessage<VectorMessage>();

    out_sub->setType(vec->getSubType());

    std::size_t messages = vec->value.size();


    VectorMessage::Ptr result(new VectorMessage);

    // iterate all entries of the vector
    for(std::size_t i = 0; i < messages; ++i) {
        std::stringstream step; step << "step " << i;
        Timer::Interlude::Ptr interlude = profiling_timer_->step(step.str());
        out_sub->setSequenceNumber(vec->sequenceNumber()+1);

        // send out ith component
        out_sub->publish(vec->value[i]);
        out_sub->sendMessages();

        // wait for the message to be processed
        msg_mutex_.lock();
        while(!msg_received_) {
            msg_received_cond_.wait(&msg_mutex_);
        }
        msg_received_ = false;
        msg_mutex_.unlock();

        // read the result
        ConnectorOut* out = dynamic_cast<ConnectorOut*>(in_sub->getSource());
        ConnectionType::Ptr msg = out->getMessage();
        result->value.push_back(msg);

        // allow sending the next message
        in_sub->free();
    }

    // publish the result
    output_->setType(result);
    output_->publish(result);
}

void Foreach::checkIO()
{
    if(isEnabled()) {
        enableInput(canReceive());

        bool eo = canReceive() && in_sub->isConnected();
        enableOutput(eo);
    } else {
        enableInput(false);
        enableOutput(false);
    }
}



void Foreach::messageProcessed()
{
    QMutexLocker lock(&msg_mutex_);
    apex_assert_hard(!msg_received_);
    msg_received_ = true;
    msg_received_cond_.wakeAll();
}

void Foreach::stop()
{
    QObject::disconnect(in_sub);

    in_sub->stop();
    out_sub->stop();
    disconnectConnector(in_sub);
    disconnectConnector(out_sub);

    Node::stop();
}
