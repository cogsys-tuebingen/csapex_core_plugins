/// HEADER
#include <csapex/model/node.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/any_message.h>
#include <csapex/signal/event.h>
#include <csapex/model/token.h>
#include <csapex/serialization/message_serializer.h>

namespace csapex
{
namespace state
{

class MemoryCell : public Node
{
private:
    class Cell {
    public:
        void setMessage(const connection_types::MessageConstPtr msg)
        {
            apex_assert(msg);
            if(msg != message) {
                message = msg;
                changed(message);
            }
        }

    public:
        slim_signal::Signal<void (connection_types::MessageConstPtr)> changed;

    private:
        connection_types::MessageConstPtr message;
    };


public:
    MemoryCell()
    {
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareText("name", ""), [this](param::Parameter* p)
        {
            std::string name = p->as<std::string>();
            if(name != name_) {
                name_ = name;

                Cell& c = getCell(name_);
                connection_ = c.changed.connect([this](connection_types::MessageConstPtr new_val) {
                       updateValue(new_val);
            });
            }
        });

        parameters.addParameter(param::ParameterFactory::declareBool("show_content", false), show_content_);
        parameters.addConditionalParameter(param::ParameterFactory::declareOutputText("text"), show_content_);
    }

    void setup(NodeModifier& modifier) override
    {
        modifier.addTypedSlot<csapex::connection_types::AnyMessage>("set", [this](const TokenPtr& token)
        {
            if(!name_.empty()) {
                auto message = std::dynamic_pointer_cast<connection_types::Message const>(token->getTokenData());

                getCell(name_).setMessage(message);
            }
        });

        change_event_ = modifier.addEvent("changed");
    }

    void process()
    {
    }

private:
    void updateValue(const connection_types::MessageConstPtr& new_val) {
        TokenPtr token = std::make_shared<Token>(new_val);
        change_event_->triggerWith(token);


        if(show_content_) {
            YAML::Node node;
            node = MessageSerializer::serializeMessage(*new_val);

            std::stringstream ss;
            convert(ss, node, "");
            setParameter("text", ss.str());
        }
    }

    void convert(std::stringstream &ss, const YAML::Node &node, const std::string& prefix)
    {
        static const std::string PREFIX = "   ";

        if(node.IsMap()) {
            std::vector<std::string> keys;
            for(YAML::Node::const_iterator it = node.begin(); it != node.end(); ++it) {
                keys.push_back(it->first.as<std::string>());
            }

            std::sort(keys.begin(), keys.end());

            for(std::vector<std::string>::iterator key = keys.begin(); key != keys.end(); ++key) {
                ss << prefix << *key << ": \n";
                convert(ss, node[*key], prefix + PREFIX);
                ss << "\n";
            }

        } else if(node.IsSequence()) {
            std::size_t total = node.size();
            std::size_t max_count = 16;

            for(std::size_t i = 0, n = std::min(max_count, total); i < n; ++i) {
                convert(ss, node[i], prefix + "|");
                ss << "\n";
            }
            if(max_count < total) {
                ss << "...\n";
            }

        } else if(node.Type() != YAML::NodeType::Null) {
            ss << prefix << node.as<std::string>().substr(0, 1000);
        }
    }

    static Cell& getCell(const std::string& name)
    {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);

        static std::map<std::string, Cell> cache;
        return cache[name];
    }

private:
    std::string name_;

    bool show_content_;

    Event* change_event_;
    slim_signal::ScopedConnection connection_;
};

}
}
CSAPEX_REGISTER_CLASS(csapex::state::MemoryCell, csapex::Node)
