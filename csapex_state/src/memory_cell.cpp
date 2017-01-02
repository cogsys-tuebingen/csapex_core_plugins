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

        void clear()
        {
            if(message) {
                message.reset();
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
        : changed_(false)
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

        parameters.addParameter(param::ParameterFactory::declareTrigger("export"), [this](param::Parameter*) {
            std::unique_lock<std::recursive_mutex> lock(value_mutex_);
            if(value_) {
                TokenPtr token = std::make_shared<Token>(value_);
                lock.unlock();

                publish_event_->triggerWith(token);
            } else {
                lock.unlock();

                empty_event_->trigger();
            }
        });
        parameters.addParameter(param::ParameterFactory::declareTrigger("clear"), [this](param::Parameter*) {
            if(!name_.empty()) {
                getCell(name_).clear();
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
        clear_event_ = modifier.addEvent("cleared");
        publish_event_ = modifier.addEvent("published");
        empty_event_ = modifier.addEvent("empty");
    }

    bool canProcess() const
    {
        return changed_;
    }

    void process()
    {
        if(changed_cell_) {
            TokenPtr token;
            {
                std::unique_lock<std::recursive_mutex> lock(value_mutex_);
                value_ = changed_cell_;
                token = std::make_shared<Token>(value_->clone());
            }
            change_event_->triggerWith(token);

            if(show_content_) {
                YAML::Node node;
                node = MessageSerializer::serializeMessage(*changed_cell_);

                std::stringstream ss;
                convert(ss, node, "");
                setParameter("text", ss.str());
            }

        } else {
            {
                std::unique_lock<std::recursive_mutex> lock(value_mutex_);
                value_.reset();
            }
            clear_event_->trigger();

            if(show_content_) {
                setParameter("text", std::string(""));
            }
        }

        changed_cell_.reset();
        changed_ = false;
    }

private:
    void updateValue(const connection_types::MessageConstPtr& new_val) {
        changed_cell_ = new_val;
        changed_ = true;
        yield();
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
    bool changed_;
    connection_types::MessageConstPtr  changed_cell_;

    Event* change_event_;
    Event* clear_event_;
    Event* publish_event_;
    Event* empty_event_;
    slim_signal::ScopedConnection connection_;

    std::recursive_mutex value_mutex_;
    connection_types::MessageConstPtr value_;
};

}
}
CSAPEX_REGISTER_CLASS(csapex::state::MemoryCell, csapex::Node)
