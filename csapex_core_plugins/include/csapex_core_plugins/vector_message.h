#ifndef VECTOR_MESSAGE_H
#define VECTOR_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <string>

namespace csapex {
namespace connection_types {

struct VectorMessage : public Message
{
    typedef boost::shared_ptr<VectorMessage> Ptr;

    VectorMessage(const std::string& frame_id = "/");

    ConnectionType::Ptr getSubType() const;


    template <typename T>
    static VectorMessage::Ptr make()
    {
        return VectorMessage::Ptr (new VectorMessage(T::make(), "/"));
    }

    static VectorMessage::Ptr make(ConnectionType::Ptr type)
    {
        return VectorMessage::Ptr (new VectorMessage(type->toType(), "/"));
    }

    static VectorMessage::Ptr make();

    virtual ConnectionType::Ptr clone();
    virtual ConnectionType::Ptr toType();

    virtual bool canConnectTo(const ConnectionType* other_side) const;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const;

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);

private:
    VectorMessage(ConnectionType::Ptr type, const std::string& frame_id);

public:
    std::vector<ConnectionType::Ptr> value;

private:
    ConnectionType::Ptr type_;

};


struct GenericVectorMessage : public Message
{
private:
    template <typename T>
    struct Implementation : public Message
    {
    private:
        typedef Implementation<T> Self;

    public:
        typedef boost::shared_ptr< Self > Ptr;


    public:
        static Self::Ptr make() {
            return Self::Ptr (new Self);
        }

        Implementation()
            : Message(std::string("std::vector<") + type2nameWithoutNamespace(typeid(T)) + ">", "/")
        {
        }

        virtual ConnectionType::Ptr clone()
        {
            Self::Ptr r(new Self);
            r->value = value;
            return r;
        }

        virtual ConnectionType::Ptr toType()
        {
            Self::Ptr r(new Self);
            return r;
        }

        virtual bool canConnectTo(const ConnectionType* other_side) const
        {
            const Self* vec = dynamic_cast<const Self*> (other_side);
            if(vec != 0) {
                return true;
            } else {
                return other_side->canConnectTo(this);
            }
        }
        virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const
        {

            const Self* vec = dynamic_cast<const Self*> (other_side);
            return vec != 0;
        }

        void writeYaml(YAML::Emitter& yaml) const
        {
            yaml << YAML::Key << "generic vector" << YAML::Value << "not writable";
        }

        void readYaml(const YAML::Node& node)
        {
            throw std::runtime_error("deserialization of generic vectors not implemented");
        }

    public:
        boost::shared_ptr< std::vector<T> > value;
    };

    template <typename T>
    struct MessageImplementation : public Implementation<T>
    {
        typedef Implementation<T> Parent;
        typedef MessageImplementation<T> Self;

        using Parent::value;

    public:
        typedef boost::shared_ptr< Self > Ptr;

        static Self::Ptr make() {
            return Self::Ptr (new Self);
        }

        void writeYaml(YAML::Emitter& yaml) const
        {
            yaml << YAML::Key << "generic vector" << YAML::Value << YAML::Flow;
            yaml << YAML::BeginSeq;
            for(unsigned i = 0; i < value->size(); ++i) {
                const T& entry = (*value)[i];
                yaml << YAML::BeginMap;
                entry.writeYaml(yaml);
                yaml << YAML::EndMap;
            }
            yaml << YAML::EndSeq;
        }

        void readYaml(const YAML::Node& node)
        {

        }
    };

public:
    typedef boost::shared_ptr<GenericVectorMessage> Ptr;

    template <typename T>
    struct TypeMap {
        typedef std::vector<T> type;
        typedef boost::shared_ptr<type> Ptr;
        typedef boost::shared_ptr<type const> ConstPtr;
    };

    template <typename T>
    static GenericVectorMessage::Ptr make(typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0)
    {
        return GenericVectorMessage::Ptr(new GenericVectorMessage(MessageImplementation<T>::make(), "/"));
    }

    template <typename T>
    static GenericVectorMessage::Ptr make(typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0)
    {
        return GenericVectorMessage::Ptr(new GenericVectorMessage(Implementation<T>::make(), "/"));
    }

    template <typename T>
    boost::shared_ptr<std::vector<T> const>  makeShared()
    {
        return boost::dynamic_pointer_cast< Implementation<T> > (impl)->value;
    }


    template <typename T>
    void set(const boost::shared_ptr< std::vector<T> > & v) {
        boost::dynamic_pointer_cast< Implementation<T> > (impl)->value = v;
    }


    virtual ConnectionType::Ptr clone();
    virtual ConnectionType::Ptr toType();

    virtual bool canConnectTo(const ConnectionType* other_side) const;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const;

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);

private:
    GenericVectorMessage(ConnectionType::Ptr impl, const std::string &frame_id);

private:
    ConnectionType::Ptr impl;

};

}
}

#endif // VECTOR_MESSAGE_H
