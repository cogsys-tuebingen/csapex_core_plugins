#ifndef VECTOR_MESSAGE_H
#define VECTOR_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <string>
#include <boost/static_assert.hpp>
#include <vector>

namespace csapex {
namespace connection_types {

struct VectorMessage : public Message
{
    typedef std::shared_ptr<VectorMessage> Ptr;
    typedef std::shared_ptr<const VectorMessage> ConstPtr;

    VectorMessage(const std::string& frame_id = "/", Stamp stamp_micro_seconds = 0);

    ConnectionType::Ptr getSubType() const;

    template <typename T>
    static void registerType()
    {
        // not used for this type
    }


    template <typename T>
    static VectorMessage::Ptr make()
    {
        return VectorMessage::Ptr (new VectorMessage(connection_types::makeEmpty<T>(), "/", 0));
    }

    static VectorMessage::Ptr make(ConnectionType::Ptr type)
    {
        return VectorMessage::Ptr (new VectorMessage(type->toType(), "/", 0));
    }

    static VectorMessage::Ptr make();

    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;

    virtual bool canConnectTo(const ConnectionType* other_side) const override;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const override;


    bool isContainer() const override;
    ConnectionType::Ptr nestedType() const override;
    virtual void addNestedValue(const ConnectionType::ConstPtr &msg) override;
    virtual ConnectionType::ConstPtr nestedValue(std::size_t i) const override;
    virtual std::size_t nestedValueCount() const override;

private:
    VectorMessage(ConnectionType::Ptr type, const std::string& frame_id, Stamp stamp_micro_seconds);

public:
    std::vector<ConnectionType::ConstPtr> value;

private:
    ConnectionType::Ptr type_;

};

template <>
struct type<VectorMessage> {
    static std::string name() {
        return "MessageVector";
    }
};


struct GenericVectorMessage : public Message
{
private:

    struct EntryInterface : public Message
    {
        typedef std::shared_ptr< EntryInterface > Ptr;

        EntryInterface(const std::string& name, Message::Stamp stamp = 0)
            : Message(name, "/", stamp)
        {
        }

        virtual ConnectionType::Ptr clone() const override
        {
            return cloneEntry();
        }

        virtual EntryInterface::Ptr cloneEntry() const = 0;

        virtual void encode(YAML::Node& node) const = 0;
        virtual void decode(const YAML::Node& node) = 0;
    };

    template <typename T>
    struct Implementation : public EntryInterface
    {
    private:
        typedef Implementation<T> Self;

    public:
        typedef std::shared_ptr< Self > Ptr;


    public:
        static Self::Ptr make() {
            return Self::Ptr (new Self);
        }

        Implementation()
            : EntryInterface(std::string("std::vector<") + type2nameWithoutNamespace(typeid(T)) + ">")
        {
            BOOST_STATIC_ASSERT((!boost::is_same<T, void*>::value));
        }

        virtual EntryInterface::Ptr cloneEntry() const override
        {
            Self::Ptr r(new Self);
            r->value = value;
            return r;
        }

        virtual ConnectionType::Ptr toType() const override
        {
            Self::Ptr r(new Self);
            return r;
        }

        virtual bool canConnectTo(const ConnectionType* other_side) const override
        {
            const Self* vec = dynamic_cast<const Self*> (other_side);
            const VectorMessage* vec_deprecated =  dynamic_cast<const VectorMessage*> (other_side);
            if(vec_deprecated) {
                return false;
            }
            if(vec != 0) {
                return true;
            } else {
                const GenericVectorMessage* vec = dynamic_cast<const GenericVectorMessage*> (other_side);
                if(vec != 0) {
                    return vec->canConnectTo(this);
                } else {
                    return dynamic_cast<const AnyMessage*> (other_side) != nullptr;
                }
            }
        }
        virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const override
        {
            const Self* vec = dynamic_cast<const Self*> (other_side);
            return vec != 0;
        }

        void encode(YAML::Node& node) const override
        {
            node["value_type"] = type2name(typeid(T));
            node["values"] = *value;
        }

        void decode(const YAML::Node& node) override
        {
            value.reset(new std::vector<T>);
            *value = node["values"].as< std::vector<T> >();
        }

    public:
        std::shared_ptr< std::vector<T> > value;
    };

    template <typename T>
    struct MessageImplementation : public Implementation<T>
    {
        typedef Implementation<T> Parent;
        typedef MessageImplementation<T> Self;

        using Parent::value;

    public:
        typedef std::shared_ptr< Self > Ptr;

        static Self::Ptr make() {
            return Self::Ptr (new Self);
        }
    };

public:
    typedef std::shared_ptr<GenericVectorMessage> Ptr;
    typedef std::shared_ptr<const GenericVectorMessage> ConstPtr;

    template <typename T>
    struct TypeMap {
        typedef std::vector<T> type;
        typedef std::shared_ptr<type> Ptr;
        typedef std::shared_ptr<type const> ConstPtr;
    };

    struct SupportedTypes {
        static SupportedTypes& instance() {
            static SupportedTypes i;
            return i;
        }
        static EntryInterface::Ptr make(const std::string& type) {
            return instance().map_.at(type)->cloneEntry();
        }

        template <typename T>
        static void registerType()
        {
            std::string type = type2name(typeid(T));
            std::map<std::string, EntryInterface::Ptr>& map = instance().map_;
            if(map.find(type) == map.end()) {
                map[type].reset(new Implementation<T>());
            }
        }

    private:
        std::map<std::string, EntryInterface::Ptr> map_;
    };

    template <typename T>
    static void registerType()
    {
        SupportedTypes::registerType<T>();
    }

    template <typename T>
    static GenericVectorMessage::Ptr make(typename std::enable_if<std::is_base_of<ConnectionType, T>::value >::type* dummy = 0)
    {
        registerType<T>();
        return GenericVectorMessage::Ptr(new GenericVectorMessage(MessageImplementation<T>::make(), "/", 0));
    }

    template <typename T>
    static GenericVectorMessage::Ptr make(typename std::enable_if<!std::is_base_of<ConnectionType, T>::value >::type* dummy = 0)
    {
        registerType<T>();
        return GenericVectorMessage::Ptr(new GenericVectorMessage(Implementation<T>::make(), "/", 0));
    }

    template <typename T>
    std::shared_ptr<std::vector<T> const>  makeShared() const
    {
        return std::dynamic_pointer_cast< Implementation<T> > (impl)->value;
    }


    template <typename T>
    void set(const std::shared_ptr< std::vector<T> > & v) {
        std::dynamic_pointer_cast< Implementation<T> > (impl)->value = v;
    }

    void encode(YAML::Node& node) const
    {
        impl->encode(node);
    }

    void decode(const YAML::Node& node)
    {
        std::string type = node["value_type"].as<std::string>();
        impl = SupportedTypes::make(type);
        assert(impl);
        impl->decode(node);
    }


    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;

    virtual bool canConnectTo(const ConnectionType* other_side) const override;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const override;

    virtual std::string descriptiveName() const override;

private:
    GenericVectorMessage(EntryInterface::Ptr impl, const std::string &frame_id, Message::Stamp stamp_micro_seconds);

private:
    EntryInterface::Ptr impl;

};

template <>
struct type<GenericVectorMessage> {
    static std::string name() {
        return "Vector";
    }
};

template <>
inline std::shared_ptr<GenericVectorMessage> makeEmpty<GenericVectorMessage>()
{
    return GenericVectorMessage::make<int>();
}

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::GenericVectorMessage> {
    static Node encode(const csapex::connection_types::GenericVectorMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::GenericVectorMessage& rhs);
};
template<>
struct convert<csapex::connection_types::VectorMessage> {
    static Node encode(const csapex::connection_types::VectorMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::VectorMessage& rhs);
};
}

#endif // VECTOR_MESSAGE_H
