// typed_payload.h
// Open type-safe value container for records. Any coherent data struct can
// be stored; readers get it back only by naming the exact type. Adding a new
// payload type changes no core code. Every payload carries a stable type
// name; the in-process type_index is for validation only and is never
// serialized.
//
// Payload structs carry data, not framework behavior. This immutable value
// erasure is a separate concept from resource contract erasure.

#pragma once
#include <memory>
#include <string>
#include <typeindex>
#include <utility>

namespace navigatr
{

class TypedPayload
{
public:
    TypedPayload() = default;

    template <typename T>
    static TypedPayload store(T value, std::string stable_name) {
        TypedPayload p;
        p.data_        = std::make_shared<Holder<T>>(std::move(value));
        p.stable_name_ = std::move(stable_name);
        return p;
    }

    // Null when empty or when the stored type is not exactly T.
    template <typename T>
    const T* get() const {
        if (data_ == nullptr || data_->type() != typeid(T)) {
            return nullptr;
        }
        return static_cast<const T*>(data_->ptr());
    }

    bool empty() const { return data_ == nullptr; }

    std::type_index cppType() const {
        return data_ == nullptr ? std::type_index(typeid(void)) : std::type_index(data_->type());
    }

    const std::string& stableTypeName() const { return stable_name_; }

private:
    struct HolderBase {
        virtual ~HolderBase()                      = default;
        virtual const std::type_info& type() const = 0;
        virtual const void*           ptr() const  = 0;
    };

    template <typename T>
    struct Holder : HolderBase {
        explicit Holder(T v) : value(std::move(v)) {}
        const std::type_info& type() const override { return typeid(T); }
        const void*           ptr() const override { return &value; }
        T                     value;
    };

    std::shared_ptr<const HolderBase> data_;
    std::string                       stable_name_;
};

} // namespace navigatr
