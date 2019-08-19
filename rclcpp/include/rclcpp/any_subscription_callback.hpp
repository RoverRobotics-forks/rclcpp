// Copyright 2019 Dan Rose
// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
#define RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_

#include <rmw/types.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
template<typename MessageT>
class AbstractSubscriptionCallback
{
protected:
  // Argument type if callback is taking ownership of the object
  using MessageMoveArg = MessageT &&;
  // Most general argument type if callback only needs the message temporarily
  using MessagePeekArg = const MessageT &;
  // Most general argument type if callback needs a copy of the message
  // note that this can be implicitly promoted to a shared_ptr if the callback needs
  using MessageCopyArg = std::unique_ptr<MessageT>;
  // Most general argument type if callback can share the message with other subscribers or
  // publishers.
  using MessageShareArg = const std::shared_ptr<const MessageT> &;

public:
  using UniquePtr = std::unique_ptr<AbstractSubscriptionCallback<MessageT>>;

  virtual bool use_take_shared_method() const = 0;
  virtual void dispatch(MessagePeekArg, const rmw_message_info_t &) const = 0;
  virtual void dispatch_shared(MessageShareArg, const rmw_message_info_t &) const = 0;
  virtual void dispatch_exclusive(MessageT &&, const rmw_message_info_t &) const = 0;
  virtual rcl_allocator_t get_rcl_allocator() const = 0;
};

template<typename MessageT>
using AnySubscriptionCallback = std::reference_wrapper<AbstractSubscriptionCallback<MessageT>>;

template<typename MessageT, typename CallbackT, typename Alloc = std::allocator<void>>
class SubscriptionCallback : public AbstractSubscriptionCallback<MessageT>
{
protected:
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;

  template <typename ArgT>
  using Callback = std::function<void(ArgT)>;

  template <typename ArgT>
  using CallbackWithInfo = std::function<void(ArgT, const rmw_message_info_t &)>;

protected:
  CallbackT callback;
  MessageAlloc message_allocator;
  MessageDeleter message_deleter;

public:
  explicit SubscriptionCallback(CallbackT callback, const Alloc && allocator)
  : callback(callback), message_allocator(allocator)
  {
    allocator::set_allocator_for_deleter(&message_deleter, &message_allocator);
  }

  virtual void dispatch(const MessageT & msg, const rmw_message_info_t & info) const override
  {
    dispatch_impl(callback, msg, info);
  }

  virtual void dispatch_intra_process(MessageShareArg msg, const rmw_message_info_t &msg_info) const {
    dispatch_shared_impl(callback,msg_info);
  }
  virtual void dispatch_intra_process(MessageShareArg msg, const rmw_message_info_t &msg_info) const {
    dispatch_shared_impl(callback, msg_info);
  }
  virtual rcl_allocator_t get_rcl_allocator() const override
  {
    return rclcpp::allocator::get_rcl_allocator<MessageT>(message_allocator);
  }
  virtual bool use_take_shared_method() const override
  {
    return (std::is_constructible<Callback<MessageShareArg>, CallbackT>::value) ||
           (std::is_constructible<CallbackWithInfo<MessageShareArg>, CallbackT>::value);
  }

protected:
  void dispatch_impl(Callback<MessagePeekArg> f, MessagePeekArg msg, rmw_message_info_t const &)
  {
    f(msg);
  }

  void dispatch_impl(
    CallbackWithInfo<MessagePeekArg> f, MessagePeekArg msg, rmw_message_info_t const & info)
  {
    f(msg, info);
  }

  void dispatch_impl(Callback<MessageCopyArg> f, MessagePeekArg msg, const rmw_message_info_t &)
  {
    auto ptr = MessageAllocTraits::allocate(message_allocator, 1);
    MessageAllocTraits::construct(message_allocator, ptr, msg);
    f(std::unique_ptr<MessageT>(msg, message_deleter));
  }

  void dispatch_impl(
    CallbackWithInfo<MessageCopyArg> f, MessagePeekArg msg, const rmw_message_info_t & info)
  {
    auto ptr = MessageAllocTraits::allocate(*message_allocator, 1);
    MessageAllocTraits::construct(message_allocator, ptr, msg);
    f(std::unique_ptr<MessageT>(msg, message_deleter), info);
  }

  void dispatch_shared_impl(
    Callback<MessageShareArg> fn, MessageShareArg msg, rmw_message_info_t const &)
  {
    fn(msg);
  }

  void dispatch_shared_impl(
    CallbackWithInfo<MessageShareArg> fn, MessageShareArg msg, rmw_message_info_t const & info)
  {
    fn(msg, info);
  }

  template<
    typename T_ = CallbackT,
    typename = std::enable_if<
      !(std::is_constructible<Callback<MessageShareArg>, T_>::value ||
      std::is_constructible<CallbackWithInfo<MessageShareArg>, T_>::value)>>
  void dispatch_shared_impl(T_ && t, MessageShareArg msg, rmw_message_info_t const & info)
  {
    MessagePeekArg msg_peek = *msg;
    dispatch_impl(std::forward(t), msg_peek, info);
  }
};

template<typename CallbackT, typename MessageT, typename Alloc = std::allocator<MessageT>>
std::unique_ptr<AbstractSubscriptionCallback<MessageT>> make_subscription_callback(CallbackT && cb, const Alloc && alloc = {})
{
  return std::make_unique<SubscriptionCallback<MessageT, Alloc, CallbackT>>(
    std::forward(cb), std::forward(alloc));
}
}  // namespace rclcpp

#endif  // RCLCPP__ANY_SUBSCRIPTION_CALLBACK_HPP_
