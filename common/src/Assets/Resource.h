/*
 Copyright (C) 2024 Kristian Duske

 This file is part of TrenchBroom.

 TrenchBroom is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 TrenchBroom is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with TrenchBroom. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "Error.h"
#include "Result.h"

#include "kdl/overload.h"
#include "kdl/reflection_impl.h"
#include "kdl/result.h"

#include <functional>
#include <future>
#include <iostream>
#include <variant>

namespace TrenchBroom::Assets
{

template <typename T>
using Task = std::function<Result<T>()>;

template <typename T>
using ResourceLoader = std::function<Result<T>()>;

template <typename T>
struct ResourceUnloaded
{
  ResourceLoader<T> loader;

  kdl_reflect_inline_empty(ResourceUnloaded);
};

template <typename T>
struct ResourceLoading
{
  std::future<Result<T>> future;

  kdl_reflect_inline_empty(ResourceLoading);
};

template <typename T>
struct ResourceLoaded
{
  T resource;

  kdl_reflect_inline(ResourceLoaded, resource);
};

template <typename T>
struct ResourceReady
{
  T resource;

  kdl_reflect_inline(ResourceReady, resource);
};

template <typename T>
struct ResourceDropping
{
  T resource;

  kdl_reflect_inline(ResourceDropping, resource);
};

struct ResourceDropped
{
  kdl_reflect_inline_empty(ResourceDropped);
};

struct ResourceFailed
{
  std::string error;

  kdl_reflect_inline(ResourceFailed, error);
};

template <typename T>
using ResourceState = std::variant<
  ResourceUnloaded<T>,
  ResourceLoading<T>,
  ResourceLoaded<T>,
  ResourceReady<T>,
  ResourceDropping<T>,
  ResourceDropped,
  ResourceFailed>;

template <typename T>
std::ostream& operator<<(std::ostream& lhs, const ResourceState<T>& rhs)
{
  std::visit([&](const auto& x) { lhs << x; }, rhs);
  return lhs;
}

namespace detail
{

template <typename T, typename TaskRunner>
ResourceState<T> triggerLoading(ResourceUnloaded<T> state, TaskRunner& taskRunner)
{
  auto future = taskRunner.run(std::move(state.loader));
  return ResourceLoading<T>{std::move(future)};
}

template <typename T>
ResourceState<T> finishLoading(ResourceLoading<T> state)
{
  if (state.future.wait_for(std::chrono::seconds{0}) == std::future_status::ready)
  {
    if (!state.future.valid())
    {
      return ResourceFailed{"Invalid future"};
    }
    return state.future.get()
      .transform([](auto value) -> ResourceState<T> {
        return ResourceLoaded<T>{std::move(value)};
      })
      .transform_error([](auto error) -> ResourceState<T> {
        return ResourceFailed{std::move(error.msg)};
      })
      .value();
  }
  return std::move(state);
}

template <typename T>
ResourceState<T> upload(ResourceLoaded<T> state)
{
  state.resource.upload();
  return ResourceReady<T>{std::move(state.resource)};
}

template <typename T>
ResourceState<T> triggerDropping(ResourceReady<T> state)
{
  return ResourceDropping<T>{std::move(state.resource)};
}

template <typename T>
ResourceState<T> drop(ResourceDropping<T> state)
{
  state.resource.drop();
  return ResourceDropped{};
}

} // namespace detail

/**
 * A resource that can be loaded, uploaded, and dropped.
 *
 * The following table shows the state transitions of a resource:
 *
 * | State          | Transition       | New state       |
 * |----------------|------------------|-----------------|
 * | Unloaded       | process          | Loading         |
 * | Loading        | process          | Loaded or Failed|
 * | Loaded         | process          | Ready           |
 * | Ready          | drop             | Dropping        |
 * | Dropping       | process          | Dropped         |
 * | Dropped        | -                | -               |
 * | Failed         | -                | -               |
 */
template <typename T>
class Resource
{
private:
  ResourceState<T> m_state;

  kdl_reflect_inline(Resource, m_state);

public:
  explicit Resource(ResourceLoader<T> loader)
    : m_state(ResourceUnloaded<T>{std::move(loader)})
  {
  }

  explicit Resource(T resource)
    : m_state(ResourceLoaded<T>{std::move(resource)})
  {
  }

  const ResourceState<T>& state() const { return m_state; }

  const T* get() const
  {
    return std::visit(
      kdl::overload(
        [](const ResourceLoaded<T>& state) -> const T* { return &state.resource; },
        [](const ResourceReady<T>& state) -> const T* { return &state.resource; },
        [](const auto&) -> const T* { return nullptr; }),
      m_state);
  }

  T* get()
  {
    return std::visit(
      kdl::overload(
        [](ResourceLoaded<T>& state) -> T* { return &state.resource; },
        [](ResourceReady<T>& state) -> T* { return &state.resource; },
        [](auto&) -> T* { return nullptr; }),
      m_state);
  }

  bool isDropped() const { return std::holds_alternative<ResourceDropped>(m_state); }

  bool needsProcessing() const
  {
    return !std::holds_alternative<ResourceReady<T>>(m_state)
           && !std::holds_alternative<ResourceFailed>(m_state);
  }

  template <typename TaskRunner>
  void process(TaskRunner& taskRunner)
  {
    m_state = std::visit(
      kdl::overload(
        [&](ResourceUnloaded<T> state) -> ResourceState<T> {
          return detail::triggerLoading(std::move(state), taskRunner);
        },
        [&](ResourceLoading<T> state) -> ResourceState<T> {
          return detail::finishLoading(std::move(state));
        },
        [&](ResourceLoaded<T> state) -> ResourceState<T> {
          return detail::upload(std::move(state));
        },
        [&](ResourceDropping<T> state) -> ResourceState<T> {
          return detail::drop(std::move(state));
        },
        [](auto state) -> ResourceState<T> { return std::move(state); }),
      std::move(m_state));
  }

  void load()
  {
    m_state = std::visit(
      kdl::overload(
        [&](ResourceUnloaded<T> state) -> ResourceState<T> {
          return state.loader()
            .transform([](auto value) -> ResourceState<T> {
              return ResourceLoaded<T>{std::move(value)};
            })
            .transform_error([](auto error) -> ResourceState<T> {
              return ResourceFailed{std::move(error.msg)};
            })
            .value();
        },
        [](auto state) -> ResourceState<T> { return std::move(state); }),
      std::move(m_state));
  }

  void upload()
  {
    m_state = std::visit(
      kdl::overload(
        [&](ResourceLoaded<T> state) -> ResourceState<T> {
          return detail::upload(std::move(state));
        },
        [](auto state) -> ResourceState<T> { return std::move(state); }),
      std::move(m_state));
  }

  void drop()
  {
    m_state = std::visit(
      kdl::overload(
        [](ResourceLoaded<T>) -> ResourceState<T> { return ResourceDropped{}; },
        [](ResourceReady<T> state) -> ResourceState<T> {
          return detail::triggerDropping(std::move(state));
        },
        [&](ResourceDropping<T> state) -> ResourceState<T> { return std::move(state); },
        [](auto) -> ResourceState<T> { return ResourceDropped{}; }),
      std::move(m_state));
  }
};

template <typename T>
Resource(T) -> Resource<T>;

template <typename T>
auto makeResource(T resource)
{
  return std::make_shared<Resource<T>>(std::move(resource));
}

} // namespace TrenchBroom::Assets
