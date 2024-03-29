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

#include "Assets/Resource.h"

#include "kdl/collection_utils.h"
#include "kdl/reflection_impl.h"
#include "kdl/vector_utils.h"

#include <vector>

namespace TrenchBroom::Assets
{

template <typename TaskRunnerT>
class ResourceWrapperBase
{
public:
  virtual ~ResourceWrapperBase() = default;

  virtual long useCount() const = 0;

  virtual bool isDropped() const = 0;
  virtual bool needsProcessing() const = 0;

  virtual void drop() = 0;
  virtual void process(TaskRunnerT& taskRunner) = 0;
};

template <typename T, typename TaskRunnerT>
class ResourceWrapper : public ResourceWrapperBase<TaskRunnerT>
{
private:
  std::shared_ptr<Resource<T>> m_resource;

  kdl_reflect_inline(ResourceWrapper, m_resource);

public:
  explicit ResourceWrapper(std::shared_ptr<Resource<T>> resource)
    : m_resource{std::move(resource)}
  {
  }

  long useCount() const override { return m_resource.use_count(); }
  bool isDropped() const override { return m_resource->isDropped(); }
  bool needsProcessing() const override { return m_resource->needsProcessing(); }
  void drop() override { m_resource->drop(); }
  void process(TaskRunnerT& taskRunner) override { m_resource->process(taskRunner); }
};

template <typename TaskRunnerT>
class ResourceManager
{
private:
  std::vector<std::unique_ptr<ResourceWrapperBase<TaskRunnerT>>> m_resources;

public:
  bool needsProcessing() const
  {
    return kdl::any_of(m_resources, [](const auto& resourceWrapper) {
      return resourceWrapper->useCount() == 1 || resourceWrapper->needsProcessing();
    });
  }

  std::vector<const ResourceWrapperBase<TaskRunnerT>*> resources() const
  {
    return kdl::vec_transform(m_resources, [](const auto& resourceWrapper) {
      return static_cast<const ResourceWrapperBase<TaskRunnerT>*>(resourceWrapper.get());
    });
  }

  template <typename ResourceT>
  void addResource(std::shared_ptr<Resource<ResourceT>> resource)
  {
    m_resources.push_back(
      std::make_unique<ResourceWrapper<ResourceT, TaskRunnerT>>(std::move(resource)));
  }

  void process(TaskRunnerT& taskRunner)
  {
    for (auto it = m_resources.begin(); it != m_resources.end();)
    {
      auto& resourceWrapper = *it;
      if (resourceWrapper->useCount() == 1 && !resourceWrapper->isDropped())
      {
        resourceWrapper->drop();
      }

      if (resourceWrapper->needsProcessing())
      {
        resourceWrapper->process(taskRunner);
      }

      it = resourceWrapper->useCount() == 1 && resourceWrapper->isDropped()
             ? m_resources.erase(it)
             : std::next(it);
    }
  }
};

} // namespace TrenchBroom::Assets
