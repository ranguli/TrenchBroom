/*
 Copyright (C) 2023 Kristian Duske

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

#include "Assets/Resource.h"
#include "Assets/ResourceManager.h"
#include "Error.h"
#include "Result.h"

#include "kdl/vector_utils.h"

#include "Catch2.h"

namespace TrenchBroom::Assets
{
namespace
{

struct MockResource
{
  void upload() const { mockUpload(); }
  void drop() const { mockDrop(); };

  std::function<void()> mockUpload = []() {};
  std::function<void()> mockDrop = []() {};
};

template <typename T>
struct MockTaskRunner
{
  auto run(Task<T> task)
  {
    auto promise = std::promise<Result<T>>{};
    auto future = promise.get_future();
    tasks.push_back({std::move(promise), std::move(task)});
    return future;
  }

  void resolveNextPromise()
  {
    auto [promise, task] = kdl::vec_pop_front(tasks);
    promise.set_value(task());
  }

  void resolveLastPromise()
  {
    auto [promise, task] = kdl::vec_pop_back(tasks);
    promise.set_value(task());
  }

  std::vector<std::tuple<std::promise<Result<T>>, Task<T>>> tasks;
};

using ResourceT = Resource<MockResource>;
using ResourceWrapperT = ResourceWrapper<MockResource, MockTaskRunner<MockResource>>;
using ResourceWrapperBaseT = ResourceWrapperBase<MockTaskRunner<MockResource>>;

bool operator==(
  const std::vector<const ResourceWrapperBaseT*>& lhs,
  const std::vector<std::shared_ptr<ResourceT>>& rhs)
{
  if (lhs.size() != rhs.size())
  {
    return false;
  }

  for (size_t i = 0; i < lhs.size(); ++i)
  {
    const auto* lhsCast = dynamic_cast<const ResourceWrapperT*>(lhs[i]);

    if (!lhsCast || *lhsCast != ResourceWrapperT{rhs[i]})
    {
      return false;
    }
  }

  return true;
}

} // namespace

TEST_CASE("ResourceManager")
{
  const auto mockResourceLoader = [&]() { return Result<MockResource>{MockResource{}}; };

  auto resourceManager = ResourceManager<MockTaskRunner<MockResource>>{};
  auto mockTaskRunner = MockTaskRunner<MockResource>{};

  SECTION("needsProcessing")
  {
    CHECK(!resourceManager.needsProcessing());

    auto resource1 = std::make_shared<ResourceT>(mockResourceLoader);
    resourceManager.addResource(resource1);

    REQUIRE(std::holds_alternative<ResourceUnloaded<MockResource>>(resource1->state()));
    CHECK(resourceManager.needsProcessing());

    resourceManager.process(mockTaskRunner);
    REQUIRE(std::holds_alternative<ResourceLoading<MockResource>>(resource1->state()));
    CHECK(resourceManager.needsProcessing());

    mockTaskRunner.resolveNextPromise();
    resourceManager.process(mockTaskRunner);
    REQUIRE(std::holds_alternative<ResourceLoaded<MockResource>>(resource1->state()));
    CHECK(resourceManager.needsProcessing());

    resourceManager.process(mockTaskRunner);
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
    CHECK(!resourceManager.needsProcessing());

    auto resource2 = std::make_shared<ResourceT>(mockResourceLoader);
    resourceManager.addResource(resource2);
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
    REQUIRE(std::holds_alternative<ResourceUnloaded<MockResource>>(resource2->state()));
    CHECK(resourceManager.needsProcessing());

    resourceManager.process(mockTaskRunner);
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
    REQUIRE(std::holds_alternative<ResourceLoading<MockResource>>(resource2->state()));
    CHECK(resourceManager.needsProcessing());

    mockTaskRunner.resolveNextPromise();
    resourceManager.process(mockTaskRunner);
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
    REQUIRE(std::holds_alternative<ResourceLoaded<MockResource>>(resource2->state()));
    CHECK(resourceManager.needsProcessing());

    resourceManager.process(mockTaskRunner);
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource2->state()));
    CHECK(!resourceManager.needsProcessing());

    resource1.reset();
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource2->state()));
    CHECK(resourceManager.needsProcessing());

    resourceManager.process(mockTaskRunner);
    REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource2->state()));
    CHECK(!resourceManager.needsProcessing());

    resource2.reset();
    CHECK(resourceManager.needsProcessing());

    resourceManager.process(mockTaskRunner);
    CHECK(!resourceManager.needsProcessing());
  }

  SECTION("addResource")
  {
    auto resource1 = std::make_shared<ResourceT>(mockResourceLoader);
    resourceManager.addResource(resource1);

    CHECK(resourceManager.resources() == std::vector{resource1});
    CHECK(resource1.use_count() == 2);
    CHECK(std::holds_alternative<ResourceUnloaded<MockResource>>(resource1->state()));

    auto resource2 = std::make_shared<ResourceT>(mockResourceLoader);
    resourceManager.addResource(resource2);

    CHECK(resourceManager.resources() == std::vector{resource1, resource2});
  }

  SECTION("process")
  {
    SECTION("resource loading")
    {
      auto resource1 = std::make_shared<ResourceT>(mockResourceLoader);
      auto resource2 = std::make_shared<ResourceT>(mockResourceLoader);
      resourceManager.addResource(resource1);
      resourceManager.addResource(resource2);

      resourceManager.process(mockTaskRunner);
      CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource1->state()));
      CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource2->state()));

      SECTION("resource1 finishes loading")
      {
        mockTaskRunner.resolveNextPromise();

        resourceManager.process(mockTaskRunner);
        CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource1->state()));
        CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource2->state()));

        SECTION("resource2 finishes loading")
        {
          mockTaskRunner.resolveNextPromise();

          resourceManager.process(mockTaskRunner);
          CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
          CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource2->state()));

          resourceManager.process(mockTaskRunner);
          CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
          CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource2->state()));
        }
      }

      SECTION("resource2 finishes loading")
      {
        mockTaskRunner.resolveLastPromise();

        resourceManager.process(mockTaskRunner);
        CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource1->state()));
        CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource2->state()));

        SECTION("resource1 finishes loading")
        {
          mockTaskRunner.resolveLastPromise();

          resourceManager.process(mockTaskRunner);
          CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource1->state()));
          CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource2->state()));

          resourceManager.process(mockTaskRunner);
          CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource1->state()));
          CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource2->state()));
        }
      }
    }

    SECTION("dropping resources")
    {
      auto mockDropCalled = std::array{false, false};
      auto sharedResources = std::array{
        std::make_shared<ResourceT>([&]() {
          return Result<MockResource>{MockResource{
            []() {},
            [&]() { mockDropCalled[0] = true; },
          }};
        }),
        std::make_shared<ResourceT>([&]() {
          return Result<MockResource>{MockResource{
            []() {},
            [&]() { mockDropCalled[1] = true; },
          }};
        }),
      };

      resourceManager.addResource(sharedResources[0]);
      resourceManager.addResource(sharedResources[1]);

      resourceManager.process(mockTaskRunner);
      mockTaskRunner.resolveNextPromise();
      mockTaskRunner.resolveNextPromise();
      resourceManager.process(mockTaskRunner);
      resourceManager.process(mockTaskRunner);
      REQUIRE(
        std::holds_alternative<ResourceReady<MockResource>>(sharedResources[0]->state()));
      REQUIRE(
        std::holds_alternative<ResourceReady<MockResource>>(sharedResources[1]->state()));

      sharedResources[0].reset();
      CHECK(resourceManager.resources().size() == 2);

      resourceManager.process(mockTaskRunner);
      CHECK(resourceManager.resources() == std::vector{sharedResources[1]});
      CHECK(mockDropCalled[0]);

      sharedResources[1].reset();
      CHECK(resourceManager.resources().size() == 1);

      resourceManager.process(mockTaskRunner);
      CHECK(resourceManager.resources().empty());
      CHECK(mockDropCalled[1]);
    }
  }
}

} // namespace TrenchBroom::Assets
