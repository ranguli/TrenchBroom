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
#include "Error.h"
#include "Result.h"

#include "kdl/reflection_impl.h"
#include "kdl/result.h"
#include "kdl/vector_utils.h"

#include "Catch2.h"

namespace TrenchBroom::Assets
{
struct MockResource
{
  void upload() const { mockUpload(); }
  void drop() const { mockDrop(); };

  std::function<void()> mockUpload = []() {};
  std::function<void()> mockDrop = []() {};

  kdl_reflect_inline_empty(MockResource);
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

  std::vector<std::tuple<std::promise<Result<T>>, Task<T>>> tasks;
};

using ResourceT = Resource<MockResource>;

template <typename State>
void setResourceState(ResourceT& resource, MockTaskRunner<MockResource>& mockTaskRunner)
{
  static_assert(!std::is_same_v<State, ResourceFailed>);

  REQUIRE(std::holds_alternative<ResourceUnloaded<MockResource>>(resource.state()));
  if (std::holds_alternative<State>(resource.state()))
  {
    return;
  }

  resource.process(mockTaskRunner);
  REQUIRE(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
  if (std::holds_alternative<State>(resource.state()))
  {
    return;
  }

  mockTaskRunner.resolveNextPromise();
  REQUIRE(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
  if (std::holds_alternative<State>(resource.state()))
  {
    return;
  }

  resource.process(mockTaskRunner);
  REQUIRE(std::holds_alternative<ResourceLoaded<MockResource>>(resource.state()));
  if (std::holds_alternative<State>(resource.state()))
  {
    return;
  }

  resource.process(mockTaskRunner);
  REQUIRE(std::holds_alternative<ResourceReady<MockResource>>(resource.state()));
  if (std::holds_alternative<State>(resource.state()))
  {
    return;
  }

  resource.drop();
  REQUIRE(std::holds_alternative<ResourceDropping<MockResource>>(resource.state()));
  if (std::holds_alternative<State>(resource.state()))
  {
    return;
  }

  resource.process(mockTaskRunner);
  REQUIRE(std::holds_alternative<ResourceDropped>(resource.state()));
}

TEST_CASE("Resource")
{
  auto mockTaskRunner = MockTaskRunner<MockResource>{};

  SECTION("Construction with loaded resource")
  {
    auto resource = ResourceT{MockResource{}};

    CHECK(resource.get() != nullptr);
    CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource.state()));
    CHECK(!resource.isDropped());
    CHECK(mockTaskRunner.tasks.empty());
  }

  SECTION("Resource loading fails")
  {
    auto resource =
      ResourceT{[&]() { return Result<MockResource>{Error{"MockResource failed"}}; }};

    SECTION("async")
    {
      setResourceState<ResourceLoading<MockResource>>(resource, mockTaskRunner);
      mockTaskRunner.resolveNextPromise();

      resource.process(mockTaskRunner);
      CHECK(
        resource.state()
        == ResourceState<MockResource>{ResourceFailed{"MockResource failed"}});
    }

    SECTION("sync")
    {
      resource.load();
      CHECK(
        resource.state()
        == ResourceState<MockResource>{ResourceFailed{"MockResource failed"}});
    }
  }

  SECTION("Resource loading succeeds")
  {
    auto mockUploadCalled = false;
    auto mockDropCalled = false;

    auto resource = ResourceT{[&]() {
      return Result<MockResource>{MockResource{
        [&]() { mockUploadCalled = true; },
        [&]() { mockDropCalled = true; },
      }};
    }};

    SECTION("ResourceUnloaded state")
    {
      setResourceState<ResourceUnloaded<MockResource>>(resource, mockTaskRunner);
      REQUIRE(!mockUploadCalled);
      REQUIRE(!mockDropCalled);

      CHECK(resource.get() == nullptr);
      CHECK(!resource.isDropped());
      CHECK(mockTaskRunner.tasks.empty());
      CHECK(!mockUploadCalled);
      CHECK(!mockDropCalled);

      SECTION("process")
      {
        resource.process(mockTaskRunner);
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.size() == 1);
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("load")
      {
        resource.load();
        CHECK(resource.get() != nullptr);
        CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("upload")
      {
        resource.upload();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceUnloaded<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("drop")
      {
        resource.drop();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
        CHECK(resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }
    }

    SECTION("ResourceLoading state")
    {
      setResourceState<ResourceLoading<MockResource>>(resource, mockTaskRunner);
      REQUIRE(!mockUploadCalled);
      REQUIRE(!mockDropCalled);

      SECTION("process")
      {
        SECTION("TaskRunner has not resolved promise")
        {
          resource.process(mockTaskRunner);
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.size() == 1);
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }

        SECTION("TaskRunner resolves promise")
        {
          mockTaskRunner.resolveNextPromise();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);

          resource.process(mockTaskRunner);
          CHECK(resource.get() != nullptr);
          CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }
      }

      SECTION("load")
      {
        SECTION("TaskRunner has not resolved promise")
        {
          resource.load();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.size() == 1);
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);

          mockTaskRunner.resolveNextPromise();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }

        SECTION("TaskRunner resolves promise")
        {
          mockTaskRunner.resolveNextPromise();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);

          resource.load();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }
      }

      SECTION("upload")
      {
        SECTION("TaskRunner has not resolved promise")
        {
          resource.upload();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.size() == 1);
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);

          mockTaskRunner.resolveNextPromise();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }

        SECTION("TaskRunner resolves promise")
        {
          mockTaskRunner.resolveNextPromise();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);

          resource.upload();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }
      }

      SECTION("drop")
      {
        SECTION("TaskRunner has not resolved promise")
        {
          resource.drop();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
          CHECK(resource.isDropped());
          CHECK(mockTaskRunner.tasks.size() == 1);
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);

          mockTaskRunner.resolveNextPromise();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
          CHECK(resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }

        SECTION("TaskRunner resolves promise")
        {
          mockTaskRunner.resolveNextPromise();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceLoading<MockResource>>(resource.state()));
          CHECK(!resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);

          resource.drop();
          CHECK(resource.get() == nullptr);
          CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
          CHECK(resource.isDropped());
          CHECK(mockTaskRunner.tasks.empty());
          CHECK(!mockUploadCalled);
          CHECK(!mockDropCalled);
        }
      }
    }

    SECTION("ResourceLoaded state")
    {
      setResourceState<ResourceLoaded<MockResource>>(resource, mockTaskRunner);
      REQUIRE(!mockUploadCalled);
      REQUIRE(!mockDropCalled);

      SECTION("process")
      {
        resource.process(mockTaskRunner);
        CHECK(resource.get() != nullptr);
        CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("load")
      {
        resource.load();
        CHECK(resource.get() != nullptr);
        CHECK(std::holds_alternative<ResourceLoaded<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("upload")
      {
        resource.upload();
        CHECK(resource.get() != nullptr);
        CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("drop")
      {
        resource.drop();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
        CHECK(resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }
    }

    SECTION("ResourceReady state")
    {
      setResourceState<ResourceReady<MockResource>>(resource, mockTaskRunner);
      REQUIRE(mockUploadCalled);
      REQUIRE(!mockDropCalled);
      mockUploadCalled = false;

      SECTION("process")
      {
        resource.process(mockTaskRunner);
        CHECK(resource.get() != nullptr);
        CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("load")
      {
        resource.load();
        CHECK(resource.get() != nullptr);
        CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("upload")
      {
        resource.upload();
        CHECK(resource.get() != nullptr);
        CHECK(std::holds_alternative<ResourceReady<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("drop")
      {
        resource.drop();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropping<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }
    }

    SECTION("ResourceDropping state")
    {
      setResourceState<ResourceDropping<MockResource>>(resource, mockTaskRunner);
      REQUIRE(mockUploadCalled);
      REQUIRE(!mockDropCalled);
      mockUploadCalled = false;

      SECTION("process")
      {
        resource.process(mockTaskRunner);
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
        CHECK(resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(mockDropCalled);
      }

      SECTION("load")
      {
        resource.load();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropping<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("upload")
      {
        resource.upload();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropping<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("drop")
      {
        resource.drop();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropping<MockResource>>(resource.state()));
        CHECK(!resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }
    }

    SECTION("ResourceDropped state")
    {
      setResourceState<ResourceDropped>(resource, mockTaskRunner);
      REQUIRE(mockUploadCalled);
      REQUIRE(mockDropCalled);
      mockUploadCalled = false;
      mockDropCalled = false;

      SECTION("process")
      {
        resource.process(mockTaskRunner);
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
        CHECK(resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("load")
      {
        resource.load();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
        CHECK(resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("upload")
      {
        resource.upload();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
        CHECK(resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }

      SECTION("drop")
      {
        resource.drop();
        CHECK(resource.get() == nullptr);
        CHECK(std::holds_alternative<ResourceDropped>(resource.state()));
        CHECK(resource.isDropped());
        CHECK(mockTaskRunner.tasks.empty());
        CHECK(!mockUploadCalled);
        CHECK(!mockDropCalled);
      }
    }
  }

  SECTION("needsProcessing")
  {
    SECTION("ResourceFailed state")
    {
      auto resource =
        ResourceT{[&]() { return Result<MockResource>{Error{"MockResource failed"}}; }};

      setResourceState<ResourceLoading<MockResource>>(resource, mockTaskRunner);
      mockTaskRunner.resolveNextPromise();

      resource.process(mockTaskRunner);
      REQUIRE(
        resource.state()
        == ResourceState<MockResource>{ResourceFailed{"MockResource failed"}});
      CHECK(!resource.needsProcessing());
    }

    SECTION("ResourceUnloaded state")
    {
      auto resource = ResourceT{[&]() { return Result<MockResource>{MockResource{}}; }};
      setResourceState<ResourceUnloaded<MockResource>>(resource, mockTaskRunner);
      CHECK(resource.needsProcessing());
    }

    SECTION("ResourceLoading state")
    {
      auto resource = ResourceT{[&]() { return Result<MockResource>{MockResource{}}; }};
      setResourceState<ResourceLoading<MockResource>>(resource, mockTaskRunner);
      CHECK(resource.needsProcessing());
    }

    SECTION("ResourceLoaded state")
    {
      auto resource = ResourceT{[&]() { return Result<MockResource>{MockResource{}}; }};
      setResourceState<ResourceLoaded<MockResource>>(resource, mockTaskRunner);
      CHECK(resource.needsProcessing());
    }

    SECTION("ResourceReady state")
    {
      auto resource = ResourceT{[&]() { return Result<MockResource>{MockResource{}}; }};
      setResourceState<ResourceReady<MockResource>>(resource, mockTaskRunner);
      CHECK(!resource.needsProcessing());
    }

    SECTION("ResourceDropping state")
    {
      auto resource = ResourceT{[&]() { return Result<MockResource>{MockResource{}}; }};
      setResourceState<ResourceDropping<MockResource>>(resource, mockTaskRunner);
      CHECK(resource.needsProcessing());
    }

    SECTION("ResourceDropped state")
    {
      auto resource = ResourceT{[&]() { return Result<MockResource>{MockResource{}}; }};
      setResourceState<ResourceDropped>(resource, mockTaskRunner);
      CHECK(resource.needsProcessing());
    }
  }
}

} // namespace TrenchBroom::Assets
