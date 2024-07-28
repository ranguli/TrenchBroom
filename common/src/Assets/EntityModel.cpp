/*
 Copyright (C) 2010-2017 Kristian Duske

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

#include "EntityModel.h"

#include "Assets/MaterialCollection.h"
#include "Assets/Texture.h"
#include "Renderer/IndexRangeMap.h"
#include "Renderer/MaterialIndexRangeMap.h"
#include "Renderer/MaterialIndexRangeRenderer.h"
#include "Renderer/PrimType.h"
#include "octree.h"

#include "kdl/reflection_impl.h"
#include "kdl/vector_utils.h"

#include "vm/bbox.h"
#include "vm/forward.h"
#include "vm/intersection.h"

#include <fmt/format.h>

#include <ranges>
#include <string>

namespace TrenchBroom::Assets
{

std::ostream& operator<<(std::ostream& lhs, const PitchType rhs)
{
  switch (rhs)
  {
  case PitchType::Normal:
    lhs << "Normal";
    break;
  case PitchType::MdlInverted:
    lhs << "MdlInverted";
    break;
  }
  return lhs;
}

std::ostream& operator<<(std::ostream& lhs, const Orientation rhs)
{
  switch (rhs)
  {
  case Orientation::ViewPlaneParallelUpright:
    lhs << "ViewPlaneParallelUpright";
    break;
  case Orientation::FacingUpright:
    lhs << "FacingUpright";
    break;
  case Orientation::ViewPlaneParallel:
    lhs << "ViewPlaneParallel";
    break;
  case Orientation::Oriented:
    lhs << "Oriented";
    break;
  case Orientation::ViewPlaneParallelOriented:
    lhs << "ViewPlaneParallelOriented";
    break;
  }
  return lhs;
}


// EntityModelFrame

kdl_reflect_impl(EntityModelFrame);

EntityModelFrame::EntityModelFrame(const size_t index)
  : m_index{index}
  , m_skinOffset{0}
{
}

EntityModelFrame::~EntityModelFrame() = default;

size_t EntityModelFrame::index() const
{
  return m_index;
}

size_t EntityModelFrame::skinOffset() const
{
  return m_skinOffset;
}

void EntityModelFrame::setSkinOffset(const size_t skinOffset)
{
  m_skinOffset = skinOffset;
}

// EntityModelLoadedFrame

kdl_reflect_impl(EntityModelLoadedFrame);

EntityModelLoadedFrame::EntityModelLoadedFrame(
  const size_t index,
  std::string name,
  const vm::bbox3f& bounds,
  const PitchType pitchType,
  const Orientation orientation)
  : EntityModelFrame{index}
  , m_name{std::move(name)}
  , m_bounds{bounds}
  , m_pitchType{pitchType}
  , m_orientation{orientation}
  , m_spacialTree{std::make_unique<SpacialTree>(16.0f)}
{
}

EntityModelLoadedFrame::~EntityModelLoadedFrame() = default;

bool EntityModelLoadedFrame::loaded() const
{
  return true;
}

const std::string& EntityModelLoadedFrame::name() const
{
  return m_name;
}

const vm::bbox3f& EntityModelLoadedFrame::bounds() const
{
  return m_bounds;
}

PitchType EntityModelLoadedFrame::pitchType() const
{
  return m_pitchType;
}

Orientation EntityModelLoadedFrame::orientation() const
{
  return m_orientation;
}

std::optional<float> EntityModelLoadedFrame::intersect(const vm::ray3f& ray) const
{
  auto closestDistance = std::optional<float>{};

  const auto candidates = m_spacialTree->find_intersectors(ray);
  for (const auto triNum : candidates)
  {
    const auto& p1 = m_tris[triNum * 3 + 0];
    const auto& p2 = m_tris[triNum * 3 + 1];
    const auto& p3 = m_tris[triNum * 3 + 2];
    closestDistance =
      vm::safe_min(closestDistance, vm::intersect_ray_triangle(ray, p1, p2, p3));
  }

  return closestDistance;
}

void EntityModelLoadedFrame::addToSpacialTree(
  const std::vector<EntityModelVertex>& vertices,
  const Renderer::PrimType primType,
  const size_t index,
  const size_t count)
{
  switch (primType)
  {
  case Renderer::PrimType::Points:
  case Renderer::PrimType::Lines:
  case Renderer::PrimType::LineStrip:
  case Renderer::PrimType::LineLoop:
    break;
  case Renderer::PrimType::Triangles: {
    assert(count % 3 == 0);
    m_tris.reserve(m_tris.size() + count);
    for (size_t i = 0; i < count; i += 3)
    {
      auto bounds = vm::bbox3f::builder{};
      const auto& p1 = Renderer::getVertexComponent<0>(vertices[index + i + 0]);
      const auto& p2 = Renderer::getVertexComponent<0>(vertices[index + i + 1]);
      const auto& p3 = Renderer::getVertexComponent<0>(vertices[index + i + 2]);
      bounds.add(p1);
      bounds.add(p2);
      bounds.add(p3);

      const auto triIndex = m_tris.size() / 3u;
      m_tris.push_back(p1);
      m_tris.push_back(p2);
      m_tris.push_back(p3);
      m_spacialTree->insert(bounds.bounds(), triIndex);
    }
    break;
  }
  case Renderer::PrimType::Polygon:
  case Renderer::PrimType::TriangleFan: {
    assert(count > 2);
    m_tris.reserve(m_tris.size() + (count - 2) * 3);

    const auto& p1 = Renderer::getVertexComponent<0>(vertices[index]);
    for (size_t i = 1; i < count - 1; ++i)
    {
      auto bounds = vm::bbox3f::builder{};
      const auto& p2 = Renderer::getVertexComponent<0>(vertices[index + i]);
      const auto& p3 = Renderer::getVertexComponent<0>(vertices[index + i + 1]);
      bounds.add(p1);
      bounds.add(p2);
      bounds.add(p3);

      const auto triIndex = m_tris.size() / 3u;
      m_tris.push_back(p1);
      m_tris.push_back(p2);
      m_tris.push_back(p3);
      m_spacialTree->insert(bounds.bounds(), triIndex);
    }
    break;
  }
  case Renderer::PrimType::Quads:
  case Renderer::PrimType::QuadStrip:
  case Renderer::PrimType::TriangleStrip: {
    assert(count > 2);
    m_tris.reserve(m_tris.size() + (count - 2) * 3);
    for (size_t i = 0; i < count - 2; ++i)
    {
      auto bounds = vm::bbox3f::builder{};
      const auto& p1 = Renderer::getVertexComponent<0>(vertices[index + i + 0]);
      const auto& p2 = Renderer::getVertexComponent<0>(vertices[index + i + 1]);
      const auto& p3 = Renderer::getVertexComponent<0>(vertices[index + i + 2]);
      bounds.add(p1);
      bounds.add(p2);
      bounds.add(p3);

      const auto triIndex = m_tris.size() / 3u;
      if (i % 2 == 0)
      {
        m_tris.push_back(p1);
        m_tris.push_back(p2);
        m_tris.push_back(p3);
      }
      else
      {
        m_tris.push_back(p1);
        m_tris.push_back(p3);
        m_tris.push_back(p2);
      }
      m_spacialTree->insert(bounds.bounds(), triIndex);
    }
    break;
  }
    switchDefault();
  }
}

// EntityModel::UnloadedFrame

/**
 * A frame of the model in its unloaded state.
 */
class EntityModelUnloadedFrame : public EntityModelFrame
{
  kdl_reflect_inline_empty(EntityModelUnloadedFrame);

public:
  /**
   * Creates a new frame with the given index.
   *
   * @param index the index of this frame
   */
  explicit EntityModelUnloadedFrame(const size_t index)
    : EntityModelFrame{index}
  {
  }

  bool loaded() const override { return false; }

  const std::string& name() const override
  {
    static const std::string name = "Unloaded frame";
    return name;
  }

  const vm::bbox3f& bounds() const override
  {
    static const auto bounds = vm::bbox3f(8.0f);
    return bounds;
  }

  PitchType pitchType() const override { return PitchType::Normal; }

  Orientation orientation() const override { return Orientation::Oriented; }

  std::optional<float> intersect(const vm::ray3f&) const override { return std::nullopt; }
};

// EntityModel::Mesh

/**
 * The mesh associated with a frame and a surface.
 */
class EntityModelMesh
{
protected:
  std::vector<EntityModelVertex> m_vertices;

  kdl_reflect_inline_empty(EntityModelMesh);

  /**
   * Creates a new frame mesh that uses the given vertices.
   *
   * @param vertices the vertices
   */
  explicit EntityModelMesh(std::vector<EntityModelVertex> vertices)
    : m_vertices{std::move(vertices)}
  {
  }

public:
  virtual ~EntityModelMesh() = default;

public:
  /**
   * Returns a renderer that renders this mesh with the given material.
   *
   * @param skin the material to use when rendering the mesh
   * @return the renderer
   */
  std::unique_ptr<Renderer::MaterialIndexRangeRenderer> buildRenderer(
    const Material* skin)
  {
    const auto vertexArray = Renderer::VertexArray::ref(m_vertices);
    return doBuildRenderer(skin, vertexArray);
  }

private:
  /**
   * Creates and returns the actual mesh renderer
   *
   * @param skin the skin to use when rendering the mesh
   * @param vertices the vertices associated with this mesh
   * @return the renderer
   */
  virtual std::unique_ptr<Renderer::MaterialIndexRangeRenderer> doBuildRenderer(
    const Material* skin, const Renderer::VertexArray& vertices) = 0;
};

// EntityModel::IndexedMesh

namespace
{

/**
 * A model frame mesh for indexed rendering. Stores vertices and vertex indices.
 */
class EntityModelIndexedMesh : public EntityModelMesh
{
private:
  Renderer::IndexRangeMap m_indices;

  kdl_reflect_inline_empty(EntityModelIndexedMesh);

public:
  /**
   * Creates a new frame mesh with the given vertices and indices.
   *
   * @param frame the frame to which this mesh belongs
   * @param vertices the vertices
   * @param indices the indices
   */
  EntityModelIndexedMesh(
    EntityModelLoadedFrame& frame,
    std::vector<EntityModelVertex> vertices,
    Renderer::IndexRangeMap indices)
    : EntityModelMesh{std::move(vertices)}
    , m_indices{std::move(indices)}
  {
    m_indices.forEachPrimitive(
      [&](const Renderer::PrimType primType, const size_t index, const size_t count) {
        frame.addToSpacialTree(m_vertices, primType, index, count);
      });
  }

private:
  std::unique_ptr<Renderer::MaterialIndexRangeRenderer> doBuildRenderer(
    const Material* skin, const Renderer::VertexArray& vertices) override
  {
    const Renderer::MaterialIndexRangeMap indices(skin, m_indices);
    return std::make_unique<Renderer::MaterialIndexRangeRenderer>(vertices, indices);
  }
};

// EntityModelMaterialMesh

/**
 * A model frame mesh for per material indexed rendering. Stores vertices and per material
 * indices.
 */
class EntityModelMaterialMesh : public EntityModelMesh
{
private:
  Renderer::MaterialIndexRangeMap m_indices;

  kdl_reflect_inline_empty(EntityModelMaterialMesh);

public:
  /**
   * Creates a new frame mesh with the given vertices and per material indices.
   *
   * @param frame the frame to which this mesh belongs
   * @param vertices the vertices
   * @param indices the per material indices
   */
  EntityModelMaterialMesh(
    EntityModelLoadedFrame& frame,
    std::vector<EntityModelVertex> vertices,
    Renderer::MaterialIndexRangeMap indices)
    : EntityModelMesh{std::move(vertices)}
    , m_indices{std::move(indices)}
  {
    m_indices.forEachPrimitive([&](
                                 const Material* /* material */,
                                 const Renderer::PrimType primType,
                                 const size_t index,
                                 const size_t count) {
      frame.addToSpacialTree(m_vertices, primType, index, count);
    });
  }

private:
  std::unique_ptr<Renderer::MaterialIndexRangeRenderer> doBuildRenderer(
    const Material* /* skin */, const Renderer::VertexArray& vertices) override
  {
    return std::make_unique<Renderer::MaterialIndexRangeRenderer>(vertices, m_indices);
  }
};

} // namespace

// EntityModelSurface

kdl_reflect_impl(EntityModelSurface);

EntityModelSurface::EntityModelSurface(std::string name, const size_t frameCount)
  : m_name{std::move(name)}
  , m_meshes{frameCount}
  , m_skins{std::make_unique<MaterialCollection>()}
{
}

EntityModelSurface::~EntityModelSurface() = default;

const std::string& EntityModelSurface::name() const
{
  return m_name;
}

void EntityModelSurface::prepare(const int minFilter, const int magFilter)
{
  for (auto& material : m_skins->materials())
  {
    if (auto* texture = material.texture())
    {
      texture->upload(true);
      texture->setFilterMode(minFilter, magFilter);
    }
  }
}

void EntityModelSurface::setFilterMode(const int minFilter, const int magFilter)
{
  m_skins->setFilterMode(minFilter, magFilter);
}

void EntityModelSurface::addMesh(
  EntityModelLoadedFrame& frame,
  std::vector<EntityModelVertex> vertices,
  Renderer::IndexRangeMap indices)
{
  assert(frame.index() < frameCount());
  m_meshes[frame.index()] = std::make_unique<EntityModelIndexedMesh>(
    frame, std::move(vertices), std::move(indices));
}

void EntityModelSurface::addMesh(
  EntityModelLoadedFrame& frame,
  std::vector<EntityModelVertex> vertices,
  Renderer::MaterialIndexRangeMap indices)
{
  assert(frame.index() < frameCount());
  m_meshes[frame.index()] = std::make_unique<EntityModelMaterialMesh>(
    frame, std::move(vertices), std::move(indices));
}

void EntityModelSurface::setSkins(std::vector<Material> skins)
{
  m_skins = std::make_unique<MaterialCollection>(std::move(skins));
}

size_t EntityModelSurface::frameCount() const
{
  return m_meshes.size();
}

size_t EntityModelSurface::skinCount() const
{
  return m_skins->materialCount();
}

const Material* EntityModelSurface::skin(const std::string& name) const
{
  return m_skins->materialByName(name);
}

const Material* EntityModelSurface::skin(const size_t index) const
{
  return m_skins->materialByIndex(index);
}

std::unique_ptr<Renderer::MaterialIndexRangeRenderer> EntityModelSurface::buildRenderer(
  const size_t skinIndex, const size_t frameIndex)
{
  assert(frameIndex < frameCount());
  assert(skinIndex < skinCount());

  return m_meshes[frameIndex] ? m_meshes[frameIndex]->buildRenderer(skin(skinIndex))
                              : nullptr;
}

// EntityModel

kdl_reflect_impl(EntityModel);

EntityModel::EntityModel(
  std::string name, const PitchType pitchType, const Orientation orientation)
  : m_name{std::move(name)}
  , m_pitchType{pitchType}
  , m_orientation{orientation}
{
}

const std::string& EntityModel::name() const
{
  return m_name;
}

std::unique_ptr<Renderer::MaterialRenderer> EntityModel::buildRenderer(
  const size_t skinIndex, const size_t frameIndex) const
{
  auto renderers = std::vector<std::unique_ptr<Renderer::MaterialIndexRangeRenderer>>{};
  if (frameIndex >= frameCount())
  {
    return nullptr;
  }

  const auto& frame = this->frame(frameIndex);
  const auto actualSkinIndex = skinIndex + frame->skinOffset();
  for (const auto& surface : m_surfaces)
  {
    // If an out of range skin is requested, use the first skin as a fallback
    const auto correctedSkinIndex =
      actualSkinIndex < surface->skinCount() ? actualSkinIndex : 0;
    if (auto renderer = surface->buildRenderer(correctedSkinIndex, frameIndex))
    {
      renderers.push_back(std::move(renderer));
    }
  }
  return !renderers.empty() ? std::make_unique<Renderer::MultiMaterialIndexRangeRenderer>(
           std::move(renderers))
                            : nullptr;
}

vm::bbox3f EntityModel::bounds(const size_t frameIndex) const
{
  return frameIndex < m_frames.size() ? m_frames[frameIndex]->bounds() : vm::bbox3f{8.0f};
}

bool EntityModel::prepared() const
{
  return m_prepared;
}

void EntityModel::prepare(const int minFilter, const int magFilter)
{
  if (!m_prepared)
  {
    for (auto& surface : m_surfaces)
    {
      surface->prepare(minFilter, magFilter);
    }
    m_prepared = true;
  }
}

void EntityModel::setFilterMode(const int minFilter, const int magFilter)
{
  for (auto& surface : m_surfaces)
  {
    surface->setFilterMode(minFilter, magFilter);
  }
}

EntityModelFrame& EntityModel::addFrame()
{
  m_frames.push_back(std::make_unique<EntityModelUnloadedFrame>(frameCount()));
  return *m_frames.back();
}

EntityModelLoadedFrame& EntityModel::loadFrame(
  const size_t frameIndex, std::string name, const vm::bbox3f& bounds)
{
  if (frameIndex >= frameCount())
  {
    throw AssetException{fmt::format(
      "Frame index {} is out of bounds (frame count = {})", frameIndex, frameCount())};
  }

  auto frame = std::make_unique<EntityModelLoadedFrame>(
    frameIndex, std::move(name), bounds, m_pitchType, m_orientation);
  frame->setSkinOffset(m_frames[frameIndex]->skinOffset());

  auto& result = *frame;
  m_frames[frameIndex] = std::move(frame);
  return result;
}

EntityModelSurface& EntityModel::addSurface(std::string name)
{
  m_surfaces.push_back(
    std::make_unique<EntityModelSurface>(std::move(name), frameCount()));
  return *m_surfaces.back();
}

size_t EntityModel::frameCount() const
{
  return m_frames.size();
}

size_t EntityModel::surfaceCount() const
{
  return m_surfaces.size();
}

std::vector<const EntityModelFrame*> EntityModel::frames() const
{
  return kdl::vec_transform(m_frames, [](const auto& frame) {
    return const_cast<const EntityModelFrame*>(frame.get());
  });
}

std::vector<EntityModelFrame*> EntityModel::frames()
{
  return kdl::vec_transform(m_frames, [](const auto& frame) { return frame.get(); });
}

std::vector<const EntityModelSurface*> EntityModel::surfaces() const
{
  return kdl::vec_transform(m_surfaces, [](const auto& surface) {
    return const_cast<const EntityModelSurface*>(surface.get());
  });
}

const EntityModelFrame* EntityModel::frame(const std::string& name) const
{
  const auto it = std::ranges::find_if(
    m_frames, [&](const auto& frame) { return frame->name() == name; });
  return it != m_frames.end() ? it->get() : nullptr;
}

const EntityModelFrame* EntityModel::frame(const size_t index) const
{
  return index < frameCount() ? m_frames[index].get() : nullptr;
}

const EntityModelSurface& EntityModel::surface(const size_t index) const
{
  if (index >= surfaceCount())
  {
    throw std::out_of_range{"Surface index is out of bounds"};
  }
  return *m_surfaces[index];
}

EntityModelSurface& EntityModel::surface(const size_t index)
{
  return const_cast<EntityModelSurface&>(
    const_cast<const EntityModel&>(*this).surface(index));
}

const EntityModelSurface* EntityModel::surface(const std::string& name) const
{
  const auto it = std::ranges::find_if(
    m_surfaces, [&](const auto& surface) { return surface->name() == name; });
  return it != m_surfaces.end() ? it->get() : nullptr;
}

} // namespace TrenchBroom::Assets
