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

#pragma once

#include "Assets/EntityModel.h"
#include "Assets/ModelSpecification.h"
#include "Result.h"

#include <filesystem>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace TrenchBroom
{
class Logger;

namespace Model
{
class EntityNode;
class Game;
} // namespace Model

namespace Renderer
{
class MaterialRenderer;
class VboManager;
} // namespace Renderer

namespace Assets
{
class EntityModelFrame;
enum class Orientation;
class Quake3Shader;

class EntityModelManager
{
private:
  Logger& m_logger;

  int m_minFilter;
  int m_magFilter;
  bool m_resetFilterMode = false;
  const Model::Game* m_game = nullptr;

  // Cache Quake 3 shaders to use when loading models
  std::vector<Quake3Shader> m_shaders;

  mutable std::unordered_map<std::filesystem::path, EntityModel> m_models;
  mutable std::unordered_set<std::filesystem::path> m_modelMismatches;
  mutable std::
    unordered_map<ModelSpecification, std::unique_ptr<Renderer::MaterialRenderer>>
      m_renderers;
  mutable std::unordered_set<ModelSpecification> m_rendererMismatches;

  mutable std::vector<EntityModel*> m_unpreparedModels;
  mutable std::vector<Renderer::MaterialRenderer*> m_unpreparedRenderers;

public:
  EntityModelManager(int magFilter, int minFilter, Logger& logger);
  ~EntityModelManager();

  void clear();
  void reloadShaders();

  void setFilterMode(int minFilter, int magFilter);
  void setGame(const Model::Game* game);

  Renderer::MaterialRenderer* renderer(const ModelSpecification& spec) const;

  const EntityModelFrame* frame(const ModelSpecification& spec) const;

private:
  EntityModel* model(const std::filesystem::path& path) const;
  EntityModel* safeGetModel(const std::filesystem::path& path) const;
  Result<EntityModel> loadModel(const std::filesystem::path& path) const;

public:
  void prepare(Renderer::VboManager& vboManager);

private:
  void resetFilterMode();
  void prepareModels();
  void prepareRenderers(Renderer::VboManager& vboManager);
};
} // namespace Assets
} // namespace TrenchBroom
