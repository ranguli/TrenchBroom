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

#include "Assets/TextureImage.h"
#include "Color.h"
#include "Renderer/GL.h"

#include "kdl/reflection_decl.h"

#include "vm/forward.h"

#include <atomic>
#include <filesystem>
#include <set>
#include <string>
#include <variant>
#include <vector>

namespace TrenchBroom::Assets
{

enum class TextureCulling
{
  Default,
  None,
  Front,
  Back,
  Both
};

std::ostream& operator<<(std::ostream& lhs, const TextureCulling& rhs);

struct TextureBlendFunc
{
  enum class Enable
  {
    /**
     * Don't change GL_BLEND and don't change the blend function.
     */
    UseDefault,
    /**
     * Don't change GL_BLEND, but set the blend function.
     */
    UseFactors,
    /**
     * Set GL_BLEND to off.
     */
    DisableBlend
  };

  Enable enable;
  GLenum srcFactor;
  GLenum destFactor;

  kdl_reflect_decl(TextureBlendFunc, enable, srcFactor, destFactor);
};

std::ostream& operator<<(std::ostream& lhs, const TextureBlendFunc::Enable& rhs);

class Texture
{
private:
  std::string m_name;
  std::filesystem::path m_absolutePath;
  std::filesystem::path m_relativePath;

  TextureImage m_image;

  std::atomic<size_t> m_usageCount;

  // TODO: move these to a Q3Data variant case of m_gameData if possible
  // Quake 3 surface parameters; move these to materials when we add proper support for
  // those.
  std::set<std::string> m_surfaceParms;

  // Quake 3 surface culling; move to materials
  TextureCulling m_culling;

  // Quake 3 blend function, move to materials
  TextureBlendFunc m_blendFunc;

  kdl_reflect_decl(
    Texture,
    m_name,
    m_absolutePath,
    m_relativePath,
    m_image,
    m_usageCount,
    m_surfaceParms,
    m_culling,
    m_blendFunc);

public:
  Texture(std::string name, TextureImage image);

  Texture(const Texture&) = delete;
  Texture& operator=(const Texture&) = delete;

  Texture(Texture&& other);
  Texture& operator=(Texture&& other);

  ~Texture();

  const std::string& name() const;

  /**
   * Absolute path of the texture
   */
  const std::filesystem::path& absolutePath() const;
  void setAbsolutePath(std::filesystem::path absolutePath);

  /**
   * Relative path of the texture in the game filesystem
   */
  const std::filesystem::path& relativePath() const;
  void setRelativePath(std::filesystem::path relativePath);

  const TextureImage& image() const;
  TextureImage& image();

  void setOpaque();

  const std::set<std::string>& surfaceParms() const;
  void setSurfaceParms(std::set<std::string> surfaceParms);

  TextureCulling culling() const;
  void setCulling(TextureCulling culling);

  void setBlendFunc(GLenum srcFactor, GLenum destFactor);
  void disableBlend();

  size_t usageCount() const;
  void incUsageCount();
  void decUsageCount();

  void activate() const;
  void deactivate() const;
};

} // namespace TrenchBroom::Assets
