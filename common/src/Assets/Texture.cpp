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

#include "Texture.h"

#include "Assets/TextureBuffer.h"
#include "Assets/TextureCollection.h"
#include "Macros.h"
#include "Renderer/GL.h"

#include "kdl/overload.h"
#include "kdl/reflection_impl.h"
#include "kdl/vector_utils.h"

#include "vm/vec_io.h"

#include <algorithm> // for std::max
#include <cassert>
#include <ostream>

namespace TrenchBroom::Assets
{

std::ostream& operator<<(std::ostream& lhs, const TextureCulling& rhs)
{
  switch (rhs)
  {
  case TextureCulling::Default:
    lhs << "Default";
    break;
  case TextureCulling::None:
    lhs << "None";
    break;
  case TextureCulling::Front:
    lhs << "Front";
    break;
  case TextureCulling::Back:
    lhs << "Back";
    break;
  case TextureCulling::Both:
    lhs << "Both";
    break;
    switchDefault();
  }
  return lhs;
}

kdl_reflect_impl(TextureBlendFunc);

std::ostream& operator<<(std::ostream& lhs, const TextureBlendFunc::Enable& rhs)
{
  switch (rhs)
  {
  case TextureBlendFunc::Enable::UseDefault:
    lhs << "UseDefault";
    break;
  case TextureBlendFunc::Enable::UseFactors:
    lhs << "UseFactors";
    break;
  case TextureBlendFunc::Enable::DisableBlend:
    lhs << "DisableBlend";
    break;
    switchDefault();
  }
  return lhs;
}

kdl_reflect_impl(Texture);

Texture::Texture(std::string name, TextureImage image)
  : m_name{std::move(name)}
  , m_image{std::move(image)}
  , m_usageCount{0u}
  , m_culling{TextureCulling::Default}
  , m_blendFunc{
      TextureBlendFunc::Enable::UseDefault, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA}
{
}

Texture::~Texture() = default;

Texture::Texture(Texture&& other)
  : m_name{std::move(other.m_name)}
  , m_absolutePath{std::move(other.m_absolutePath)}
  , m_relativePath{std::move(other.m_relativePath)}
  , m_image{std::move(other.m_image)}
  , m_usageCount{static_cast<size_t>(other.m_usageCount)}
  , m_surfaceParms{std::move(other.m_surfaceParms)}
  , m_culling{std::move(other.m_culling)}
  , m_blendFunc{std::move(other.m_blendFunc)}
{
}

Texture& Texture::operator=(Texture&& other)
{
  m_name = std::move(other.m_name);
  m_absolutePath = std::move(other.m_absolutePath);
  m_relativePath = std::move(other.m_relativePath);
  m_image = std::move(other.m_image);
  m_usageCount = static_cast<size_t>(other.m_usageCount);
  m_surfaceParms = std::move(other.m_surfaceParms);
  m_culling = std::move(other.m_culling);
  m_blendFunc = std::move(other.m_blendFunc);
  return *this;
}

const std::string& Texture::name() const
{
  return m_name;
}

const std::filesystem::path& Texture::absolutePath() const
{
  return m_absolutePath;
}

void Texture::setAbsolutePath(std::filesystem::path absolutePath)
{
  m_absolutePath = std::move(absolutePath);
}

const std::filesystem::path& Texture::relativePath() const
{
  return m_relativePath;
}

void Texture::setRelativePath(std::filesystem::path relativePath)
{
  m_relativePath = std::move(relativePath);
}

const TextureImage& Texture::image() const
{
  return m_image;
}

TextureImage& Texture::image()
{
  return const_cast<TextureImage&>(const_cast<const Texture*>(this)->image());
}

void Texture::setOpaque()
{
  // TODO: this is really a material property, not a texture property
  m_image.setMask(TextureMask::Off);
}

const std::set<std::string>& Texture::surfaceParms() const
{
  return m_surfaceParms;
}

void Texture::setSurfaceParms(std::set<std::string> surfaceParms)
{
  m_surfaceParms = std::move(surfaceParms);
}

TextureCulling Texture::culling() const
{
  return m_culling;
}

void Texture::setCulling(const TextureCulling culling)
{
  m_culling = culling;
}

void Texture::setBlendFunc(const GLenum srcFactor, const GLenum destFactor)
{
  m_blendFunc.enable = TextureBlendFunc::Enable::UseFactors;
  m_blendFunc.srcFactor = srcFactor;
  m_blendFunc.destFactor = destFactor;
}

void Texture::disableBlend()
{
  m_blendFunc.enable = TextureBlendFunc::Enable::DisableBlend;
}

size_t Texture::usageCount() const
{
  return static_cast<size_t>(m_usageCount);
}

void Texture::incUsageCount()
{
  ++m_usageCount;
}

void Texture::decUsageCount()
{
  const size_t previous = m_usageCount--;
  assert(previous > 0);
  unused(previous);
}

void Texture::activate() const
{
  if (m_image.activate())
  {
    switch (m_culling)
    {
    case Assets::TextureCulling::None:
      glAssert(glDisable(GL_CULL_FACE));
      break;
    case Assets::TextureCulling::Front:
      glAssert(glCullFace(GL_FRONT));
      break;
    case Assets::TextureCulling::Both:
      glAssert(glCullFace(GL_FRONT_AND_BACK));
      break;
    case Assets::TextureCulling::Default:
    case Assets::TextureCulling::Back:
      break;
    }

    if (m_blendFunc.enable != TextureBlendFunc::Enable::UseDefault)
    {
      glAssert(glPushAttrib(GL_COLOR_BUFFER_BIT));
      if (m_blendFunc.enable == TextureBlendFunc::Enable::UseFactors)
      {
        glAssert(glBlendFunc(m_blendFunc.srcFactor, m_blendFunc.destFactor));
      }
      else
      {
        assert(m_blendFunc.enable == TextureBlendFunc::Enable::DisableBlend);
        glAssert(glDisable(GL_BLEND));
      }
    }
  }
}

void Texture::deactivate() const
{
  if (m_image.deactivate())
  {
    if (m_blendFunc.enable != TextureBlendFunc::Enable::UseDefault)
    {
      glAssert(glPopAttrib());
    }

    switch (m_culling)
    {
    case Assets::TextureCulling::None:
      glAssert(glEnable(GL_CULL_FACE));
      break;
    case Assets::TextureCulling::Front:
      glAssert(glCullFace(GL_BACK));
      break;
    case Assets::TextureCulling::Both:
      glAssert(glCullFace(GL_BACK));
      break;
    case Assets::TextureCulling::Default:
    case Assets::TextureCulling::Back:
      break;
    }

    glAssert(glBindTexture(GL_TEXTURE_2D, 0));
  }
}

} // namespace TrenchBroom::Assets
