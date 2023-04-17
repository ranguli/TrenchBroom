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

#include "PakFileSystem.h"

#include "IO/DiskFileSystem.h"
#include "IO/File.h"
#include "IO/Path.h"
#include "IO/ReaderException.h"
#include "Macros.h"

#include <kdl/string_format.h>

#include <cassert>
#include <cstring>
#include <memory>

namespace TrenchBroom
{
namespace IO
{
namespace PakLayout
{
static const size_t HeaderMagicLength = 0x4;
static const size_t IdEntryLength = 0x40;
static const size_t DkEntryLength = 0x48;
static const size_t EntryNameLength = 0x38;
static const std::string HeaderMagic = "PACK";
} // namespace PakLayout

namespace
{

struct PakEntry
{
  size_t offset;
  size_t size;
};

[[maybe_unused]] bool operator<(const PakEntry& lhs, const PakEntry& rhs)
{
  return lhs.offset < rhs.offset;
}

bool operator==(const PakEntry& lhs, const PakEntry& rhs)
{
  return lhs.offset == rhs.offset && lhs.size == rhs.size;
}

bool operator!=(const PakEntry& lhs, const PakEntry& rhs)
{
  return !(lhs == rhs);
}

PakEntry readPakEntry(Reader& reader, const PakType asType)
{
  const auto entryName = reader.readString(PakLayout::EntryNameLength);
  const auto entryAddress = reader.readSize<int32_t>();

  if (asType == PakType::Id)
  {
    const auto entrySize = reader.readSize<int32_t>();
    return {entryAddress, entrySize};
  }

  const auto uncompressedSize = reader.readSize<int32_t>();
  const auto compressedSize = reader.readSize<int32_t>();
  const auto compressed = reader.readBool<int32_t>();
  const auto entrySize = compressed ? compressedSize : uncompressedSize;

  return {entryAddress, entrySize};
}

// A directory entry is valid if it doesn't exceed the pak file size and it doesn't
// overlap any other directory entry (duplicate entries are allowed though).
bool isValidDirectoryEntry(
  Reader& reader,
  const PakType asType,
  const size_t pakFileSize,
  std::vector<PakEntry>& entries)
{
  try
  {
    const auto entry = readPakEntry(reader, asType);
    if (entry.offset == 0 || entry.offset + entry.size > pakFileSize)
    {
      return false;
    }

    // find the first entry with offset >= entry.offset
    auto it = std::lower_bound(entries.begin(), entries.end(), entry);
    if (
      (it != entries.begin()
       && std::prev(it)->offset + std::prev(it)->size > entry.offset)
      || (it != entries.end() && *it != entry && it->offset < entry.offset + entry.size))
    {
      // entry overlaps with the previous or next entry
      return false;
    }

    entries.insert(it, entry);
    return true;
  }
  catch (const ReaderException&)
  {
    return false;
  }
}

bool isValidDirectory(
  Reader reader,
  const PakType asType,
  const size_t directoryOffset,
  const size_t directorySize,
  const size_t pakFileSize)
{
  const auto entrySize =
    asType == PakType::Id ? PakLayout::IdEntryLength : PakLayout::DkEntryLength;
  const auto entryCount = directorySize / entrySize;

  auto entries = std::vector<PakEntry>{};
  entries.reserve(entryCount);

  reader.seekFromBegin(directoryOffset);

  for (size_t i = 0; i < entryCount; ++i)
  {
    if (!isValidDirectoryEntry(reader, asType, pakFileSize, entries))
    {
      return false;
    }
  }
  return true;
}

PakType detectPakType(const File& file)
{
  auto reader = file.reader();
  reader.seekFromBegin(PakLayout::HeaderMagicLength);

  const auto directoryOffset = reader.readSize<int32_t>();
  const auto directorySize = reader.readSize<int32_t>();
  if (
    directorySize % PakLayout::IdEntryLength == 0
    && directorySize % PakLayout::DkEntryLength == 0)
  {
    if (isValidDirectory(
          reader, PakType::Id, directoryOffset, directorySize, file.size()))
    {
      return PakType::Id;
    }
    if (isValidDirectory(
          reader, PakType::Daikatana, directoryOffset, directorySize, file.size()))
    {
      return PakType::Daikatana;
    }
  }
  else if (directorySize % PakLayout::IdEntryLength == 0)
  {
    return PakType::Id;
  }
  else if (directorySize % PakLayout::DkEntryLength == 0)
  {
    return PakType::Daikatana;
  }

  return PakType::Unknown;
}

std::unique_ptr<char[]> decompress(
  std::shared_ptr<File> file, const size_t uncompressedSize)
{
  auto reader = file->reader().buffer();

  auto result = std::make_unique<char[]>(uncompressedSize);
  auto* begin = result.get();
  auto* curTarget = begin;

  auto x = reader.readUnsignedChar<unsigned char>();
  while (!reader.eof() && x < 0xFF)
  {
    if (x < 0x40)
    {
      // x+1 bytes of uncompressed data follow (just read+write them as they are)
      const auto len = static_cast<size_t>(x) + 1;
      reader.read(curTarget, len);
      curTarget += len;
    }
    else if (x < 0x80)
    {
      // run-length encoded zeros, write (x - 62) zero-bytes to output
      const auto len = static_cast<size_t>(x) - 62;
      std::memset(curTarget, 0, len);
      curTarget += len;
    }
    else if (x < 0xC0)
    {
      // run-length encoded data, read one byte, write it (x-126) times to output
      const auto len = static_cast<size_t>(x) - 126;
      const auto data = reader.readInt<unsigned char>();
      std::memset(curTarget, data, len);
      curTarget += len;
    }
    else if (x < 0xFE)
    {
      // this references previously uncompressed data
      // read one byte to get _offset_
      // read (x-190) bytes from the already uncompressed and written output data,
      // starting at (offset+2) bytes before the current write position (and add them to
      // output, of course)
      const auto len = static_cast<size_t>(x) - 190;
      const auto offset = reader.readSize<unsigned char>();
      auto* from = curTarget - (offset + 2);

      assert(from >= begin);
      assert(from <= curTarget - len);

      std::memcpy(curTarget, from, len);
      curTarget += len;
    }

    x = reader.readUnsignedChar<unsigned char>();
  }

  return result;
}

} // namespace

std::ostream& operator<<(std::ostream& lhs, const PakType& rhs)
{
  switch (rhs)
  {
  case PakType::Id:
    lhs << "Id";
    break;
  case PakType::Daikatana:
    lhs << "Daikatana";
    break;
  case PakType::Unknown:
    lhs << "Unknown";
    break;
    switchDefault();
  }
  return lhs;
}

PakFileSystem::PakFileSystem(Path path)
  : ImageFileSystem{std::move(path)}
{
  initialize();
}

PakType PakFileSystem::type() const
{
  return m_type;
}

void PakFileSystem::doReadDirectory()
{
  auto reader = m_file->reader();

  const auto magic = reader.readString(PakLayout::HeaderMagicLength);
  if (magic != PakLayout::HeaderMagic)
  {
    throw FileSystemException{"Invalid pak file header"};
  }

  m_type = detectPakType(*m_file);
  switch (m_type)
  {
  case PakType::Id:
    readIdPakDirectory(reader);
    break;
  case PakType::Daikatana:
    readDkPakDirectory(reader);
    break;
  case PakType::Unknown:
    throw FileSystemException{"Cannot detect pak file type"};
  }
}

void PakFileSystem::readIdPakDirectory(Reader& reader)
{
  const auto directoryOffset = reader.readSize<int32_t>();
  const auto directorySize = reader.readSize<int32_t>();
  const auto entryCount = directorySize / PakLayout::IdEntryLength;

  reader.seekFromBegin(directoryOffset);

  for (size_t i = 0; i < entryCount; ++i)
  {
    const auto entryName = reader.readString(PakLayout::EntryNameLength);
    const auto entryAddress = reader.readSize<int32_t>();
    const auto entrySize = reader.readSize<int32_t>();

    const auto entryPath = Path{kdl::str_to_lower(entryName)};
    auto entryFile =
      std::make_shared<FileView>(entryPath, m_file, entryAddress, entrySize);
    addFile(entryPath, [entryFile = std::move(entryFile)]() { return entryFile; });
  }
}

void PakFileSystem::readDkPakDirectory(Reader& reader)
{
  const auto directoryOffset = reader.readSize<int32_t>();
  const auto directorySize = reader.readSize<int32_t>();
  const auto entryCount = directorySize / PakLayout::DkEntryLength;

  reader.seekFromBegin(directoryOffset);

  for (size_t i = 0; i < entryCount; ++i)
  {
    const auto entryName = reader.readString(PakLayout::EntryNameLength);
    const auto entryAddress = reader.readSize<int32_t>();
    const auto uncompressedSize = reader.readSize<int32_t>();
    const auto compressedSize = reader.readSize<int32_t>();
    const auto compressed = reader.readBool<int32_t>();
    const auto entrySize = compressed ? compressedSize : uncompressedSize;

    const auto entryPath = Path(kdl::str_to_lower(entryName));
    auto entryFile =
      std::make_shared<FileView>(entryPath, m_file, entryAddress, entrySize);

    if (compressed)
    {
      addFile(
        entryPath,
        [entryFile = std::move(entryFile), uncompressedSize]() -> std::shared_ptr<File> {
          auto data = decompress(entryFile, uncompressedSize);
          return std::make_shared<OwningBufferFile>(
            entryFile->path(), std::move(data), uncompressedSize);
        });
    }
    else
    {
      addFile(entryPath, [entryFile = std::move(entryFile)]() { return entryFile; });
    }
  }
}
} // namespace IO
} // namespace TrenchBroom
