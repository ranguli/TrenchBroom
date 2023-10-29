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

#pragma once

#include "FloatType.h"
#include "Result.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace TrenchBroom::Model
{
class EntityNode;
class GroupNode;
class Node;

using UpdateLinkedGroupsResult =
  std::vector<std::pair<Node*, std::vector<std::unique_ptr<Node>>>>;

/**
 * Updates the given target group nodes from the given source group node.
 *
 * The children of the source node are cloned (recursively) and transformed into the
 * target nodes by means of the recorded transformations of the source group and the
 * corresponding target groups.
 *
 * Depending on the protected property keys of the cloned entities and their corresponding
 * entities in the target groups, some entity property changes may not be propagated from
 * the source group to the target groups. Specifically, if an entity property is protected
 * in either the cloned entity or its corresponding entity in a target group, then changes
 * to that entity property incl. removal are not propagated. This also applies to numbered
 * properties, i.e. properties whose names end in a number. So if the entity property
 * "target" is protected, then changes to the property "target2" are not propagated or
 * overwritten during propagation.
 *
 * If this operation fails for any child and target group, then an error is returned. The
 * operation can fail if any of the following conditions arises:
 *
 * - the transformation of the source group node is not invertible
 * - transforming any of the source node's children fails
 * - any of the transformed children is no longer within the world bounds
 *
 * If this operation succeeds, a vector of pairs is returned where each pair consists of
 * the target node that should be updated, and the new children that should replace the
 * target node's children.
 */
Result<UpdateLinkedGroupsResult> updateLinkedGroups(
  const GroupNode& sourceGroupNode,
  const std::vector<Model::GroupNode*>& targetGroupNodes,
  const vm::bbox3& worldBounds);

/**
 * Generate unique IDs for every node in the given link set. For each of the given
 * groups, every node at the same position in the node tree gets the same ID. Consider
 * the following example where we pass two linked groups A and B with identical
 * structures:
 *
 * GroupNode A             GroupNode B
 * - EntityNode A1         - EntityNode B1
 * - BrushNode A2          - BrushNode B2
 * - GroupNode A3          - GroupNode B3
 *   - BrushNode A3_1        - BrushNode B3_1
 *   - EntityNode A3_2       - EntityNode B3_2
 *
 * Assuming that A and B have the same link ID, then the returned map will contain the
 * following data:
 * - A1: unique_id_1
 * - B1: unique_id_1
 * - A2: unique_id_2
 * - B2: unique_id_2
 * - A3_1: unique_id_3
 * - B3_1: unique_id_3
 * - A3_2: unique_id_4
 * - B3_2: unique_id_4
 *
 * Note how the nodes at the same positions in the subtrees such as A1 and B1 received
 * the same ID, and nodes at different positions, such as A1 and A3_2 received
 * different IDs.
 *
 * The function returns nullopt if any of the passed groups' subtrees have a different
 * structure. The function requires that the given vector contains at least two group
 * nodes, and that all top level group nodes in the given vector have the same link ID.
 */
Result<std::unordered_map<Node*, std::string>> generateLinkIds(
  const std::vector<GroupNode*>& groupNodes);

Result<void> initializeLinkIds(const std::vector<GroupNode*>& groupNodes);

} // namespace TrenchBroom::Model
