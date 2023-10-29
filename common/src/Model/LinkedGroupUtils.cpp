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

#include "LinkedGroupUtils.h"

#include "Ensure.h"
#include "Error.h"
#include "Model/BrushNode.h"
#include "Model/EntityNode.h"
#include "Model/GroupNode.h"
#include "Model/LayerNode.h"
#include "Model/Node.h"
#include "Model/NodeContents.h"
#include "Model/PatchNode.h"
#include "Model/WorldNode.h"
#include "Uuid.h"

#include <kdl/overload.h>
#include <kdl/parallel.h>
#include <kdl/result.h>
#include <kdl/result_fold.h>
#include <kdl/vector_utils.h>
#include <kdl/zip_iterator.h>

namespace TrenchBroom::Model
{

namespace
{
/**
 * Recursively collect the nodes to clone + transform, starting with the children of
 * `node`.
 * (`node` itself is skipped.)
 */
std::vector<const Node*> collectNodesToCloneAndTransform(const Node& node)
{
  auto result = std::vector<const Node*>{};

  std::function<void(const Node*)> collectNodes = [&](const Node* n) {
    result.push_back(n);
    for (auto* child : n->children())
    {
      collectNodes(child);
    }
  };

  for (auto* child : node.children())
  {
    collectNodes(child);
  }

  return result;
}

Result<std::unique_ptr<Node>> cloneAndTransformRecursive(
  const Node* nodeToClone,
  std::unordered_map<const Node*, NodeContents>& origNodeToTransformedContents,
  const vm::bbox3& worldBounds)
{
  // First, clone `n`, and move in the new (transformed) content which was
  // prepared for it above
  auto clone = nodeToClone->accept(kdl::overload(
    [](const WorldNode*) -> std::unique_ptr<Node> {
      ensure(false, "Linked group structure is valid");
    },
    [](const LayerNode*) -> std::unique_ptr<Node> {
      ensure(false, "Linked group structure is valid");
    },
    [&](const GroupNode* groupNode) -> std::unique_ptr<Node> {
      auto& group = std::get<Group>(origNodeToTransformedContents.at(groupNode).get());
      return std::make_unique<GroupNode>(std::move(group));
    },
    [&](const EntityNode* entityNode) -> std::unique_ptr<Node> {
      auto& entity = std::get<Entity>(origNodeToTransformedContents.at(entityNode).get());
      return std::make_unique<EntityNode>(std::move(entity));
    },
    [&](const BrushNode* brushNode) -> std::unique_ptr<Node> {
      auto& brush = std::get<Brush>(origNodeToTransformedContents.at(brushNode).get());
      return std::make_unique<BrushNode>(std::move(brush));
    },
    [&](const PatchNode* patchNode) -> std::unique_ptr<Node> {
      auto& patch =
        std::get<BezierPatch>(origNodeToTransformedContents.at(patchNode).get());
      return std::make_unique<PatchNode>(std::move(patch));
    }));

  if (!worldBounds.contains(clone->logicalBounds()))
  {
    return Error{"Updating a linked node would exceed world bounds"};
  }

  return kdl::fold_results(kdl::vec_transform(
                             nodeToClone->children(),
                             [&](const auto* childNode) {
                               return cloneAndTransformRecursive(
                                 childNode, origNodeToTransformedContents, worldBounds);
                             }))
    .transform([&](auto childClones) {
      for (auto& childClone : childClones)
      {
        clone->addChild(childClone.release());
      }

      return std::move(clone);
    });
}

/**
 * Given a node, clones its children recursively and applies the given transform.
 *
 * Returns a vector of the cloned direct children of `node`.
 */
Result<std::vector<std::unique_ptr<Node>>> cloneAndTransformChildren(
  const Node& node, const vm::bbox3& worldBounds, const vm::mat4x4& transformation)
{
  auto nodesToClone = collectNodesToCloneAndTransform(node);

  using TransformResult = Result<std::pair<const Node*, NodeContents>>;

  // In parallel, produce pairs { node pointer, transformed contents } from the nodes in
  // `nodesToClone`
  auto transformResults =
    kdl::vec_parallel_transform(nodesToClone, [&](const Node* nodeToTransform) {
      return nodeToTransform->accept(kdl::overload(
        [](const WorldNode*) -> TransformResult {
          ensure(false, "Linked group structure is valid");
        },
        [](const LayerNode*) -> TransformResult {
          ensure(false, "Linked group structure is valid");
        },
        [&](const GroupNode* groupNode) -> TransformResult {
          auto group = groupNode->group();
          group.transform(transformation);
          return std::make_pair(nodeToTransform, NodeContents{std::move(group)});
        },
        [&](const EntityNode* entityNode) -> TransformResult {
          auto entity = entityNode->entity();
          entity.transform(entityNode->entityPropertyConfig(), transformation);
          return std::make_pair(nodeToTransform, NodeContents{std::move(entity)});
        },
        [&](const BrushNode* brushNode) -> TransformResult {
          auto brush = brushNode->brush();
          return brush.transform(worldBounds, transformation, true)
            .and_then([&]() -> TransformResult {
              return std::make_pair(nodeToTransform, NodeContents{std::move(brush)});
            });
        },
        [&](const PatchNode* patchNode) -> TransformResult {
          auto patch = patchNode->patch();
          patch.transform(transformation);
          return std::make_pair(nodeToTransform, NodeContents{std::move(patch)});
        }));
    });

  return kdl::fold_results(std::move(transformResults))
    .or_else(
      [](const auto&) -> Result<std::vector<std::pair<const Node*, NodeContents>>> {
        return Error{"Failed to transform a linked node"};
      })
    .and_then(
      [&](auto origNodeAndTransformedContents)
        -> Result<std::vector<std::unique_ptr<Node>>> {
        // Move into map for easier lookup
        auto resultsMap = std::unordered_map<const Node*, NodeContents>{
          origNodeAndTransformedContents.begin(), origNodeAndTransformedContents.end()};
        origNodeAndTransformedContents.clear();

        // Do a recursive traversal of the input node tree again,
        // creating a matching tree structure, and move in the contents
        // we've transformed above.
        return kdl::fold_results(
          kdl::vec_transform(node.children(), [&](const auto* childNode) {
            return cloneAndTransformRecursive(childNode, resultsMap, worldBounds);
          }));
      });
}

template <typename T>
void preserveGroupNames(
  const std::vector<T>& clonedNodes, const std::vector<Model::Node*>& correspondingNodes)
{
  auto clIt = std::begin(clonedNodes);
  auto coIt = std::begin(correspondingNodes);
  while (clIt != std::end(clonedNodes) && coIt != std::end(correspondingNodes))
  {
    auto& clonedNode = *clIt;
    const auto* correspondingNode = *coIt;

    clonedNode->accept(kdl::overload(
      [](WorldNode*) {},
      [](LayerNode*) {},
      [&](GroupNode* clonedGroupNode) {
        if (
          const auto* correspondingGroupNode =
            dynamic_cast<const GroupNode*>(correspondingNode))
        {
          auto group = clonedGroupNode->group();
          group.setName(correspondingGroupNode->group().name());
          clonedGroupNode->setGroup(std::move(group));

          preserveGroupNames(
            clonedGroupNode->children(), correspondingGroupNode->children());
        }
      },
      [](EntityNode*) {},
      [](BrushNode*) {},
      [](PatchNode*) {}));

    ++clIt;
    ++coIt;
  }
}

void preserveEntityProperties(
  EntityNode& clonedEntityNode, const EntityNode& correspondingEntityNode)
{
  if (
    clonedEntityNode.entity().protectedProperties().empty()
    && correspondingEntityNode.entity().protectedProperties().empty())
  {
    return;
  }

  auto clonedEntity = clonedEntityNode.entity();
  const auto& correspondingEntity = correspondingEntityNode.entity();

  const auto allProtectedProperties = kdl::vec_sort_and_remove_duplicates(kdl::vec_concat(
    clonedEntity.protectedProperties(), correspondingEntity.protectedProperties()));

  clonedEntity.setProtectedProperties(correspondingEntity.protectedProperties());

  const auto entityPropertyConfig = clonedEntityNode.entityPropertyConfig();
  for (const auto& propertyKey : allProtectedProperties)
  {
    // this can change the order of properties
    clonedEntity.removeProperty(entityPropertyConfig, propertyKey);
    if (const auto* propertyValue = correspondingEntity.property(propertyKey))
    {
      clonedEntity.addOrUpdateProperty(entityPropertyConfig, propertyKey, *propertyValue);
    }
  }

  clonedEntityNode.setEntity(std::move(clonedEntity));
}

template <typename T>
void preserveEntityProperties(
  const std::vector<T>& clonedNodes, const std::vector<Node*>& correspondingNodes)
{
  auto clIt = std::begin(clonedNodes);
  auto coIt = std::begin(correspondingNodes);
  while (clIt != std::end(clonedNodes) && coIt != std::end(correspondingNodes))
  {
    auto& clonedNode =
      *clIt; // deduces either to std::unique_ptr<Node>& or Node*& depending on T
    const auto* correspondingNode = *coIt;

    clonedNode->accept(kdl::overload(
      [](WorldNode*) {},
      [](LayerNode*) {},
      [&](GroupNode* clonedGroupNode) {
        if (
          const auto* correspondingGroupNode =
            dynamic_cast<const GroupNode*>(correspondingNode))
        {
          preserveEntityProperties(
            clonedGroupNode->children(), correspondingGroupNode->children());
        }
      },
      [&](EntityNode* clonedEntityNode) {
        if (
          const auto* correspondingEntityNode =
            dynamic_cast<const EntityNode*>(correspondingNode))
        {
          preserveEntityProperties(*clonedEntityNode, *correspondingEntityNode);
        }
      },
      [](BrushNode*) {},
      [](PatchNode*) {}));

    ++clIt;
    ++coIt;
  }
}
} // namespace

Result<UpdateLinkedGroupsResult> updateLinkedGroups(
  const GroupNode& sourceGroupNode,
  const std::vector<Model::GroupNode*>& targetGroupNodes,
  const vm::bbox3& worldBounds)
{
  const auto& sourceGroup = sourceGroupNode.group();
  const auto [success, invertedSourceTransformation] =
    vm::invert(sourceGroup.transformation());
  if (!success)
  {
    return Error{"Group transformation is not invertible"};
  }

  const auto _invertedSourceTransformation = invertedSourceTransformation;
  const auto targetGroupNodesToUpdate =
    kdl::vec_erase(targetGroupNodes, &sourceGroupNode);
  return kdl::fold_results(
    kdl::vec_transform(targetGroupNodesToUpdate, [&](auto* targetGroupNode) {
      const auto transformation =
        targetGroupNode->group().transformation() * _invertedSourceTransformation;
      return cloneAndTransformChildren(sourceGroupNode, worldBounds, transformation)
        .transform([&](std::vector<std::unique_ptr<Node>>&& newChildren) {
          preserveGroupNames(newChildren, targetGroupNode->children());
          preserveEntityProperties(newChildren, targetGroupNode->children());

          return std::make_pair(
            static_cast<Node*>(targetGroupNode), std::move(newChildren));
        });
    }));
}
namespace
{

template <typename SourceNode, typename TargetNode, typename F>
Result<void> visitNodesPerPosition(
  SourceNode& sourceNode, TargetNode& targetNode, const F& f)
{
  return f(sourceNode, targetNode).and_then([&](const auto& recurse) -> Result<void> {
    if (recurse)
    {
      if (sourceNode.childCount() != targetNode.childCount())
      {
        return Error{"Inconsistent linked group structure"};
      }

      return kdl::fold_results(kdl::vec_transform(
        kdl::make_zip_range(sourceNode.children(), targetNode.children()),
        [&](auto& childPair) {
          auto& [sourceChild, targetChild] = childPair;
          return visitNodesPerPosition(*sourceChild, *targetChild, f);
        }));
    }

    return Result<void>{};
  });
}

template <typename N>
Result<bool> checkType(const Node& node, const bool successResult)
{
  return dynamic_cast<const N*>(&node)
           ? Result<bool>{successResult}
           : Result<bool>{Error{"Inconsistent linked group structure"}};
}

auto makeCopyEntityLinkIds(
  const std::string& containingLinkedGroupId,
  std::unordered_map<EntityNode*, std::string>& entityLinkIds)
{
  return [&](auto& sourceNode, auto& targetNode) {
    return sourceNode.accept(kdl::overload(
      [&](const WorldNode*) { return checkType<WorldNode>(targetNode, true); },
      [&](const LayerNode*) { return checkType<LayerNode>(targetNode, true); },
      [&](const GroupNode* sourceGroupNode) {
        // don't recurse into nested linked groups
        const auto& nestedLinkedGroupId = sourceGroupNode->group().linkedGroupId();
        const auto recurse =
          !nestedLinkedGroupId || nestedLinkedGroupId == containingLinkedGroupId;
        return checkType<GroupNode>(targetNode, recurse);
      },
      [&](const EntityNode* sourceEntityNode) {
        if (auto* targetEntityNode = dynamic_cast<EntityNode*>(&targetNode))
        {
          auto sourceLinkIdIt =
            entityLinkIds.find(const_cast<EntityNode*>(sourceEntityNode));
          ensure(sourceLinkIdIt != entityLinkIds.end(), "Source entity has link ID");

          const auto& sourceLinkId = sourceLinkIdIt->second;
          entityLinkIds[targetEntityNode] = sourceLinkId;

          return Result<bool>{false};
        }
        return Result<bool>{Error{"Inconsistent linked group structure"}};
      },
      [&](const BrushNode*) { return checkType<BrushNode>(targetNode, false); },
      [&](const PatchNode*) { return checkType<PatchNode>(targetNode, false); }));
  };
}

std::unordered_map<EntityNode*, std::string> initializeEntityLinkIds(
  GroupNode& rootGroupNode)
{
  auto result = std::unordered_map<EntityNode*, std::string>{};

  const auto& rootLinkedGroupId = rootGroupNode.group().linkedGroupId();
  rootGroupNode.accept(kdl::overload(
    [](const WorldNode*) {},
    [](const LayerNode*) {},
    [&](auto&& thisLambda, const GroupNode* groupNode) {
      // Don't recurse into nested linked groups
      const auto& nestedLinkedGroupId = groupNode->group().linkedGroupId();
      if (!nestedLinkedGroupId || nestedLinkedGroupId == rootLinkedGroupId)
      {
        groupNode->visitChildren(thisLambda);
      }
    },
    [&](EntityNode* entityNode) { result[entityNode] = generateUuid(); },
    [](const BrushNode*) {},
    [](const PatchNode*) {}));

  return result;
}

Result<void> copyEntityLinkIds(
  const GroupNode& sourceNode,
  GroupNode& targetNode,
  std::unordered_map<EntityNode*, std::string>& entityLinkIds)
{
  return visitNodesPerPosition(
    sourceNode,
    targetNode,
    makeCopyEntityLinkIds(*sourceNode.group().linkedGroupId(), entityLinkIds));
}


Result<void> copyEntityLinkIds(
  const Model::GroupNode& sourceGroupNode,
  const std::vector<GroupNode*>& targetGroupNodes,
  std::unordered_map<EntityNode*, std::string>& entityLinkIds)
{
  return kdl::fold_results(kdl::vec_transform(
    kdl::range{targetGroupNodes.begin(), targetGroupNodes.end()},
    [&](auto* targetGroupNode) {
      return copyEntityLinkIds(sourceGroupNode, *targetGroupNode, entityLinkIds);
    }));
}

} // namespace

Result<std::unordered_map<EntityNode*, std::string>> generateEntityLinkIds(
  const Model::GroupNode& sourceGroupNode,
  const std::vector<GroupNode*>& targetGroupNodes)
{
  auto entityLinkIds = std::unordered_map<EntityNode*, std::string>{};
  sourceGroupNode.accept(kdl::overload(
    [](const WorldNode*) {},
    [](const LayerNode*) {},
    [&](auto&& thisLambda, const GroupNode* groupNode) {
      // Don't recurse into nested linked groups
      const auto& nestedLinkedGroupId = groupNode->group().linkedGroupId();
      if (
        !nestedLinkedGroupId
        || nestedLinkedGroupId == sourceGroupNode.group().linkedGroupId())
      {
        groupNode->visitChildren(thisLambda);
      }
    },
    [&](const EntityNode* entityNode) {
      entityLinkIds[const_cast<EntityNode*>(entityNode)] = *entityNode->entity().linkId();
    },
    [](const BrushNode*) {},
    [](const PatchNode*) {}));

  return copyEntityLinkIds(sourceGroupNode, targetGroupNodes, entityLinkIds)
    .transform([&]() { return std::move(entityLinkIds); });
}

Result<std::unordered_map<EntityNode*, std::string>> generateEntityLinkIds(
  const std::vector<GroupNode*>& groupNodes)
{
  if (groupNodes.empty())
  {
    return Error{"Link set must contain at least one group"};
  }

  auto& sourceGroupNode = *groupNodes.front();
  auto entityLinkIds = initializeEntityLinkIds(sourceGroupNode);

  return kdl::fold_results(kdl::vec_transform(
                             kdl::range{std::next(groupNodes.begin()), groupNodes.end()},
                             [&](auto* targetGroupNode) {
                               return copyEntityLinkIds(
                                 sourceGroupNode, *targetGroupNode, entityLinkIds);
                             }))
    .transform([&]() { return std::move(entityLinkIds); });
}

Result<void> initializeEntityLinkIds(
  const Model::GroupNode& sourceGroupNode, const std::vector<GroupNode*>& groupNodes)
{
  return generateEntityLinkIds(sourceGroupNode, groupNodes)
    .transform([](auto entityLinkIds) {
      for (auto& [entityNode, linkId] : entityLinkIds)
      {
        auto entity = entityNode->entity();
        entity.setLinkId(linkId);
        entityNode->setEntity(std::move(entity));
      }
    });
}

Result<void> initializeEntityLinkIds(const std::vector<GroupNode*>& groupNodes)
{
  return generateEntityLinkIds(groupNodes).transform([](auto entityLinkIds) {
    for (auto& [entityNode, linkId] : entityLinkIds)
    {
      auto entity = entityNode->entity();
      entity.setLinkId(linkId);
      entityNode->setEntity(std::move(entity));
    }
  });
}

void resetEntityLinkIds(const std::vector<GroupNode*>& groupNodes)
{
  Node::visitAll(
    groupNodes,
    kdl::overload(
      [](const WorldNode*) {},
      [](const LayerNode*) {},
      [](auto&& thisLambda, const GroupNode* groupNode) {
        groupNode->visitChildren(thisLambda);
      },
      [](EntityNode* entityNode) {
        auto entity = entityNode->entity();
        entity.resetLinkId();
        entityNode->setEntity(std::move(entity));
      },
      [](const BrushNode*) {},
      [](const PatchNode*) {}));
}

} // namespace TrenchBroom::Model
