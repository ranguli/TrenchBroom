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

template <typename N1, typename N2>
Result<N1*> tryCast(N2& targetNode)
{
  auto* targetNodeCasted = dynamic_cast<N1*>(&targetNode);
  return targetNodeCasted ? Result<N1*>{targetNodeCasted}
                          : Result<N1*>{Error{"Inconsistent linked group structure"}};
}

template <typename SourceNode, typename TargetNode, typename F>
Result<void> visitNodesPerPosition(
  SourceNode& sourceNode, TargetNode& targetNode, const F& f)
{
  return sourceNode
    .accept(kdl::overload(
      [&](WorldNode* sourceWorldNode) {
        return tryCast<WorldNode>(targetNode).transform([&](WorldNode* targetWorldNode) {
          return f(*sourceWorldNode, *targetWorldNode);
        });
      },
      [&](LayerNode* sourceLayerNode) {
        return tryCast<LayerNode>(targetNode).transform([&](LayerNode* targetLayerNode) {
          return f(*sourceLayerNode, *targetLayerNode);
        });
      },
      [&](GroupNode* sourceGroupNode) {
        return tryCast<GroupNode>(targetNode).transform([&](GroupNode* targetGroupNode) {
          return f(*sourceGroupNode, *targetGroupNode);
        });
      },
      [&](EntityNode* sourceEntityNode) {
        return tryCast<EntityNode>(targetNode)
          .transform([&](EntityNode* targetEntityNode) {
            return f(*sourceEntityNode, *targetEntityNode);
          });
      },
      [&](BrushNode* sourceBrushNode) {
        return tryCast<BrushNode>(targetNode).transform([&](BrushNode* targetBrushNode) {
          return f(*sourceBrushNode, *targetBrushNode);
        });
      },
      [&](PatchNode* sourcePatchNode) {
        return tryCast<PatchNode>(targetNode).transform([&](PatchNode* targetPatchNode) {
          return f(*sourcePatchNode, *targetPatchNode);
        });
      }))
    .and_then([&](const auto& recurse) {
      if (recurse)
      {
        return visitChildrenPerPosition(sourceNode, targetNode, f);
      }

      return Result<void>{};
    });
}

template <typename SourceNode, typename TargetNode, typename F>
Result<void> visitChildrenPerPosition(
  SourceNode& sourceNode, TargetNode& targetNode, const F& f)
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

std::unordered_map<Node*, std::string> generateLinkIds(GroupNode& rootNode)
{
  auto result = std::unordered_map<Node*, std::string>{};

  // rootNode already has a link ID, so we don't visit it
  rootNode.visitChildren(kdl::overload(
    [](const WorldNode*) {},
    [](const LayerNode*) {},
    [&](auto&& thisLambda, GroupNode* groupNode) {
      // don't recurse into nested linked groups
      if (!groupNode->group().linkId())
      {
        result[groupNode] = generateUuid();
        groupNode->visitChildren(thisLambda);
      }
    },
    [&](auto&& thisLambda, EntityNode* entityNode) {
      result[entityNode] = generateUuid();
      entityNode->visitChildren(thisLambda);
    },
    [&](BrushNode* brushNode) { result[brushNode] = generateUuid(); },
    [&](PatchNode* patchNode) { result[patchNode] = generateUuid(); }));

  return result;
}

template <typename N>
void copyLinkId(
  N& sourceNode, N& targetNode, std::unordered_map<Node*, std::string>& linkIds)
{
  auto sourceLinkIdIt = linkIds.find(&sourceNode);
  ensure(sourceLinkIdIt != linkIds.end(), "Source entity has link ID");

  const auto& sourceLinkId = sourceLinkIdIt->second;
  linkIds[&targetNode] = sourceLinkId;
}

Result<void> copyLinkIds(
  GroupNode& sourceRootNode,
  GroupNode& targetRootNode,
  std::unordered_map<Node*, std::string>& linkIds)
{
  // sourceRootNode and targetRootNode already have a link IDs, so we don't visit them
  return visitChildrenPerPosition(
    sourceRootNode,
    targetRootNode,
    kdl::overload(
      [&](const WorldNode&, const WorldNode&) { return true; },
      [&](const LayerNode&, const LayerNode&) { return true; },
      [&](GroupNode& sourceGroupNode, GroupNode& targetGroupNode) {
        // don't recurse into nested linked groups
        if (!targetGroupNode.group().linkId())
        {
          copyLinkId(sourceGroupNode, targetGroupNode, linkIds);
          return true;
        }
        return false;
      },
      [&](EntityNode& sourceEntityNode, EntityNode& targetEntityNode) {
        copyLinkId(sourceEntityNode, targetEntityNode, linkIds);
        return true;
      },
      [&](BrushNode& sourceBrushNode, BrushNode& targetBrushNode) {
        copyLinkId(sourceBrushNode, targetBrushNode, linkIds);
        return false;
      },
      [&](PatchNode& sourcePatchNode, PatchNode& targetPatchNode) {
        copyLinkId(sourcePatchNode, targetPatchNode, linkIds);
        return false;
      }));
}

} // namespace

Result<std::unordered_map<Node*, std::string>> generateLinkIds(
  const std::vector<GroupNode*>& groupNodes)
{
  if (groupNodes.empty())
  {
    return Error{"Link set must contain at least one group"};
  }

  auto& sourceGroupNode = *groupNodes.front();
  auto linkIds = generateLinkIds(sourceGroupNode);

  return kdl::fold_results(kdl::vec_transform(
                             kdl::range{std::next(groupNodes.begin()), groupNodes.end()},
                             [&](auto* targetGroupNode) {
                               return copyLinkIds(
                                 sourceGroupNode, *targetGroupNode, linkIds);
                             }))
    .transform([&]() { return std::move(linkIds); });
}

Result<void> initializeLinkIds(const std::vector<GroupNode*>& groupNodes)
{
  return generateLinkIds(groupNodes).transform([](auto linkIds) {
    for (auto& [node, linkId] : linkIds)
    {
      node->accept(kdl::overload(
        [](const WorldNode*) {},
        [](const LayerNode*) {},
        [&linkId = linkId](GroupNode* groupNode) {
          auto group = groupNode->group();
          group.setLinkId(std::move(linkId));
          groupNode->setGroup(std::move(group));
        },
        [&linkId = linkId](EntityNode* entityNode) {
          auto entity = entityNode->entity();
          entity.setLinkId(std::move(linkId));
          entityNode->setEntity(std::move(entity));
        },
        [&linkId = linkId](BrushNode* brushNode) {
          auto brush = brushNode->brush();
          brush.setLinkId(std::move(linkId));
          brushNode->setBrush(std::move(brush));
        },
        [&linkId = linkId](PatchNode* patchNode) {
          auto patch = patchNode->patch();
          patch.setLinkId(std::move(linkId));
          patchNode->setPatch(std::move(patch));
        }));
    }
  });
}

} // namespace TrenchBroom::Model
