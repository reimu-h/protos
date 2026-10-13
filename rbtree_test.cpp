#include "rbtree.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <iterator>
#include <memory>
#include <set>
#include <thread>
#include <vector>

namespace protos {
namespace {

namespace t = ::testing;

struct BasicRbTestNode {
  int key;
  int value;
  RbNode node;
};

struct AugmentedRbTestNode {
  int key;
  int64_t payload_size;
  int64_t subtree_max_size;
  int64_t subtree_count;
  int subtree_key_sum;
  RbNode node;
};

struct AugmentedRbTestTraits {
  static int GetKey(const AugmentedRbTestNode& item) { return item.key; }

  static bool Less(const int a, const int b) { return a < b; }

  static void UpdateAugment(AugmentedRbTestNode* const item,
                            const AugmentedRbTestNode* const left,
                            const AugmentedRbTestNode* const right) {
    int64_t max_size = item->payload_size;
    int64_t count = 1;
    int key_sum = item->key;

    if (left != nullptr) {
      if (left->subtree_max_size > max_size) {
        max_size = left->subtree_max_size;
      }
      count += left->subtree_count;
      key_sum += left->subtree_key_sum;
    }
    if (right != nullptr) {
      if (right->subtree_max_size > max_size) {
        max_size = right->subtree_max_size;
      }
      count += right->subtree_count;
      key_sum += right->subtree_key_sum;
    }

    item->subtree_max_size = max_size;
    item->subtree_count = count;
    item->subtree_key_sum = key_sum;
  }
};

constexpr int64_t kMaxVerifyDepth = 64;
constexpr int64_t kBulkTestNodeCount = 128;
constexpr int64_t kSearchTestNodeCount = 256;

// The key and the ordering that an `RbTree` with `Traits` uses.
template <typename Traits, typename T>
static auto TestKeyOf(const T& item) {
  if constexpr (internal::RbDeclaresGetKey<Traits>) {
    return Traits::GetKey(item);
  } else {
    return item.key;
  }
}

template <typename Traits, typename T>
static bool TestItemLess(const T& a, const T& b) {
  if constexpr (internal::RbDeclaresLess<Traits>) {
    return Traits::Less(TestKeyOf<Traits>(a), TestKeyOf<Traits>(b));
  } else {
    return TestKeyOf<Traits>(a) < TestKeyOf<Traits>(b);
  }
}

// Whether the augmented fields of `item` match its own fields and the
// augmented fields of its children.
static bool AugmentIsConsistent(const AugmentedRbTestNode& item,
                                const AugmentedRbTestNode* const left,
                                const AugmentedRbTestNode* const right) {
  int64_t expected_max = item.payload_size;
  int64_t expected_count = 1;
  int expected_key_sum = item.key;
  for (const AugmentedRbTestNode* const child : {left, right}) {
    if (child != nullptr) {
      expected_max = std::max(expected_max, child->subtree_max_size);
      expected_count += child->subtree_count;
      expected_key_sum += child->subtree_key_sum;
    }
  }
  return item.subtree_max_size == expected_max &&
         item.subtree_count == expected_count &&
         item.subtree_key_sum == expected_key_sum;
}

template <typename Node>
  requires requires(const Node& node) {
    node.payload;
    node.subtree_max;
  }
static bool AugmentIsConsistent(const Node& item, const Node* const left,
                                const Node* const right) {
  int64_t expected_max = item.payload;
  for (const Node* const child : {left, right}) {
    if (child != nullptr) {
      expected_max = std::max(expected_max, child->subtree_max);
    }
  }
  return item.subtree_max == expected_max;
}

// Checks the subtree rooted at `curr`, whose keys must lie strictly between
// those of `lower` and `upper` (either may be null), and returns its black
// height.
template <typename T, RbNode T::* kNodeMember, typename Traits>
static t::AssertionResult VerifyRbSubtree(
    const RbTree<T, kNodeMember, Traits>& tree,  //
    const T* const curr,                         //
    const T* const expected_parent,              //
    const T* const lower,                        //
    const T* const upper,                        //
    const int64_t depth,                         //
    int64_t* const out_black_height,             //
    int64_t* const io_visited_count) {
  if (curr == nullptr) {
    *out_black_height = 1;
    return t::AssertionSuccess();
  }
  if (depth > kMaxVerifyDepth) {
    return t::AssertionFailure()
           << "Deeper than " << kMaxVerifyDepth << " levels (cycle?)";
  }

  ++(*io_visited_count);
  if (tree.Parent(curr) != expected_parent) {
    return t::AssertionFailure() << "Wrong parent link at depth " << depth;
  }
  if ((lower != nullptr && !TestItemLess<Traits>(*lower, *curr)) ||
      (upper != nullptr && !TestItemLess<Traits>(*curr, *upper))) {
    return t::AssertionFailure() << "Key out of order at depth " << depth;
  }

  const T* const left = tree.Left(curr);
  const T* const right = tree.Right(curr);
  const RbColor color = tree.Color(curr);

  // Invariant (2): No red node has a red child.
  if (color == RbColor::kRed && (tree.Color(left) == RbColor::kRed ||
                                 tree.Color(right) == RbColor::kRed)) {
    return t::AssertionFailure()
           << "Red node with a red child at depth " << depth;
  }

  int64_t left_black_height = 0;
  int64_t right_black_height = 0;
  const t::AssertionResult left_result = VerifyRbSubtree(tree,                //
                                                         left,                //
                                                         curr,                //
                                                         lower,               //
                                                         curr,                //
                                                         depth + 1,           //
                                                         &left_black_height,  //
                                                         io_visited_count);
  if (!left_result) {
    return left_result;
  }
  const t::AssertionResult right_result =
      VerifyRbSubtree(tree,                 //
                      right,                //
                      curr,                 //
                      curr,                 //
                      upper,                //
                      depth + 1,            //
                      &right_black_height,  //
                      io_visited_count);
  if (!right_result) {
    return right_result;
  }

  // Invariant (3): All root-to-null paths have identical black-height.
  if (left_black_height != right_black_height) {
    return t::AssertionFailure() << "Black-height mismatch at depth " << depth;
  }
  *out_black_height = left_black_height + ((color == RbColor::kBlack) ? 1 : 0);

  // Invariant (5): Every node's augmented value matches the value recomputed
  // from its own fields and its children.
  if constexpr (internal::RbDeclaresUpdateAugment<Traits>) {
    if (!AugmentIsConsistent(*curr, left, right)) {
      return t::AssertionFailure() << "Stale augmentation at depth " << depth;
    }
  }
  return t::AssertionSuccess();
}

template <typename T, RbNode T::* kNodeMember, typename Traits>
static t::AssertionResult VerifyRbTree(
    const RbTree<T, kNodeMember, Traits>& tree, const int64_t expected_count) {
  const T* const root = tree.root();
  if (root == nullptr) {
    if (tree.Empty() && tree.First() == nullptr && tree.Last() == nullptr &&
        expected_count == 0) {
      return t::AssertionSuccess();
    }
    return t::AssertionFailure()
           << "Empty tree state mismatch for expected_count=" << expected_count;
  }
  if (tree.Empty() || expected_count == 0) {
    return t::AssertionFailure()
           << "Non-null root when expected_count=" << expected_count;
  }

  // Invariant (1): Root is black and has no parent.
  if (tree.Parent(root) != nullptr || tree.Color(root) != RbColor::kBlack) {
    return t::AssertionFailure() << "Root must be black with null parent";
  }

  int64_t black_height = 0;
  int64_t visited_count = 0;
  const T* const null_node = nullptr;
  const t::AssertionResult subtree_result = VerifyRbSubtree(tree,           //
                                                            root,           //
                                                            null_node,      //
                                                            null_node,      //
                                                            null_node,      //
                                                            0,              //
                                                            &black_height,  //
                                                            &visited_count);
  if (!subtree_result) {
    return subtree_result;
  }
  if (visited_count != expected_count) {
    return t::AssertionFailure() << "Visited node count " << visited_count
                                 << " != expected " << expected_count;
  }

  const T* leftmost = root;
  while (tree.Left(leftmost) != nullptr) {
    leftmost = tree.Left(leftmost);
  }
  const T* rightmost = root;
  while (tree.Right(rightmost) != nullptr) {
    rightmost = tree.Right(rightmost);
  }
  if (tree.First() != leftmost || tree.Last() != rightmost) {
    return t::AssertionFailure() << "First()/Last() mismatch with tree extrema";
  }
  if (tree.Prev(tree.First()) != nullptr || tree.Next(tree.Last()) != nullptr) {
    return t::AssertionFailure()
           << "Prev(First()) and Next(Last()) must be null";
  }

  // Invariant (4): In-order traversal keys are strictly ordered, and
  // Next()/Prev() are exact inverses across all nodes.
  int64_t iter_count = 0;
  const T* prev = nullptr;
  const T* curr = tree.First();
  while (curr != nullptr) {
    ++iter_count;
    if (iter_count > expected_count) {
      return t::AssertionFailure() << "Iteration cycle detected";
    }
    if (prev != nullptr) {
      if (!TestItemLess<Traits>(*prev, *curr)) {
        return t::AssertionFailure() << "Out-of-order keys during traversal";
      }
      if (tree.Prev(curr) != prev) {
        return t::AssertionFailure() << "Prev(curr) != prev during traversal";
      }
    }
    prev = curr;
    curr = tree.Next(curr);
  }

  if (iter_count != expected_count || prev != tree.Last()) {
    return t::AssertionFailure() << "In-order traversal count/end mismatch";
  }
  return t::AssertionSuccess();
}

template <typename Tree>
static std::vector<int> CollectKeys(const Tree& tree) {
  std::vector<int> keys;
  for (auto* curr = tree.First(); curr != nullptr; curr = Tree::Next(curr)) {
    keys.push_back(curr->key);
  }
  return keys;
}

template <typename Tree>
static std::vector<int> CollectKeysReverse(const Tree& tree) {
  std::vector<int> keys;
  for (auto* curr = tree.Last(); curr != nullptr; curr = Tree::Prev(curr)) {
    keys.push_back(curr->key);
  }
  return keys;
}

// Number of items on the longest path from `item` down to a leaf.
template <typename Tree, typename T>
static int SubtreeHeight(const T* const item) {
  if (item == nullptr) {
    return 0;
  }
  return 1 + std::max(SubtreeHeight<Tree>(Tree::Left(item)),
                      SubtreeHeight<Tree>(Tree::Right(item)));
}

static void InitAugmentedTestNode(AugmentedRbTestNode* const item,
                                  const int64_t index) {
  const int key = (index + 1) * 16;
  const int64_t payload_size = ((index * 97 + 31) % 500) + 16;
  item->key = key;
  item->payload_size = payload_size;
  item->subtree_max_size = 0;
  item->subtree_count = 0;
  item->subtree_key_sum = 0;
  item->node = {};
}

// Deterministic splitmix64 generator, so that every run of a randomized test
// performs the same operations.
class TestRng {
 public:
  explicit TestRng(const uint64_t seed) : state_(seed) {}

  // Returns a pseudo-random value in [0, bound).
  int Uniform(const int bound) {
    state_ += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z ^= z >> 31;
    return static_cast<int>(z % static_cast<uint64_t>(bound));
  }

 private:
  uint64_t state_;
};

static void VerifyThreeNodeRotationCase(const int k0,  //
                                        const int k1,  //
                                        const int k2) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;
  BasicRbTestNode nodes[3] = {
      {k0, k0 * 10, {}},
      {k1, k1 * 10, {}},
      {k2, k2 * 10, {}},
  };

  ASSERT_TRUE(tree.Insert(&nodes[0]));
  ASSERT_TRUE(VerifyRbTree(tree, 1));
  ASSERT_TRUE(tree.Insert(&nodes[1]));
  ASSERT_TRUE(VerifyRbTree(tree, 2));
  ASSERT_TRUE(tree.Insert(&nodes[2]));
  ASSERT_TRUE(VerifyRbTree(tree, 3));

  BasicRbTestNode* const root = tree.root();
  ASSERT_THAT(root, t::NotNull());
  EXPECT_THAT(root->key, t::Eq(20));
  EXPECT_THAT(tree.Color(root), t::Eq(RbColor::kBlack));

  const BasicRbTestNode* const left = tree.Left(root);
  const BasicRbTestNode* const right = tree.Right(root);
  ASSERT_THAT(left, t::NotNull());
  ASSERT_THAT(right, t::NotNull());
  EXPECT_THAT(left->key, t::Eq(10));
  EXPECT_THAT(right->key, t::Eq(30));
  EXPECT_THAT(tree.Color(left), t::Eq(RbColor::kRed));
  EXPECT_THAT(tree.Color(right), t::Eq(RbColor::kRed));
  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(10, 20, 30));

  tree.Erase(root);
  ASSERT_TRUE(VerifyRbTree(tree, 2));
  tree.Erase(tree.First());
  ASSERT_TRUE(VerifyRbTree(tree, 1));
  tree.Erase(tree.root());
  EXPECT_TRUE(VerifyRbTree(tree, 0));
}

TEST(RbTreeTest, EmptyTreeAndNullPointerSafety) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  EXPECT_TRUE(tree.Empty());
  EXPECT_THAT(tree.root(), t::IsNull());
  EXPECT_THAT(tree.First(), t::IsNull());
  EXPECT_THAT(tree.Last(), t::IsNull());
  EXPECT_THAT(tree.Find(42), t::IsNull());
  EXPECT_THAT(tree.LowerBound(42), t::IsNull());
  EXPECT_THAT(tree.UpperBound(42), t::IsNull());
  EXPECT_THAT(Tree::Next(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Prev(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Left(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Right(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Parent(static_cast<BasicRbTestNode*>(nullptr)),
              t::IsNull());
  EXPECT_THAT(Tree::Color(nullptr), t::Eq(RbColor::kBlack));
  EXPECT_FALSE(tree.Insert(nullptr));
  EXPECT_THAT(CollectKeys(tree), t::IsEmpty());
  EXPECT_TRUE(VerifyRbTree(tree, 0));

  tree.Erase(nullptr);
  tree.PropagateAugment(nullptr);
  EXPECT_TRUE(VerifyRbTree(tree, 0));
}

TEST(RbTreeTest, SingleNodeLifecycleAndDuplicateKeyRejection) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  BasicRbTestNode first_node = {50, 500, {}};
  BasicRbTestNode duplicate_node = {50, 501, {}};

  ASSERT_TRUE(tree.Insert(first_node));
  ASSERT_TRUE(VerifyRbTree(tree, 1));

  EXPECT_FALSE(tree.Insert(&duplicate_node));
  ASSERT_TRUE(VerifyRbTree(tree, 1));
  EXPECT_THAT(Tree::Parent(&duplicate_node), t::IsNull());
  EXPECT_THAT(Tree::Left(&duplicate_node), t::IsNull());
  EXPECT_THAT(Tree::Right(&duplicate_node), t::IsNull());

  // Inserting an item that is already in the tree is refused the same way.
  EXPECT_FALSE(tree.Insert(first_node));
  ASSERT_TRUE(VerifyRbTree(tree, 1));

  EXPECT_THAT(tree.root(), t::Eq(&first_node));
  EXPECT_THAT(tree.First(), t::Eq(&first_node));
  EXPECT_THAT(tree.Last(), t::Eq(&first_node));
  EXPECT_THAT(Tree::Left(&first_node), t::IsNull());
  EXPECT_THAT(Tree::Right(&first_node), t::IsNull());
  EXPECT_THAT(Tree::Parent(&first_node), t::IsNull());
  EXPECT_THAT(Tree::Color(&first_node), t::Eq(RbColor::kBlack));
  EXPECT_THAT(tree.Find(50), t::Eq(&first_node));
  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(50));

  tree.Erase(first_node);
  EXPECT_TRUE(VerifyRbTree(tree, 0));
  EXPECT_THAT(CollectKeys(tree), t::IsEmpty());
}

TEST(RbTreeTest, RotationsAndMultiLevelInsertEraseFixup) {
  // LL (right rotation), RR (left rotation), LR (left-right double rotation),
  // and RL (right-left double rotation) cases.
  VerifyThreeNodeRotationCase(30, 20, 10);
  VerifyThreeNodeRotationCase(10, 20, 30);
  VerifyThreeNodeRotationCase(30, 10, 20);
  VerifyThreeNodeRotationCase(10, 30, 20);

  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;
  constexpr int kFixupKeys[] = {20, 10, 30, 5, 15, 25, 35, 2,
                                7,  12, 18, 1, 3,  6,  8};
  constexpr int64_t kFixupCount = sizeof(kFixupKeys) / sizeof(kFixupKeys[0]);
  BasicRbTestNode nodes[kFixupCount];

  for (int64_t i = 0; i < kFixupCount; ++i) {
    nodes[i] = {kFixupKeys[i], kFixupKeys[i] * 3, {}};
    ASSERT_TRUE(tree.Insert(&nodes[i]));
    ASSERT_TRUE(VerifyRbTree(tree, i + 1));
  }

  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(1, 2, 3, 5, 6, 7, 8, 10, 12, 15,
                                                18, 20, 25, 30, 35));

  for (int64_t i = 0; i < kFixupCount; ++i) {
    BasicRbTestNode* const victim = ((i & 1) == 0) ? tree.root() : tree.First();
    tree.Erase(victim);
    ASSERT_TRUE(VerifyRbTree(tree, kFixupCount - 1 - i));
  }

  EXPECT_TRUE(tree.Empty());
}

TEST(RbTreeTest, QueriesAndBidirectionalIteration) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  constexpr int kInsertOrder[] = {80, 40, 120, 20, 60,  100, 140, 10,
                                  30, 50, 70,  90, 110, 130, 150, 160};
  constexpr int64_t kCount = sizeof(kInsertOrder) / sizeof(kInsertOrder[0]);
  BasicRbTestNode nodes[kCount];

  for (int64_t i = 0; i < kCount; ++i) {
    nodes[i] = {kInsertOrder[i], static_cast<int>(i + 100), {}};
    ASSERT_TRUE(tree.Insert(&nodes[i]));
    ASSERT_TRUE(VerifyRbTree(tree, i + 1));
  }

  const Tree& const_tree = tree;

  // Below minimum key (10).
  EXPECT_THAT(tree.Find(0), t::IsNull());
  EXPECT_THAT(const_tree.Find(9), t::IsNull());
  ASSERT_THAT(tree.LowerBound(0), t::NotNull());
  EXPECT_THAT(tree.LowerBound(0)->key, t::Eq(10));
  ASSERT_THAT(const_tree.LowerBound(9), t::NotNull());
  EXPECT_THAT(const_tree.LowerBound(9)->key, t::Eq(10));
  ASSERT_THAT(tree.UpperBound(0), t::NotNull());
  EXPECT_THAT(tree.UpperBound(0)->key, t::Eq(10));
  ASSERT_THAT(const_tree.UpperBound(9), t::NotNull());
  EXPECT_THAT(const_tree.UpperBound(9)->key, t::Eq(10));

  // Exact keys (10, 20, ..., 160) and missing odd midpoint keys (15, 25, ...).
  for (int64_t idx = 1; idx <= kCount; ++idx) {
    const int exact_key = idx * 10;
    const BasicRbTestNode* const found = tree.Find(exact_key);
    const BasicRbTestNode* const cfound = const_tree.Find(exact_key);
    const BasicRbTestNode* const lb = tree.LowerBound(exact_key);
    const BasicRbTestNode* const clb = const_tree.LowerBound(exact_key);
    const BasicRbTestNode* const ub = tree.UpperBound(exact_key);
    const BasicRbTestNode* const cub = const_tree.UpperBound(exact_key);

    ASSERT_THAT(found, t::NotNull());
    EXPECT_THAT(found->key, t::Eq(exact_key));
    EXPECT_THAT(cfound, t::Eq(found));
    EXPECT_THAT(lb, t::Eq(found));
    EXPECT_THAT(clb, t::Eq(found));
    EXPECT_THAT(ub, t::Eq(cub));

    if (idx < kCount) {
      ASSERT_THAT(ub, t::NotNull());
      EXPECT_THAT(ub->key, t::Eq(exact_key + 10));

      const int mid_key = exact_key + 5;
      EXPECT_THAT(tree.Find(mid_key), t::IsNull());
      EXPECT_THAT(const_tree.Find(mid_key), t::IsNull());
      EXPECT_THAT(tree.LowerBound(mid_key), t::Eq(ub));
      EXPECT_THAT(const_tree.LowerBound(mid_key), t::Eq(ub));
      EXPECT_THAT(tree.UpperBound(mid_key), t::Eq(ub));
      EXPECT_THAT(const_tree.UpperBound(mid_key), t::Eq(ub));
    } else {
      EXPECT_THAT(ub, t::IsNull());
    }
  }

  // Above maximum key (160).
  EXPECT_THAT(tree.Find(161), t::IsNull());
  EXPECT_THAT(const_tree.Find(1000), t::IsNull());
  EXPECT_THAT(tree.LowerBound(161), t::IsNull());
  EXPECT_THAT(const_tree.LowerBound(1000), t::IsNull());
  EXPECT_THAT(tree.UpperBound(161), t::IsNull());
  EXPECT_THAT(const_tree.UpperBound(1000), t::IsNull());

  EXPECT_THAT(CollectKeys(const_tree),
              t::ElementsAre(10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120,
                             130, 140, 150, 160));

  EXPECT_THAT(CollectKeysReverse(const_tree),
              t::ElementsAre(160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60,
                             50, 40, 30, 20, 10));

  for (int64_t i = 0; i < kCount; ++i) {
    tree.Erase(&nodes[i]);
    ASSERT_TRUE(VerifyRbTree(tree, kCount - 1 - i));
  }
}

TEST(RbTreeTest, AugmentedBulkInsertAndErase) {
  using AugTree = RbTree<AugmentedRbTestNode,         //
                         &AugmentedRbTestNode::node,  //
                         AugmentedRbTestTraits>;
  AugTree tree;
  AugmentedRbTestNode nodes[kBulkTestNodeCount];

  // Phase 1: Ascending insert (0..127) and ascending erase (0..127).
  for (int64_t i = 0; i < kBulkTestNodeCount; ++i) {
    InitAugmentedTestNode(&nodes[i], i);
    ASSERT_TRUE(tree.Insert(&nodes[i]));
    ASSERT_TRUE(VerifyRbTree(tree, i + 1));
  }
  for (int64_t i = 0; i < kBulkTestNodeCount; ++i) {
    tree.Erase(&nodes[i]);
    ASSERT_TRUE(VerifyRbTree(tree, kBulkTestNodeCount - 1 - i));
  }

  // Phase 2: Descending insert (127..0) and descending erase (127..0).
  for (int64_t i = kBulkTestNodeCount; i > 0; --i) {
    const int64_t idx = i - 1;
    InitAugmentedTestNode(&nodes[idx], idx);
    const int64_t expected_count = kBulkTestNodeCount - idx;
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
    ASSERT_TRUE(VerifyRbTree(tree, expected_count));
  }
  for (int64_t i = kBulkTestNodeCount; i > 0; --i) {
    const int64_t idx = i - 1;
    tree.Erase(&nodes[idx]);
    ASSERT_TRUE(VerifyRbTree(tree, idx));
  }

  // Phase 3: Interleaved outside-in insert + continuous root deletion (forcing
  // two-child successor transplants and fixups at every step).
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    const int64_t idx =
        ((step & 1) == 0) ? (step / 2) : (kBulkTestNodeCount - 1 - step / 2);
    InitAugmentedTestNode(&nodes[idx], idx);
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
    ASSERT_TRUE(VerifyRbTree(tree, step + 1));
  }
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    AugmentedRbTestNode* const current_root = tree.root();
    ASSERT_THAT(current_root, t::NotNull());
    tree.Erase(current_root);
    ASSERT_TRUE(VerifyRbTree(tree, kBulkTestNodeCount - 1 - step));
  }

  // Phase 4: Pseudo-random permutation insert + independent pseudo-random
  // permutation erase (gcd(37, 128) == 1 and gcd(83, 128) == 1).
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    const int64_t idx = (step * 37 + 11) & (kBulkTestNodeCount - 1);
    InitAugmentedTestNode(&nodes[idx], idx);
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
    ASSERT_TRUE(VerifyRbTree(tree, step + 1));
  }
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    const int64_t idx = (step * 83 + 59) & (kBulkTestNodeCount - 1);
    tree.Erase(&nodes[idx]);
    ASSERT_TRUE(VerifyRbTree(tree, kBulkTestNodeCount - 1 - step));
  }

  EXPECT_TRUE(tree.Empty());
}

TEST(RbTreeTest, PropagateAugmentAndAugmentedSearchWithMockPredicates) {
  using AugTree = RbTree<AugmentedRbTestNode,         //
                         &AugmentedRbTestNode::node,  //
                         AugmentedRbTestTraits>;
  AugTree tree;
  AugmentedRbTestNode nodes[kSearchTestNodeCount];

  // Verify FindFirstAugmented on an empty tree never invokes predicates.
  {
    t::MockFunction<bool(const AugmentedRbTestNode&)> empty_pred;
    EXPECT_CALL(empty_pred, Call).Times(0);
    EXPECT_THAT(tree.FindFirstAugmented(empty_pred.AsStdFunction(),
                                        empty_pred.AsStdFunction()),
                t::IsNull());
  }

  // Populate 256 nodes in pseudo-random order (gcd(73, 256) == 1).
  for (int64_t step = 0; step < kSearchTestNodeCount; ++step) {
    const int64_t idx = (step * 73 + 19) & (kSearchTestNodeCount - 1);
    InitAugmentedTestNode(&nodes[idx], idx);
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
  }
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  // Test PropagateAugment after in-place payload_size mutations.
  const int64_t original_size_42 = nodes[42].payload_size;
  nodes[42].payload_size = 50000;
  tree.PropagateAugment(&nodes[42]);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Eq(50000));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  nodes[42].payload_size = 8;
  tree.PropagateAugment(nodes[42]);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Lt(50000));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  nodes[42].payload_size = original_size_42;
  tree.PropagateAugment(&nodes[42]);
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  // Mutate root in-place and propagate.
  AugmentedRbTestNode* const root_node = tree.root();
  const int64_t old_root_size = root_node->payload_size;
  root_node->payload_size = 12345;
  tree.PropagateAugment(root_node);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Eq(12345));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));
  root_node->payload_size = old_root_size;
  tree.PropagateAugment(root_node);

  // Place a unique peak size (20000) at the maximum-key node (index 255) so
  // any unpruned search for 20000 would have to visit all 256 nodes.
  nodes[kSearchTestNodeCount - 1].payload_size = 20000;
  tree.PropagateAugment(&nodes[kSearchTestNodeCount - 1]);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Eq(20000));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  constexpr int64_t kTargetSizes[] = {1,   8,   16,    100,   250,  400,
                                      500, 515, 20000, 20001, 99999};
  const AugTree& const_tree = tree;

  // A red-black tree of n items is at most 2 * log2(n + 1) levels high:
  // floor(2 * log2(257)) = 16.
  const int height = SubtreeHeight<AugTree>(const_tree.root());
  ASSERT_THAT(height, t::Le(16));

  for (const int64_t target : kTargetSizes) {
    // Ground-truth linear scan in key order (First -> Next).
    const AugmentedRbTestNode* expected_match = nullptr;
    for (const AugmentedRbTestNode* curr = const_tree.First(); curr != nullptr;
         curr = AugTree::Next(curr)) {
      if (curr->payload_size >= target) {
        expected_match = curr;
        break;
      }
    }

    t::MockFunction<bool(const AugmentedRbTestNode&)> subtree_pred;
    t::MockFunction<bool(const AugmentedRbTestNode&)> node_pred;

    if (target > const_tree.root()->subtree_max_size) {
      EXPECT_CALL(subtree_pred, Call)
          .WillOnce([&const_tree, target](const AugmentedRbTestNode& node) {
            EXPECT_THAT(&node, t::Eq(const_tree.root()));
            return node.subtree_max_size >= target;
          });
      EXPECT_CALL(node_pred, Call).Times(0);
    } else {
      // With an exact subtree predicate, the search calls `subtree_pred` at
      // most twice per level (on the subtree it descends into and on a left
      // subtree it rejects) and `node_pred` at most once per level.
      EXPECT_CALL(subtree_pred, Call)
          .Times(t::Between(1, 2 * height))
          .WillRepeatedly([target](const AugmentedRbTestNode& node) {
            EXPECT_THAT(node.subtree_count, t::Ge(1));
            return node.subtree_max_size >= target;
          });
      EXPECT_CALL(node_pred, Call)
          .Times(t::Between(1, height))
          .WillRepeatedly([target](const AugmentedRbTestNode& node) {
            return node.payload_size >= target;
          });
    }

    const AugmentedRbTestNode* const found_two_pred = tree.FindFirstAugmented(
        subtree_pred.AsStdFunction(), node_pred.AsStdFunction());
    EXPECT_THAT(found_two_pred, t::Eq(expected_match));

    // Also verify the const overload on const_tree.
    const AugmentedRbTestNode* const found_const =
        const_tree.FindFirstAugmented(
            [target](const AugmentedRbTestNode& node) {
              return node.subtree_max_size >= target;
            },
            [target](const AugmentedRbTestNode& node) {
              return node.payload_size >= target;
            });
    EXPECT_THAT(found_const, t::Eq(expected_match));
  }

  EXPECT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));
}

TEST(RbTreeTest, RandomizedInsertEraseAndPropagateMatchStdSet) {
  using AugTree = RbTree<AugmentedRbTestNode,         //
                         &AugmentedRbTestNode::node,  //
                         AugmentedRbTestTraits>;
  constexpr uint64_t kSeeds[] = {1, 2, 3, 0xC0FFEE};
  constexpr int kKeySpace = 192;
  constexpr int kSteps = 3000;
  constexpr int kPhaseSteps = 300;
  constexpr int kMaxPayload = 1000;
  constexpr int kNoKey = -1000;

  const auto key_or_none = [](const AugmentedRbTestNode* const item) {
    return (item == nullptr) ? kNoKey : item->key;
  };

  for (const uint64_t seed : kSeeds) {
    SCOPED_TRACE(t::Message() << "seed " << seed);
    TestRng rng(seed);
    AugTree tree;
    // Item `i` has key `2 * i`, so odd probes fall between items.
    std::vector<AugmentedRbTestNode> nodes(kKeySpace);
    std::set<int> keys;

    for (int step = 0; step < kSteps; ++step) {
      // Alternate between phases that mostly insert and phases that mostly
      // erase, so the tree size sweeps between small and large.
      const bool growing = (step / kPhaseSteps) % 2 == 0;
      const int index = rng.Uniform(kKeySpace);
      const int key = 2 * index;
      AugmentedRbTestNode& node = nodes[index];
      if (keys.contains(key)) {
        if (growing) {
          node.payload_size = rng.Uniform(kMaxPayload);
          tree.PropagateAugment(node);
        } else {
          tree.Erase(node);
          keys.erase(key);
        }
      } else if (growing || rng.Uniform(4) == 0) {
        node = {key, rng.Uniform(kMaxPayload), 0, 0, 0, {}};
        ASSERT_TRUE(tree.Insert(node));
        keys.insert(key);
      }
      ASSERT_TRUE(VerifyRbTree(tree, std::ssize(keys))) << "step " << step;

      const int probe = rng.Uniform(2 * kKeySpace + 2) - 1;
      const auto lower = keys.lower_bound(probe);
      const auto upper = keys.upper_bound(probe);
      ASSERT_THAT(key_or_none(tree.Find(probe)),
                  t::Eq(keys.contains(probe) ? probe : kNoKey));
      ASSERT_THAT(key_or_none(tree.LowerBound(probe)),
                  t::Eq((lower == keys.end()) ? kNoKey : *lower));
      ASSERT_THAT(key_or_none(tree.UpperBound(probe)),
                  t::Eq((upper == keys.end()) ? kNoKey : *upper));

      if (step % 16 == 0) {
        const int64_t target = rng.Uniform(kMaxPayload + 1);
        const AugmentedRbTestNode* expected_match = nullptr;
        for (const int linked_key : keys) {
          if (nodes[linked_key / 2].payload_size >= target) {
            expected_match = &nodes[linked_key / 2];
            break;
          }
        }
        ASSERT_THAT(tree.FindFirstAugmented(
                        [target](const AugmentedRbTestNode& item) {
                          return item.subtree_max_size >= target;
                        },
                        [target](const AugmentedRbTestNode& item) {
                          return item.payload_size >= target;
                        }),
                    t::Eq(expected_match));
      }
    }
    EXPECT_THAT(CollectKeys(tree), t::ElementsAreArray(keys));
  }
}

class MockAugmentObserver {
 public:
  MOCK_METHOD(void,             //
              OnUpdateAugment,  //
              (int node_key, const int* left_key, const int* right_key));
};

struct ObservedRbTestNode {
  int key;
  std::shared_ptr<MockAugmentObserver> observer;
  RbNode node;
};

struct ObservedRbTestTraits {
  static int GetKey(const ObservedRbTestNode& item) { return item.key; }

  static void UpdateAugment(ObservedRbTestNode* const item,
                            const ObservedRbTestNode* const left,
                            const ObservedRbTestNode* const right) {
    if (item->observer != nullptr) {
      const int* const left_key = (left != nullptr) ? &left->key : nullptr;
      const int* const right_key = (right != nullptr) ? &right->key : nullptr;
      item->observer->OnUpdateAugment(item->key, left_key, right_key);
    }
  }
};

TEST(RbTreeTest, CustomTraitsInvokeAugmentCallbacksOnInsertAndPropagate) {
  const auto observer = std::make_shared<MockAugmentObserver>();
  RbTree<ObservedRbTestNode, &ObservedRbTestNode::node, ObservedRbTestTraits>
      tree;

  ObservedRbTestNode n10 = {10, observer, {}};
  ObservedRbTestNode n20 = {20, observer, {}};

  EXPECT_CALL(*observer, OnUpdateAugment)
      .WillOnce([](const int node_key,         //
                   const int* const left_key,  //
                   const int* const right_key) {
        EXPECT_THAT(node_key, t::Eq(10));
        EXPECT_THAT(left_key, t::IsNull());
        EXPECT_THAT(right_key, t::IsNull());
      })
      .WillRepeatedly([](const int node_key,         //
                         const int* const left_key,  //
                         const int* const right_key) {
        EXPECT_THAT(node_key, t::AnyOf(t::Eq(10), t::Eq(20)));
        (void)left_key;
        (void)right_key;
      });

  ASSERT_TRUE(tree.Insert(&n10));
  ASSERT_TRUE(tree.Insert(&n20));

  EXPECT_CALL(*observer, OnUpdateAugment)
      .WillOnce([](const int node_key,         //
                   const int* const left_key,  //
                   const int* const right_key) {
        EXPECT_THAT(node_key, t::Eq(20));
        EXPECT_THAT(left_key, t::IsNull());
        EXPECT_THAT(right_key, t::IsNull());
      })
      .WillOnce([](const int node_key,         //
                   const int* const left_key,  //
                   const int* const right_key) {
        EXPECT_THAT(node_key, t::Eq(10));
        EXPECT_THAT(left_key, t::IsNull());
        ASSERT_THAT(right_key, t::NotNull());
        EXPECT_THAT(*right_key, t::Eq(20));
      });

  tree.PropagateAugment(&n20);

  // Read-only FindFirstAugmented queries must not trigger observer callbacks.
  EXPECT_CALL(*observer, OnUpdateAugment).Times(0);
  const auto& const_tree = tree;
  EXPECT_THAT(const_tree.FindFirstAugmented(
                  [](const ObservedRbTestNode&) { return true; },
                  [](const ObservedRbTestNode& n) { return n.key >= 10; }),
              t::Eq(&n10));
  EXPECT_THAT(const_tree.FindFirstAugmented(
                  [](const ObservedRbTestNode&) { return true; },
                  [](const ObservedRbTestNode& n) { return n.key > 100; }),
              t::IsNull());
}

TEST(RbTreeTest, EraseOfDetachedItemIsNoOp) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;
  BasicRbTestNode inserted[8];
  for (int i = 0; i < 8; ++i) {
    inserted[i] = {(i + 1) * 10, i, {}};
    ASSERT_TRUE(tree.Insert(&inserted[i]));
  }
  ASSERT_TRUE(VerifyRbTree(tree, 8));

  BasicRbTestNode never_inserted = {999, 99, {}};
  tree.Erase(&never_inserted);
  tree.Erase(never_inserted);
  ASSERT_TRUE(VerifyRbTree(tree, 8));

  // Erase a real node, then erase it a second time after it has been removed.
  tree.Erase(&inserted[3]);
  ASSERT_TRUE(VerifyRbTree(tree, 7));
  EXPECT_THAT(Tree::Parent(&inserted[3]), t::IsNull());
  EXPECT_THAT(Tree::Left(&inserted[3]), t::IsNull());
  EXPECT_THAT(Tree::Right(&inserted[3]), t::IsNull());
  EXPECT_THAT(Tree::Next(&inserted[3]), t::IsNull());
  EXPECT_THAT(Tree::Prev(&inserted[3]), t::IsNull());
  tree.Erase(&inserted[3]);
  ASSERT_TRUE(VerifyRbTree(tree, 7));

  Tree empty_tree;
  empty_tree.Erase(never_inserted);
  empty_tree.Erase(inserted[3]);
  EXPECT_TRUE(VerifyRbTree(empty_tree, 0));
}

TEST(RbTreeDeathTest, EraseOfItemNotInThisTreeDchecks) {
#ifdef NDEBUG
  GTEST_SKIP() << "Erase() checks tree membership only with DCHECKs enabled";
#endif
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree_a;
  Tree tree_b;
  BasicRbTestNode a_nodes[8];
  BasicRbTestNode b_nodes[8];
  for (int i = 0; i < 8; ++i) {
    a_nodes[i] = {i, 0, {}};
    b_nodes[i] = {i, 0, {}};
    ASSERT_TRUE(tree_a.Insert(a_nodes[i]));
    ASSERT_TRUE(tree_b.Insert(b_nodes[i]));
  }

  // A non-root item of another tree.
  BasicRbTestNode* const a_leaf = tree_a.First();
  ASSERT_THAT(Tree::Parent(a_leaf), t::NotNull());
  EXPECT_DEATH(tree_b.Erase(a_leaf), "Check failed");

  // The root of another tree.
  EXPECT_DEATH(tree_b.Erase(tree_a.root()), "Check failed");

  // An item that was in the tree before Clear().
  tree_a.Clear();
  BasicRbTestNode fresh_nodes[2] = {{100, 0, {}}, {101, 0, {}}};
  ASSERT_TRUE(tree_a.Insert(fresh_nodes[0]));
  ASSERT_TRUE(tree_a.Insert(fresh_nodes[1]));
  EXPECT_DEATH(tree_a.Erase(a_leaf), "Check failed");
}

TEST(RbTreeTest, ClearForgetsItemsWithoutTouchingThem) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;
  constexpr int kCount = 16;
  auto nodes = std::make_unique<BasicRbTestNode[]>(kCount);
  for (int i = 0; i < kCount; ++i) {
    nodes[i].key = i;
    ASSERT_TRUE(tree.Insert(nodes[i]));
  }

  // The items' storage may be gone by the time the tree is cleared. Under
  // ASan, any access to them would be reported as a heap-use-after-free.
  nodes.reset();
  tree.Clear();
  EXPECT_TRUE(VerifyRbTree(tree, 0));
}

TEST(RbTreeTest, ClearedItemsCanBeInsertedAgain) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;
  constexpr int kCount = 16;
  BasicRbTestNode nodes[kCount];
  for (int step = 0; step < kCount; ++step) {
    const int i = (step * 7) % kCount;
    nodes[i] = {i, 0, {}};
    ASSERT_TRUE(tree.Insert(nodes[i]));
  }
  tree.Clear();
  EXPECT_TRUE(VerifyRbTree(tree, 0));

  // The items still have stale links, which Insert() overwrites.
  for (BasicRbTestNode& node : nodes) {
    ASSERT_TRUE(tree.Insert(node));
  }
  EXPECT_TRUE(VerifyRbTree(tree, kCount));
  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                                10, 11, 12, 13, 14, 15));
}

// Traits protocol test cases: hooks are detected by name (also when
// inherited), and a detected hook that can't be called as documented makes
// `RbTree` fail to compile instead of being ignored.
struct ProtocolNode {
  int key;
  RbNode node;
};

struct KeylessProtocolNode {
  int id;
  RbNode node;
};

struct KeyMethodProtocolNode {
  int key() const { return 0; }
  RbNode node;
};

struct ValidHooksTraits {
  static int GetKey(const ProtocolNode& item) { return item.key; }
  static bool Less(const int a, const int b) { return a < b; }
  static void UpdateAugment(ProtocolNode*, const ProtocolNode*,
                            const ProtocolNode*) {}
};

struct InheritedHooksTraits : ValidHooksTraits {};

struct GenericHooksTraits {
  template <typename T>
  static void UpdateAugment(T*, const T*, const T*) {}
};

struct NonConstChildrenTraits {
  static void UpdateAugment(ProtocolNode*, ProtocolNode*, ProtocolNode*) {}
};

struct ReferenceItemTraits {
  static void UpdateAugment(ProtocolNode&, const ProtocolNode*,
                            const ProtocolNode*) {}
};

struct BoolReturningUpdateAugmentTraits {
  static bool UpdateAugment(ProtocolNode*, const ProtocolNode*,
                            const ProtocolNode*) {
    return true;
  }
};

struct SingleArgumentUpdateAugmentTraits {
  static void UpdateAugment(ProtocolNode*) {}
};

struct NonStaticUpdateAugmentTraits {
  void UpdateAugment(ProtocolNode*, const ProtocolNode*, const ProtocolNode*) {}
};

struct DataMemberUpdateAugmentTraits {
  static constexpr bool UpdateAugment = true;
};

struct PointerGetKeyTraits {
  static int GetKey(const ProtocolNode* const item) { return item->key; }
};

struct NonConstGetKeyTraits {
  static int GetKey(ProtocolNode& item) { return item.key; }
};

struct NonConstLessTraits {
  static bool Less(int& a, int& b) { return a < b; }
};

struct NonStaticLessTraits {
  bool Less(const int a, const int b) const { return a < b; }
};

struct FinalTraits final {};

TEST(RbTreeTraitsTest, HooksAreDetectedByNameAndMustBeCallableAsDocumented) {
  // No hooks: `item.key`, `operator<`, and no augmentation.
  static_assert(!internal::RbDeclaresGetKey<DefaultRbTraits>);
  static_assert(!internal::RbDeclaresLess<DefaultRbTraits>);
  static_assert(!internal::RbDeclaresUpdateAugment<DefaultRbTraits>);
  static_assert(internal::RbGetKeyUsable<ProtocolNode, DefaultRbTraits>);
  static_assert(internal::RbLessUsable<int, DefaultRbTraits>);
  static_assert(internal::RbUpdateAugmentUsable<ProtocolNode, DefaultRbTraits>);
  // The default key must be a `key` data member.
  static_assert(
      !internal::RbGetKeyUsable<KeylessProtocolNode, DefaultRbTraits>);
  static_assert(
      !internal::RbGetKeyUsable<KeyMethodProtocolNode, DefaultRbTraits>);

  // Hooks declared as documented, also taking parameters by value, inherited,
  // or as templates.
  static_assert(internal::RbDeclaresGetKey<ValidHooksTraits>);
  static_assert(internal::RbDeclaresLess<ValidHooksTraits>);
  static_assert(internal::RbDeclaresUpdateAugment<ValidHooksTraits>);
  static_assert(internal::RbGetKeyUsable<ProtocolNode, ValidHooksTraits>);
  static_assert(internal::RbLessUsable<int, ValidHooksTraits>);
  static_assert(
      internal::RbUpdateAugmentUsable<ProtocolNode, ValidHooksTraits>);
  static_assert(internal::RbDeclaresGetKey<InheritedHooksTraits>);
  static_assert(internal::RbDeclaresLess<InheritedHooksTraits>);
  static_assert(internal::RbDeclaresUpdateAugment<InheritedHooksTraits>);
  static_assert(
      internal::RbUpdateAugmentUsable<ProtocolNode, InheritedHooksTraits>);
  static_assert(internal::RbDeclaresUpdateAugment<GenericHooksTraits>);
  static_assert(
      internal::RbUpdateAugmentUsable<ProtocolNode, GenericHooksTraits>);

  // Declared hooks that can't be called as documented are detected but not
  // usable, so `RbTree` rejects them with a static_assert.
  static_assert(internal::RbDeclaresUpdateAugment<NonConstChildrenTraits>);
  static_assert(
      !internal::RbUpdateAugmentUsable<ProtocolNode, NonConstChildrenTraits>);
  static_assert(internal::RbDeclaresUpdateAugment<ReferenceItemTraits>);
  static_assert(
      !internal::RbUpdateAugmentUsable<ProtocolNode, ReferenceItemTraits>);
  static_assert(
      internal::RbDeclaresUpdateAugment<BoolReturningUpdateAugmentTraits>);
  static_assert(
      !internal::RbUpdateAugmentUsable<ProtocolNode,
                                       BoolReturningUpdateAugmentTraits>);
  static_assert(
      internal::RbDeclaresUpdateAugment<SingleArgumentUpdateAugmentTraits>);
  static_assert(
      !internal::RbUpdateAugmentUsable<ProtocolNode,
                                       SingleArgumentUpdateAugmentTraits>);
  static_assert(
      internal::RbDeclaresUpdateAugment<NonStaticUpdateAugmentTraits>);
  static_assert(!internal::RbUpdateAugmentUsable<ProtocolNode,
                                                 NonStaticUpdateAugmentTraits>);
  static_assert(
      internal::RbDeclaresUpdateAugment<DataMemberUpdateAugmentTraits>);
  static_assert(
      !internal::RbUpdateAugmentUsable<ProtocolNode,
                                       DataMemberUpdateAugmentTraits>);
  static_assert(internal::RbDeclaresGetKey<PointerGetKeyTraits>);
  static_assert(!internal::RbGetKeyUsable<ProtocolNode, PointerGetKeyTraits>);
  static_assert(internal::RbDeclaresGetKey<NonConstGetKeyTraits>);
  static_assert(!internal::RbGetKeyUsable<ProtocolNode, NonConstGetKeyTraits>);
  static_assert(internal::RbDeclaresLess<NonConstLessTraits>);
  static_assert(!internal::RbLessUsable<int, NonConstLessTraits>);
  static_assert(internal::RbDeclaresLess<NonStaticLessTraits>);
  static_assert(!internal::RbLessUsable<int, NonStaticLessTraits>);

  // Hook detection needs a traits class that can be derived from.
  static_assert(!internal::RbTraitsClass<FinalTraits>);
  static_assert(!internal::RbTraitsClass<int>);
}

struct DescendingKeyTraits {
  static bool Less(const int a, const int b) { return a > b; }
};

TEST(RbTreeTraitsTest, LessHookDefinesTheOrder) {
  using Tree =
      RbTree<BasicRbTestNode, &BasicRbTestNode::node, DescendingKeyTraits>;
  Tree tree;
  constexpr int kKeys[] = {40, 10, 70, 30, 60, 20, 50};
  constexpr int kCount = sizeof(kKeys) / sizeof(kKeys[0]);
  BasicRbTestNode nodes[kCount];
  for (int i = 0; i < kCount; ++i) {
    nodes[i] = {kKeys[i], i, {}};
    ASSERT_TRUE(tree.Insert(nodes[i]));
  }
  ASSERT_TRUE(VerifyRbTree(tree, kCount));
  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(70, 60, 50, 40, 30, 20, 10));

  // Lookups follow the custom order too.
  EXPECT_THAT(tree.Find(60), t::Eq(&nodes[4]));
  ASSERT_THAT(tree.LowerBound(35), t::NotNull());
  EXPECT_THAT(tree.LowerBound(35)->key, t::Eq(30));
  ASSERT_THAT(tree.UpperBound(30), t::NotNull());
  EXPECT_THAT(tree.UpperBound(30)->key, t::Eq(20));
  BasicRbTestNode duplicate = {40, 99, {}};
  EXPECT_FALSE(tree.Insert(duplicate));
}

struct ValueKeyedTraits {
  static int GetKey(const BasicRbTestNode& item) { return item.value; }
};

TEST(RbTreeTraitsTest, GetKeyHookDefinesTheKey) {
  using Tree =
      RbTree<BasicRbTestNode, &BasicRbTestNode::node, ValueKeyedTraits>;
  Tree tree;
  BasicRbTestNode nodes[] = {
      {1, 50, {}}, {2, 40, {}}, {3, 30, {}}, {4, 20, {}}, {5, 10, {}},
  };
  for (BasicRbTestNode& node : nodes) {
    ASSERT_TRUE(tree.Insert(node));
  }
  ASSERT_TRUE(VerifyRbTree(tree, 5));
  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(5, 4, 3, 2, 1));
  EXPECT_THAT(tree.Find(30), t::Eq(&nodes[2]));
  EXPECT_THAT(tree.Find(3), t::IsNull());
}

struct InheritedAugmentedTraits : AugmentedRbTestTraits {};

TEST(RbTreeTraitsTest, InheritedHooksAreUsed) {
  using Tree = RbTree<AugmentedRbTestNode,         //
                      &AugmentedRbTestNode::node,  //
                      InheritedAugmentedTraits>;
  Tree tree;
  constexpr int kCount = 32;
  AugmentedRbTestNode nodes[kCount];
  for (int step = 0; step < kCount; ++step) {
    const int i = (step * 13) % kCount;
    InitAugmentedTestNode(&nodes[i], i);
    ASSERT_TRUE(tree.Insert(nodes[i]));
  }
  // Only the inherited `UpdateAugment` keeps the augmentation consistent.
  ASSERT_TRUE(VerifyRbTree(tree, kCount));
  for (int i = 0; i < kCount; i += 2) {
    tree.Erase(nodes[i]);
  }
  EXPECT_TRUE(VerifyRbTree(tree, kCount / 2));
}

// Node types for the typed augmented-search tests. Each caches the largest
// `payload` in its subtree in `subtree_max`.
struct CopyableMaxNode {
  int key = 0;
  int64_t payload = 0;
  int64_t subtree_max = 0;
  RbNode node;
};

struct NonCopyableMaxNode {
  NonCopyableMaxNode() = default;
  NonCopyableMaxNode(const NonCopyableMaxNode&) = delete;
  NonCopyableMaxNode& operator=(const NonCopyableMaxNode&) = delete;

  int key = 0;
  int64_t payload = 0;
  int64_t subtree_max = 0;
  RbNode node;
};

// Not trivially destructible.
struct SharedOwnerMaxNode {
  int key = 0;
  int64_t payload = 0;
  int64_t subtree_max = 0;
  std::shared_ptr<int> owner;
  RbNode node;
};

// Ordered by address, like the heap and PMM free trees.
struct AddressKeyedMaxNode {
  int64_t payload = 0;
  int64_t subtree_max = 0;
  RbNode node;
};

// Maintains `subtree_max` and counts its calls.
template <typename Node>
struct MaxPayloadTraits {
  static void UpdateAugment(Node* const item, const Node* const left,
                            const Node* const right) {
    update_calls.fetch_add(1, std::memory_order_relaxed);
    int64_t max_payload = item->payload;
    for (const Node* const child : {left, right}) {
      if (child != nullptr) {
        max_payload = std::max(max_payload, child->subtree_max);
      }
    }
    item->subtree_max = max_payload;
  }

  static inline std::atomic<int64_t> update_calls{0};
};

template <typename Node>
struct AddressKeyedMaxPayloadTraits : MaxPayloadTraits<Node> {
  static uintptr_t GetKey(const Node& item) {
    return reinterpret_cast<uintptr_t>(&item);
  }
};

template <typename NodeT, typename TraitsT>
struct AugmentedSearchCase {
  using Node = NodeT;
  using Traits = TraitsT;
};

template <typename Case>
class RbTreeAugmentedSearchTest : public t::Test {
 protected:
  using Node = typename Case::Node;
  using Traits = typename Case::Traits;
  using Tree = RbTree<Node, &Node::node, Traits>;

  static constexpr int kCount = 64;
  static constexpr int64_t kPeakPayload = 5000;

  // Links `kCount` items in a scrambled order. Their key order is their array
  // order: keys grow with the index, as do addresses.
  void SetUp() override {
    nodes_ = std::make_unique<Node[]>(kCount);
    for (int i = 0; i < kCount; ++i) {
      Node& node = nodes_[i];
      if constexpr (requires { node.key; }) {
        node.key = (i + 1) * 10;
      }
      node.payload = ((i * 37 + 11) % 200) + 10;
    }
    nodes_[kCount - 1].payload = kPeakPayload;
    for (int step = 0; step < kCount; ++step) {
      ASSERT_TRUE(tree_.Insert(nodes_[(step * 29 + 7) % kCount]));
    }
    ASSERT_TRUE(VerifyRbTree(tree_, kCount));
  }

  // The first item in key order whose payload is at least `target`.
  const Node* LinearFirstFit(const int64_t target) const {
    for (int i = 0; i < kCount; ++i) {
      if (nodes_[i].payload >= target) {
        return &nodes_[i];
      }
    }
    return nullptr;
  }

  // The same, found with `FindFirstAugmented()` on the const tree. Sets
  // `*saw_foreign_item` if a predicate gets an item that isn't in `nodes_`.
  const Node* AugmentedFirstFit(
      const int64_t target, std::atomic<bool>* const saw_foreign_item) const {
    const Node* const begin = &nodes_[0];
    const Node* const end = begin + kCount;
    const auto check_item = [begin, end, saw_foreign_item](const Node& item) {
      if (&item < begin || &item >= end) {
        saw_foreign_item->store(true, std::memory_order_relaxed);
      }
    };
    const Tree& const_tree = tree_;
    return const_tree.FindFirstAugmented(
        [target, &check_item](const Node& item) {
          check_item(item);
          return item.subtree_max >= target;
        },
        [target, &check_item](const Node& item) {
          check_item(item);
          return item.payload >= target;
        });
  }

  std::unique_ptr<Node[]> nodes_;
  Tree tree_;
};

using AugmentedSearchCases = t::Types<
    AugmentedSearchCase<CopyableMaxNode, MaxPayloadTraits<CopyableMaxNode>>,
    AugmentedSearchCase<NonCopyableMaxNode,
                        MaxPayloadTraits<NonCopyableMaxNode>>,
    AugmentedSearchCase<SharedOwnerMaxNode,
                        MaxPayloadTraits<SharedOwnerMaxNode>>,
    AugmentedSearchCase<AddressKeyedMaxNode,
                        AddressKeyedMaxPayloadTraits<AddressKeyedMaxNode>>>;
TYPED_TEST_SUITE(RbTreeAugmentedSearchTest, AugmentedSearchCases);

constexpr int64_t kFirstFitTargets[] = {1, 10, 100, 209, 210, 5000, 5001};

TYPED_TEST(RbTreeAugmentedSearchTest, MatchesLinearScanOnLinkedItems) {
  using Traits = typename TestFixture::Traits;
  Traits::update_calls.store(0);
  for (const int64_t target : kFirstFitTargets) {
    SCOPED_TRACE(target);
    std::atomic<bool> saw_foreign_item{false};
    EXPECT_THAT(this->AugmentedFirstFit(target, &saw_foreign_item),
                t::Eq(this->LinearFirstFit(target)));
    // The predicates get the linked items themselves, never copies.
    EXPECT_FALSE(saw_foreign_item.load());
  }
  // Queries don't recompute the augmentation.
  EXPECT_THAT(Traits::update_calls.load(), t::Eq(0));
}

TYPED_TEST(RbTreeAugmentedSearchTest, ConcurrentConstQueriesAgree) {
  constexpr int kNumThreads = 4;
  constexpr int kIterations = 50;
  std::atomic<bool> saw_foreign_item{false};
  std::vector<std::thread> readers;
  readers.reserve(kNumThreads);
  for (int i = 0; i < kNumThreads; ++i) {
    readers.emplace_back([this, &saw_foreign_item]() {
      for (int iter = 0; iter < kIterations; ++iter) {
        for (const int64_t target : kFirstFitTargets) {
          EXPECT_THAT(this->AugmentedFirstFit(target, &saw_foreign_item),
                      t::Eq(this->LinearFirstFit(target)));
        }
      }
    });
  }
  for (std::thread& reader : readers) {
    reader.join();
  }
  EXPECT_FALSE(saw_foreign_item.load());
}

}  // namespace
}  // namespace protos
