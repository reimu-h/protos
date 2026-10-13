#ifndef PROTOS_RBTREE_H_
#define PROTOS_RBTREE_H_

// Intrusive red-black tree with optional subtree augmentation.
//
// The algorithms are the classic ones from Cormen et al., "Introduction to
// Algorithms" (CLRS), chapter 13, with each node's color stored in bit 0 of its
// parent pointer. Augmentation follows CLRS "Augmenting Data Structures": each
// node can cache a value computed from its own fields and its children's cached
// values, such as the largest free block in its subtree, which
// `FindFirstAugmented()` uses to prune searches.
//
// Intrusive model:
// - Items embed an `RbNode`, and `RbTree<T, &T::member, Traits>` links them
//   through it. The tree never allocates, copies, moves, or destroys items, and
//   it does not own them.
// - An item can be in at most one tree per embedded `RbNode`.
// - While an item is linked, it must stay alive at the same address and its key
//   must not change. To change a key, erase the item, update the key, and
//   insert the item again.
// - An item is detached while its `RbNode` is in the initial state
//   (`RbNode{}`), which is also the state that `Erase()` leaves it in.
// - Keys are unique: `Insert()` refuses an item whose key is equivalent to the
//   key of an item already in the tree.
//
// Key extraction, ordering, and augmentation are customized with a traits
// class; see `DefaultRbTraits`.
//
// Complexity: `Insert()`, `Erase()`, `Find()`, `LowerBound()`, `UpperBound()`,
// `First()`, `Last()`, `Next()`, and `Prev()` take O(log n) time, and a full
// traversal with `Next()` or `Prev()` takes O(n). A tree of n items is at most
// 2 * log2(n + 1) levels high. Only `FindFirstAugmented()` is recursive, and
// its recursion depth is bounded by the tree height.
//
// Thread safety: there is no internal synchronization. Mutations must be
// serialized with all other accesses, e.g. with an `IrqSpinLock`. The const
// member functions and the static navigation functions modify neither the tree
// nor its items, so concurrent readers are safe while nothing mutates the tree.

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "check.h"

namespace protos {

enum class RbColor : uint8_t {
  kRed = 0,
  kBlack = 1,
};

// The links that an item embeds to be part of an `RbTree`. `RbNode{}` is the
// detached state. The parent pointer and the color share `parent_color`: the
// color is bit 0, which is free because `RbNode`s are at least 2-byte aligned.
struct RbNode {
  RbNode* left = nullptr;
  RbNode* right = nullptr;
  uintptr_t parent_color = 0;

  RbNode* parent() const {
    return reinterpret_cast<RbNode*>(parent_color & ~uintptr_t{1});
  }

  void set_parent(RbNode* const p) {
    parent_color =
        reinterpret_cast<uintptr_t>(p) | (parent_color & uintptr_t{1});
  }

  RbColor color() const {
    return static_cast<RbColor>(parent_color & uintptr_t{1});
  }

  void set_color(const RbColor c) {
    parent_color = (parent_color & ~uintptr_t{1}) | static_cast<uintptr_t>(c);
  }

  void set_parent_and_color(RbNode* const p, const RbColor c) {
    parent_color = reinterpret_cast<uintptr_t>(p) | static_cast<uintptr_t>(c);
  }
};

static_assert(sizeof(RbNode) == 24);
static_assert(alignof(RbNode) >= 2);

// Default `RbTree` traits. Also documents the traits protocol.
//
// A traits class customizes `RbTree<T, kNodeMember, Traits>` with up to three
// static member functions ("hooks"). Every hook is optional; an omitted hook
// gets the default behavior listed with it. A declared hook must be callable
// exactly as shown (taking parameters by value instead of by const reference is
// fine), otherwise `RbTree` fails to compile with a `static_assert` that names
// the hook: it never silently falls back to the default. Hooks are detected by
// name, so they can also be inherited from a base class. The traits class must
// be a non-final class type.
//
//   static Key GetKey(const T& item);
//     Returns the key that orders `item`, by value or by const reference.
//     Default: the data member `item.key`.
//
//   static bool Less(const Key& a, const Key& b);
//     A strict weak ordering of keys. Default: `a < b`.
//
//   static void UpdateAugment(T* item, const T* left, const T* right);
//     Recomputes the augmented fields of `item` from its other fields and from
//     its children `left` and `right` (either may be null), whose augmented
//     fields are already up to date. It must not read anything else and must
//     not change keys or `RbNode`s. Declaring it makes the tree augmented:
//     `Insert()` and `Erase()`, including their rotations, call it bottom-up on
//     every node whose subtree changed, and `PropagateAugment()` calls it from
//     an item up to the root. Default: the tree is not augmented.
//
// `DefaultRbTraits` declares no hooks, so it orders items by `item.key` with
// `operator<` and maintains no augmentation.
struct DefaultRbTraits {};

namespace internal {

// Traits hooks are detected by name: looking up `Name` in
// `RbHookLookup<Traits, Probe>`, where `Probe` declares a `Name` of its own, is
// ambiguous, and therefore ill-formed, exactly when `Traits` declares or
// inherits a member called `Name`, whatever its kind or signature.
template <typename Traits, typename Probe>
struct RbHookLookup : Traits, Probe {};

struct RbGetKeyProbe {
  static void GetKey();
};

struct RbLessProbe {
  static void Less();
};

struct RbUpdateAugmentProbe {
  static void UpdateAugment();
};

template <typename Traits>
concept RbTraitsClass = std::is_class_v<Traits> && !std::is_final_v<Traits>;

template <typename Traits>
concept RbDeclaresGetKey = RbTraitsClass<Traits> && !requires {
  &RbHookLookup<Traits, RbGetKeyProbe>::GetKey;
};

template <typename Traits>
concept RbDeclaresLess = RbTraitsClass<Traits> && !requires {
  &RbHookLookup<Traits, RbLessProbe>::Less;
};

template <typename Traits>
concept RbDeclaresUpdateAugment = RbTraitsClass<Traits> && !requires {
  &RbHookLookup<Traits, RbUpdateAugmentProbe>::UpdateAugment;
};

// Whether each hook, or the default that replaces it, can be used as
// documented in `DefaultRbTraits`.
template <typename T, typename Traits>
concept RbGetKeyUsable =
    (RbDeclaresGetKey<Traits> && requires(const T& item) {
      Traits::GetKey(item);
    }) || (!RbDeclaresGetKey<Traits> && requires(const T& item) { item.key; });

template <typename Key, typename Traits>
concept RbLessUsable =
    (RbDeclaresLess<Traits> && requires(const Key& a, const Key& b) {
      { Traits::Less(a, b) } -> std::convertible_to<bool>;
    }) || (!RbDeclaresLess<Traits> && requires(const Key& a, const Key& b) {
      { a < b } -> std::convertible_to<bool>;
    });

template <typename T, typename Traits>
concept RbUpdateAugmentUsable =
    !RbDeclaresUpdateAugment<Traits> || requires(T* item, const T* child) {
      { Traits::UpdateAugment(item, child, child) } -> std::same_as<void>;
    };

// A predicate for `RbTree::FindFirstAugmented()`.
template <typename Pred, typename T>
concept RbItemPredicate = requires(const Pred& pred, const T& item) {
  { pred(item) } -> std::convertible_to<bool>;
};

}  // namespace internal

// Intrusive red-black tree of `T` items linked through their `kNodeMember`,
// customized by `Traits` (see `DefaultRbTraits`). See the overview at the top
// of this file.
template <typename T,               //
          RbNode T::* kNodeMember,  //
          typename Traits = DefaultRbTraits>
class RbTree {
  static_assert(kNodeMember != nullptr, "RbTree: kNodeMember must not be null");
  static_assert(internal::RbTraitsClass<Traits>,
                "RbTree: Traits must be a non-final class type");
  static_assert(internal::RbGetKeyUsable<T, Traits>,
                "RbTree: Traits::GetKey must be callable as `static Key "
                "GetKey(const T& item)`; without a GetKey hook, T needs a "
                "`key` data member");
  static_assert(internal::RbUpdateAugmentUsable<T, Traits>,
                "RbTree: Traits::UpdateAugment must be callable as `static "
                "void UpdateAugment(T* item, const T* left, const T* right)`");
  static_assert(alignof(T) >= 2,
                "RbTree: T must be at least 2-byte aligned (not packed) so "
                "that bit 0 of RbNode addresses is free for the color");

  static decltype(auto) ExtractKey(const T& item) {
    if constexpr (internal::RbDeclaresGetKey<Traits>) {
      return Traits::GetKey(item);
    } else {
      return item.key;
    }
  }

 public:
  // The type of item keys.
  using KeyType =
      std::remove_cvref_t<decltype(ExtractKey(std::declval<const T&>()))>;

  static_assert(internal::RbLessUsable<KeyType, Traits>,
                "RbTree: Traits::Less must be callable as `static bool "
                "Less(const Key& a, const Key& b)`; without a Less hook, keys "
                "need `operator<`");

  constexpr RbTree() = default;
  RbTree(const RbTree&) = delete;
  RbTree& operator=(const RbTree&) = delete;

  // Returns true if the tree has no items.
  bool Empty() const { return root_ == nullptr; }

  // Forgets all items in O(1) time without touching them, so it is safe even if
  // their storage has already been released or reused. The former items keep
  // stale links: before passing one to an `RbTree` function again, insert it
  // (`Insert()` overwrites the links) or reset its `RbNode` to `RbNode{}`.
  void Clear() { root_ = nullptr; }

  // Returns the root item, or null if the tree is empty.
  T* root() { return NodeToItem(root_); }
  const T* root() const { return NodeToItem(root_); }

  // Return the left child, right child, or parent of `item`, or null if there
  // is none or `item` is null. They only follow the links of `item`, so they
  // are static; for a detached item they return null.
  static T* Left(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->left);
  }
  static const T* Left(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->left);
  }

  static T* Right(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->right);
  }
  static const T* Right(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->right);
  }

  static T* Parent(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->parent());
  }
  static const T* Parent(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->parent());
  }

  // Returns the color of `item`. Null, the empty leaf, is black, and a detached
  // item is red.
  static RbColor Color(const T* const item) {
    return (item == nullptr) ? RbColor::kBlack : ItemToNode(item)->color();
  }

  // Return the item with the smallest (`First()`) or largest (`Last()`) key, or
  // null if the tree is empty.
  T* First() { return NodeToItem(MinimumNode(root_)); }
  const T* First() const { return NodeToItem(MinimumNode(root_)); }

  T* Last() { return NodeToItem(MaximumNode(root_)); }
  const T* Last() const { return NodeToItem(MaximumNode(root_)); }

  // Return the in-order successor (`Next()`) or predecessor (`Prev()`) of
  // `item`, or null if there is none or `item` is null or detached.
  static T* Next(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(NextNode(ItemToNode(item)));
  }
  static const T* Next(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(NextNode(ItemToNode(item)));
  }

  static T* Prev(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(PrevNode(ItemToNode(item)));
  }
  static const T* Prev(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(PrevNode(ItemToNode(item)));
  }

  // Links `item` into the tree and returns true. Returns false, leaving the
  // tree and `item` untouched, if `item` is null or the tree already has an
  // item with an equivalent key. `item` must not be linked in another tree; its
  // `RbNode` is overwritten.
  bool Insert(T& item) { return Insert(&item); }

  bool Insert(T* const item) {
    if (item == nullptr) {
      return false;
    }
    const KeyType& z_key = ExtractKey(*item);
    RbNode* parent = nullptr;
    RbNode* curr = root_;
    bool insert_left = false;

    while (curr != nullptr) {
      parent = curr;
      const KeyType& curr_key = ExtractKey(*NodeToItem(curr));
      if (KeyLess(z_key, curr_key)) {
        curr = curr->left;
        insert_left = true;
      } else if (KeyLess(curr_key, z_key)) {
        curr = curr->right;
        insert_left = false;
      } else {
        return false;
      }
    }

    RbNode* const z = ItemToNode(item);
    DCHECK((reinterpret_cast<uintptr_t>(z) & uintptr_t{1}) == 0);
    z->left = nullptr;
    z->right = nullptr;
    z->set_parent_and_color(parent, RbColor::kRed);

    if (parent == nullptr) {
      root_ = z;
    } else if (insert_left) {
      parent->left = z;
    } else {
      parent->right = z;
    }

    PropagateNodeAugmentToRoot(z);
    InsertFixup(z);
    return true;
  }

  // Unlinks `item` from the tree and leaves it detached. `item` must be in this
  // tree or detached (never inserted, or already erased); a detached or null
  // `item` is ignored. Erasing an item that is linked in another tree, or that
  // was in this tree before `Clear()`, corrupts the trees; DCHECK builds catch
  // it with an O(log n) membership check.
  void Erase(T& item) { Erase(&item); }

  void Erase(T* const item) {
    if (item == nullptr) {
      return;
    }
    RbNode* const z = ItemToNode(item);
    if (z->parent() == nullptr && z != root_) {
      // Only a detached item may get here: the root of another tree would be a
      // caller bug.
      DCHECK(IsDetached(z));
      return;
    }
    DCHECK(TopmostAncestor(z) == root_);
    RbNode* y = z;
    RbColor y_original_color = y->color();
    RbNode* x = nullptr;
    RbNode* x_parent = nullptr;

    if (z->left == nullptr) {
      x = z->right;
      x_parent = z->parent();
      Transplant(z, z->right);
    } else if (z->right == nullptr) {
      x = z->left;
      x_parent = z->parent();
      Transplant(z, z->left);
    } else {
      y = MinimumNode(z->right);
      y_original_color = y->color();
      x = y->right;
      if (y->parent() == z) {
        x_parent = y;
      } else {
        x_parent = y->parent();
        Transplant(y, y->right);
        y->right = z->right;
        y->right->set_parent(y);
      }
      Transplant(z, y);
      y->left = z->left;
      y->left->set_parent(y);
      y->set_color(z->color());
    }

    z->left = nullptr;
    z->right = nullptr;
    z->set_parent_and_color(nullptr, RbColor::kRed);

    PropagateNodeAugmentToRoot(x_parent);

    if (y_original_color == RbColor::kBlack) {
      EraseFixup(x, x_parent);
    }
  }

  // Recomputes the augmentation of `item` and of its ancestors, bottom-up.
  //
  // `Insert()` and `Erase()` already maintain the augmentation of every
  // affected node, including across rotations, so call this only after
  // changing, in place, a field that `Traits::UpdateAugment` reads (e.g. a
  // block size) of an item that is in the tree. If the key of an item changes,
  // erase and re-insert the item instead. Does nothing if the tree is not
  // augmented or `item` is null.
  void PropagateAugment(T& item) { PropagateAugment(&item); }

  void PropagateAugment(T* const item) {
    if (item == nullptr) {
      return;
    }
    PropagateNodeAugmentToRoot(ItemToNode(item));
  }

  // Returns the item whose key is equivalent to `key`, or null if there is
  // none.
  T* Find(const KeyType& key) {
    return const_cast<T*>(static_cast<const RbTree*>(this)->Find(key));
  }

  const T* Find(const KeyType& key) const {
    const RbNode* curr = root_;
    while (curr != nullptr) {
      const T* const curr_item = NodeToItem(curr);
      const KeyType& curr_key = ExtractKey(*curr_item);
      if (KeyLess(key, curr_key)) {
        curr = curr->left;
      } else if (KeyLess(curr_key, key)) {
        curr = curr->right;
      } else {
        return curr_item;
      }
    }
    return nullptr;
  }

  // Returns the first item, in key order, whose key is not less than `key`, or
  // null if there is none.
  T* LowerBound(const KeyType& key) {
    return const_cast<T*>(static_cast<const RbTree*>(this)->LowerBound(key));
  }

  const T* LowerBound(const KeyType& key) const {
    const RbNode* curr = root_;
    const T* result = nullptr;
    while (curr != nullptr) {
      const T* const curr_item = NodeToItem(curr);
      if (!KeyLess(ExtractKey(*curr_item), key)) {
        result = curr_item;
        curr = curr->left;
      } else {
        curr = curr->right;
      }
    }
    return result;
  }

  // Returns the first item, in key order, whose key is greater than `key`, or
  // null if there is none.
  T* UpperBound(const KeyType& key) {
    return const_cast<T*>(static_cast<const RbTree*>(this)->UpperBound(key));
  }

  const T* UpperBound(const KeyType& key) const {
    const RbNode* curr = root_;
    const T* result = nullptr;
    while (curr != nullptr) {
      const T* const curr_item = NodeToItem(curr);
      if (KeyLess(key, ExtractKey(*curr_item))) {
        result = curr_item;
        curr = curr->left;
      } else {
        curr = curr->right;
      }
    }
    return result;
  }

  // Returns the first item, in key order, that satisfies `node_pred`, or null
  // if there is none.
  //
  // `subtree_pred(item)` prunes the search: it is called on the root of each
  // subtree before the subtree is searched, and the subtree is skipped if it
  // returns false. It must return true whenever an item in the subtree of
  // `item`, `item` included, satisfies `node_pred`, typically by checking the
  // augmented fields of `item`; otherwise matches are silently missed.
  //
  // The search visits O(log n) items when `subtree_pred` is exact, i.e. true
  // only if the subtree contains a match. A looser `subtree_pred`, e.g. one
  // that can't check everything `node_pred` checks, still gives correct results
  // but may visit up to O(n) items.
  //
  // Both predicates are called with the linked items themselves (as
  // `const T&`, never with copies), must return something convertible to bool,
  // and must not modify the tree.
  template <typename SubtreePred, typename NodePred>
    requires internal::RbItemPredicate<SubtreePred, T> &&
             internal::RbItemPredicate<NodePred, T>
  T* FindFirstAugmented(const SubtreePred& subtree_pred,
                        const NodePred& node_pred) {
    return const_cast<T*>(FindFirstInSubtree(root_, subtree_pred, node_pred));
  }

  template <typename SubtreePred, typename NodePred>
    requires internal::RbItemPredicate<SubtreePred, T> &&
             internal::RbItemPredicate<NodePred, T>
  const T* FindFirstAugmented(const SubtreePred& subtree_pred,
                              const NodePred& node_pred) const {
    return FindFirstInSubtree(root_, subtree_pred, node_pred);
  }

 private:
  static constexpr bool kAugmented = internal::RbDeclaresUpdateAugment<Traits>;

  RbNode* root_ = nullptr;

  // Byte offset of `kNodeMember` within `T`. The Itanium C++ ABI, which GCC
  // and Clang follow on x86-64, represents a pointer to data member as exactly
  // this offset, so it is read from that representation instead of being
  // computed through a `T*` that doesn't point to a `T`.
  static std::ptrdiff_t NodeOffset() {
    static_assert(sizeof(kNodeMember) == sizeof(std::ptrdiff_t));
    return __builtin_bit_cast(std::ptrdiff_t, kNodeMember);
  }

  // `Item` is `T` or `const T`, and the result is `RbNode*` or
  // `const RbNode*` accordingly.
  template <typename Item>
  static auto* ItemToNode(Item* const item) {
    return &(item->*kNodeMember);
  }

  // Returns the item that embeds `node` (the "container_of" idiom), or null.
  // `Node` is `RbNode` or `const RbNode`, and the result is `T*` or `const T*`
  // accordingly.
  template <typename Node>
  static auto* NodeToItem(Node* const node) {
    using Item = std::conditional_t<std::is_const_v<Node>, const T, T>;
    using Byte = std::conditional_t<std::is_const_v<Node>, const char, char>;
    if (node == nullptr) {
      return static_cast<Item*>(nullptr);
    }
    return reinterpret_cast<Item*>(reinterpret_cast<Byte*>(node) -
                                   NodeOffset());
  }

  static bool KeyLess(const KeyType& a, const KeyType& b) {
    if constexpr (internal::RbDeclaresLess<Traits>) {
      return Traits::Less(a, b);
    } else {
      return a < b;
    }
  }

  static bool IsRed(const RbNode* const node) {
    return node != nullptr && node->color() == RbColor::kRed;
  }

  static bool IsBlack(const RbNode* const node) {
    return node == nullptr || node->color() == RbColor::kBlack;
  }

  static bool IsDetached(const RbNode* const node) {
    return node->left == nullptr && node->right == nullptr &&
           node->parent_color == 0;
  }

  // Returns the root of the tree that `node` is linked in.
  static const RbNode* TopmostAncestor(const RbNode* node) {
    while (node->parent() != nullptr) {
      node = node->parent();
    }
    return node;
  }

  // The navigation helpers below take and return `RbNode*` or `const RbNode*`.
  template <typename Node>
  static Node* MinimumNode(Node* node) {
    if (node != nullptr) {
      while (node->left != nullptr) {
        node = node->left;
      }
    }
    return node;
  }

  template <typename Node>
  static Node* MaximumNode(Node* node) {
    if (node != nullptr) {
      while (node->right != nullptr) {
        node = node->right;
      }
    }
    return node;
  }

  template <typename Node>
  static Node* NextNode(Node* node) {
    if (node->right != nullptr) {
      return MinimumNode<Node>(node->right);
    }
    Node* parent = node->parent();
    while (parent != nullptr && node == parent->right) {
      node = parent;
      parent = parent->parent();
    }
    return parent;
  }

  template <typename Node>
  static Node* PrevNode(Node* node) {
    if (node->left != nullptr) {
      return MaximumNode<Node>(node->left);
    }
    Node* parent = node->parent();
    while (parent != nullptr && node == parent->left) {
      node = parent;
      parent = parent->parent();
    }
    return parent;
  }

  static void UpdateNodeAugment([[maybe_unused]] RbNode* const node) {
    if constexpr (kAugmented) {
      Traits::UpdateAugment(NodeToItem(node),                               //
                            static_cast<const T*>(NodeToItem(node->left)),  //
                            static_cast<const T*>(NodeToItem(node->right)));
    }
  }

  static void PropagateNodeAugmentToRoot([[maybe_unused]] RbNode* const start) {
    if constexpr (kAugmented) {
      for (RbNode* curr = start; curr != nullptr; curr = curr->parent()) {
        UpdateNodeAugment(curr);
      }
    }
  }

  void Transplant(RbNode* const u, RbNode* const v) {
    RbNode* const u_parent = u->parent();
    if (u_parent == nullptr) {
      root_ = v;
    } else if (u == u_parent->left) {
      u_parent->left = v;
    } else {
      u_parent->right = v;
    }
    if (v != nullptr) {
      v->set_parent(u_parent);
    }
  }

  void RotateLeft(RbNode* const x) {
    RbNode* const y = x->right;
    x->right = y->left;
    if (y->left != nullptr) {
      y->left->set_parent(x);
    }
    RbNode* const x_parent = x->parent();
    y->set_parent(x_parent);
    if (x_parent == nullptr) {
      root_ = y;
    } else if (x == x_parent->left) {
      x_parent->left = y;
    } else {
      x_parent->right = y;
    }
    y->left = x;
    x->set_parent(y);

    UpdateNodeAugment(x);
    UpdateNodeAugment(y);
  }

  void RotateRight(RbNode* const x) {
    RbNode* const y = x->left;
    x->left = y->right;
    if (y->right != nullptr) {
      y->right->set_parent(x);
    }
    RbNode* const x_parent = x->parent();
    y->set_parent(x_parent);
    if (x_parent == nullptr) {
      root_ = y;
    } else if (x == x_parent->right) {
      x_parent->right = y;
    } else {
      x_parent->left = y;
    }
    y->right = x;
    x->set_parent(y);

    UpdateNodeAugment(x);
    UpdateNodeAugment(y);
  }

  void InsertFixup(RbNode* const start) {
    RbNode* z = start;
    while (IsRed(z->parent())) {
      RbNode* const p = z->parent();
      RbNode* const g = p->parent();
      if (p == g->left) {
        RbNode* const uncle = g->right;
        if (IsRed(uncle)) {
          p->set_color(RbColor::kBlack);
          uncle->set_color(RbColor::kBlack);
          g->set_color(RbColor::kRed);
          z = g;
        } else {
          if (z == p->right) {
            z = p;
            RotateLeft(z);
          }
          z->parent()->set_color(RbColor::kBlack);
          g->set_color(RbColor::kRed);
          RotateRight(g);
        }
      } else {
        RbNode* const uncle = g->left;
        if (IsRed(uncle)) {
          p->set_color(RbColor::kBlack);
          uncle->set_color(RbColor::kBlack);
          g->set_color(RbColor::kRed);
          z = g;
        } else {
          if (z == p->left) {
            z = p;
            RotateRight(z);
          }
          z->parent()->set_color(RbColor::kBlack);
          g->set_color(RbColor::kRed);
          RotateLeft(g);
        }
      }
    }
    root_->set_color(RbColor::kBlack);
    PropagateNodeAugmentToRoot(z);
  }

  void EraseFixup(RbNode* const start_x, RbNode* const start_x_parent) {
    RbNode* x = start_x;
    RbNode* x_parent = start_x_parent;
    RbNode* last_touched = x_parent;
    while (x != root_ && IsBlack(x)) {
      if (x == x_parent->left) {
        RbNode* w = x_parent->right;
        if (IsRed(w)) {
          w->set_color(RbColor::kBlack);
          x_parent->set_color(RbColor::kRed);
          RotateLeft(x_parent);
          w = x_parent->right;
        }
        if (IsBlack(w->left) && IsBlack(w->right)) {
          w->set_color(RbColor::kRed);
          x = x_parent;
          x_parent = x->parent();
          last_touched = x;
        } else {
          if (IsBlack(w->right)) {
            if (w->left != nullptr) {
              w->left->set_color(RbColor::kBlack);
            }
            w->set_color(RbColor::kRed);
            RotateRight(w);
            w = x_parent->right;
          }
          w->set_color(x_parent->color());
          x_parent->set_color(RbColor::kBlack);
          if (w->right != nullptr) {
            w->right->set_color(RbColor::kBlack);
          }
          RotateLeft(x_parent);
          last_touched = w;
          x = root_;
          x_parent = nullptr;
        }
      } else {
        RbNode* w = x_parent->left;
        if (IsRed(w)) {
          w->set_color(RbColor::kBlack);
          x_parent->set_color(RbColor::kRed);
          RotateRight(x_parent);
          w = x_parent->left;
        }
        if (IsBlack(w->right) && IsBlack(w->left)) {
          w->set_color(RbColor::kRed);
          x = x_parent;
          x_parent = x->parent();
          last_touched = x;
        } else {
          if (IsBlack(w->left)) {
            if (w->right != nullptr) {
              w->right->set_color(RbColor::kBlack);
            }
            w->set_color(RbColor::kRed);
            RotateLeft(w);
            w = x_parent->left;
          }
          w->set_color(x_parent->color());
          x_parent->set_color(RbColor::kBlack);
          if (w->left != nullptr) {
            w->left->set_color(RbColor::kBlack);
          }
          RotateRight(x_parent);
          last_touched = w;
          x = root_;
          x_parent = nullptr;
        }
      }
    }
    if (x != nullptr) {
      x->set_color(RbColor::kBlack);
    }
    PropagateNodeAugmentToRoot(last_touched);
  }

  template <typename SubtreePred, typename NodePred>
  static const T* FindFirstInSubtree(const RbNode* const node,
                                     const SubtreePred& subtree_pred,
                                     const NodePred& node_pred) {
    if (node == nullptr) {
      return nullptr;
    }
    const T* const item = NodeToItem(node);
    if (!subtree_pred(*item)) {
      return nullptr;
    }
    const T* const left_match =
        FindFirstInSubtree(node->left, subtree_pred, node_pred);
    if (left_match != nullptr) {
      return left_match;
    }
    if (node_pred(*item)) {
      return item;
    }
    return FindFirstInSubtree(node->right, subtree_pred, node_pred);
  }
};

}  // namespace protos

#endif  // PROTOS_RBTREE_H_
