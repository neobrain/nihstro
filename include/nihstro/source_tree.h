// Copyright 2014 Tony Wasserka
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the owner nor the names of its contributors may
//       be used to endorse or promote products derived from this software
//       without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <list>
#include <string>

#include <boost/optional.hpp>
#include <boost/variant/recursive_wrapper.hpp>

namespace nihstro {

struct SourceTreeIterator;


struct Node;
// SequenceContainer
struct SourceTree {
    SourceTree() = default;
    SourceTree(const SourceTree& oth);

    std::string code;

    struct {
        std::string filename;
    } file_info;

    SourceTree* parent = nullptr;

    // ordered with respect to "position"
    std::list<Node> children;

    SourceTreeIterator begin();
    SourceTreeIterator end();

    // Attach the given tree, changing the child's owner to *this.
    SourceTree& Attach(SourceTree tree, std::string::difference_type offset);
};

struct Node {
    SourceTree tree;

    std::string::difference_type offset_within_parent; // within "code"
};

inline SourceTree::SourceTree(const SourceTree& oth) : code(oth.code), file_info(oth.file_info), parent(oth.parent), children(oth.children) {
    for (auto& child : children)
        child.tree.parent = this;
}

inline SourceTree& SourceTree::Attach(SourceTree tree, std::string::difference_type offset) {
    tree.parent = this;
    children.push_back(Node{tree, offset});
    return *this;
}

// RandomAccessIterator
struct SourceTreeIterator {
    using difference_type = std::string::iterator::difference_type;
    using reference = std::string::iterator::reference;
    using value_type = std::string::iterator::value_type;
    using pointer = std::string::iterator::pointer;
    using iterator_category = std::random_access_iterator_tag;
    
    SourceTreeIterator() {
    }

    SourceTreeIterator(SourceTree& tree) : tree(&tree), position(tree.code.begin()), node_iterator(tree.children.begin()) {
        UpdateChildIterator();
    }

    SourceTreeIterator(const SourceTreeIterator&) = default;

    SourceTreeIterator& operator += (difference_type n) {
        if (n > 0) {
            while (n) {
                if (child_iterator) {
                    auto remaining_to_child = node_iterator->offset_within_parent - (position - tree->code.begin());
                    if (remaining_to_child >= n) {
                        // If the next child is more than n steps away, increase position by n and return
                        // TODO: Should we make sure that we don't end up out-of-bounds here?
                        position += n;
                        UpdateNodeIterator();
                        break;
                    } else {
                        // Otherwise, move current position to the child if it isn't there already
                        position += remaining_to_child;
                        n -= remaining_to_child;
                        UpdateNodeIterator();
                    }

                    if (child_iterator->get().StepsRemaining() > n) {
                        // If child is larger than n, advance child by n and return
                        child_iterator->get() += n;
                        break;
                    } else {
                        // else step out of the child and increment next child iterator by one
                        n -= child_iterator->get().StepsRemaining();
                        if (node_iterator != tree->children.end())
                            node_iterator++;
                        UpdateChildIterator();
                    }
                } else {
                    // TODO: Should we make sure that we don't end up out-of-bounds here?
                    position += n;
                    UpdateNodeIterator();
                    break;
                }
            }
        } else if (n < 0) {
            // Reduce to n>0 case by starting from begin()
            n = (*this - tree->begin()) + n;
            *this = tree->begin() + n;
        }
        return *this;
    }

    SourceTreeIterator& operator -= (difference_type n) {
        *this += -n;
        return *this;
    }

    difference_type operator -(SourceTreeIterator it) const {
        return this->StepsGone() - it.StepsGone();
    }

    bool operator < (const SourceTreeIterator& it) const {
        return std::distance(*this, it) > 0;
    }

    bool operator <= (const SourceTreeIterator& it) const {
        return std::distance(*this, it) >= 0;
    }

    bool operator > (const SourceTreeIterator& it) const {
        return !(*this <= it);
    }

    bool operator >= (const SourceTreeIterator& it) const {
        return !(*this < it);
    }

    bool operator == (const SourceTreeIterator& it) const {
        return (*this <= it) && !(*this < it);
    }

    bool operator != (const SourceTreeIterator& it) const {
        return !(*this == it);
    }

    reference operator* () {
        return (*this)[0];
    }

    SourceTreeIterator operator++ () {
        *this += 1;
        return *this;
    }

    SourceTreeIterator operator++ (int) {
        auto it = *this;
        *this += 1;
        return it;
    }

    SourceTreeIterator operator +(difference_type n) const {
        SourceTreeIterator it2 = *this;
        it2 += n;
        return it2;
    }

    SourceTreeIterator operator -(SourceTreeIterator::difference_type n) const {
        return *this + (-n);
    }

    reference operator [] (difference_type n) {
        auto it = (*this + n);
        if (it.WithinChild())
            return it.child_iterator->get()[0];
        else return *it.position;
    }

    // Get line number (one-based) within "tree"
    unsigned GetLineNumber() const {
        // Adding one for natural (i.e. one-based) line numbers
        return std::count(tree->code.begin(), position, '\n') + 1;
    }

    // Get line number (one-based) within the tree of the current child
    unsigned GetCurrentLineNumber() const {
        if (WithinChild())
            return child_iterator->get().GetCurrentLineNumber();

        return GetLineNumber();
    }

    const std::string GetCurrentFilename() const {
        if (WithinChild())
            return child_iterator->get().GetCurrentFilename();

        return tree->file_info.filename;
    }

    SourceTreeIterator GetParentIterator(const SourceTree* reference_tree) const {
        if (tree == reference_tree) {
            return *this;
        } else {
            return child_iterator->get().GetParentIterator(reference_tree);
        }
    }

    SourceTree* GetCurrentTree() {
        if (WithinChild())
            return child_iterator->get().GetCurrentTree();
        else
            return tree;
    }

private:
    difference_type StepsRemaining() const {
        return std::distance(*this, tree->end());
    }

    difference_type StepsGone() const {
        auto it = tree->begin();

        difference_type diff = 0;

        // Advance reference iterator starting from the beginning until we reach *this,
        // making sure that both the main position and the child iterator match.
        while (it.position != position ||
               ((bool)it.child_iterator ^ (bool)child_iterator) ||
               (it.child_iterator && child_iterator && it.child_iterator->get() != child_iterator->get())) {
            // Move to next child (if there is one), or abort if we reach the reference position
            if (it.child_iterator) {
                auto distance_to_child = std::min(it.node_iterator->offset_within_parent - (it.position -it.tree->code.begin() ), position - it.position);

                // Move to child or this->position
                diff += distance_to_child;
                it.position += distance_to_child;

                if (it.position - it.tree->code.begin() == it.node_iterator->offset_within_parent) {
                    if (node_iterator != tree->children.end() && it.node_iterator == node_iterator) {
                        return diff + (child_iterator->get() - it.child_iterator->get());
                    } else {
                        // Move out of child
                        diff += it.child_iterator->get().StepsRemaining();
                    }
                } else {
                    // We moved to this->position => done
                    return diff;
                }

                // Move to next child
                if (it.node_iterator != it.tree->children.end()) {
                    it.node_iterator++;
                    it.UpdateChildIterator();
                }
            } else {
                // no child remaining, hence just move to the given position
                return diff + (position - it.position);
            }
        }

        return diff;
    }

    bool WithinChild() const {
        return child_iterator && position - tree->code.begin() == node_iterator->offset_within_parent;
    }

    void UpdateChildIterator() {
        if (node_iterator != tree->children.end())
            child_iterator = boost::recursive_wrapper<SourceTreeIterator>(node_iterator->tree);
        else
            child_iterator = boost::none;
    }

    void UpdateNodeIterator() {
        // Move to the first node which is at the cursor or behind it
        while (node_iterator != tree->children.end() && node_iterator->offset_within_parent < std::distance(tree->code.begin(), position)) {
            node_iterator++;
            UpdateChildIterator();
        }
    }

    SourceTree* tree;
    std::string::iterator position;

    boost::optional<boost::recursive_wrapper<SourceTreeIterator>> child_iterator; // points to current or next child
    std::list<Node>::iterator node_iterator; // points to current or next node

    friend struct SourceTree;
};

inline SourceTreeIterator operator +(SourceTreeIterator::difference_type n, const SourceTreeIterator& it) {
    return it + n;
}

inline SourceTreeIterator operator -(SourceTreeIterator::difference_type n, const SourceTreeIterator& it) {
    return it - n;
}

inline SourceTreeIterator SourceTree::begin() {
    return SourceTreeIterator(*this);
}

inline SourceTreeIterator SourceTree::end() {
    auto it = SourceTreeIterator(*this);
    it.position = code.end();
    it.node_iterator = children.end();
    it.child_iterator = boost::none;
    return it;
}

} // namespace
