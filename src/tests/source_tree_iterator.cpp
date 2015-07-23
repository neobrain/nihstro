// Copyright 2015 Tony Wasserka
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

#include <iostream>
#include "nihstro/source_tree.h"

#define BOOST_TEST_MODULE SourceTreeIterator
#include <boost/test/unit_test.hpp>

#include <iterator>

namespace std {

std::ostream& operator << (std::ostream& os, const nihstro::SourceTree& tree) {
    std::string::const_iterator it = tree.code.cbegin();
    for (auto& child : tree.children) {
        os << "\"";
        os << std::string(it, tree.code.cbegin() + child.offset_within_parent);
        os << "\"";
        os << " { ";
        os << child.tree;
        os << " } ";
        it = tree.code.cbegin() + child.offset_within_parent;
    }
    os << "\"" << std::string(it, tree.code.end()) << "\"";
    return os;
}

}

// Utility function to manually flatten the given tree into a string
static std::string FlattenTree(const nihstro::SourceTree& tree) {
    std::string ret;
    std::string::const_iterator it = tree.code.cbegin();
    for (auto& child : tree.children) {
        ret += std::string(it, tree.code.cbegin() + child.offset_within_parent);
        ret += FlattenTree(child.tree);
        it = tree.code.cbegin() + child.offset_within_parent;
    }
    ret += std::string(it, tree.code.end());
    return ret;
}

// Utility function to manually determine the size of the given tree
static std::string::size_type TreeSize(const nihstro::SourceTree& tree) {
    std::string::size_type ret = 0;
    for (auto& child : tree.children) {
        ret += TreeSize(child.tree);
    }
    ret += tree.code.length();
    return ret;
}

#define CHECK_TREE(tree) do { \
    /* Check length */ \
    BOOST_CHECK_EQUAL(tree.end() - tree.begin(), TreeSize(tree)); \
    BOOST_CHECK_EQUAL(std::distance(tree.begin(), tree.end()), TreeSize(tree)); \
    /* Check forward iteration */ \
    std::string flattened_tree; \
    for (auto& val : tree) \
        flattened_tree += val; \
    auto reference_flattened_tree = FlattenTree(tree); \
    BOOST_CHECK_EQUAL(flattened_tree, reference_flattened_tree); \
    BOOST_CHECK_EQUAL_COLLECTIONS(flattened_tree.begin(), flattened_tree.end(), \
                                  reference_flattened_tree.begin(), reference_flattened_tree.end()); \
    \
    /* Check reverse iteration */ \
    flattened_tree.clear(); \
    for (auto it = tree.end() - 1;; it -= 1) { \
        flattened_tree += *it; \
        if (it == tree.begin()) \
            break; \
    } \
    std::reverse(reference_flattened_tree.begin(), reference_flattened_tree.end()); \
    BOOST_CHECK_EQUAL(flattened_tree, reference_flattened_tree); \
    BOOST_CHECK_EQUAL_COLLECTIONS(flattened_tree.begin(), flattened_tree.end(), \
                                  reference_flattened_tree.begin(), reference_flattened_tree.end()); \
 \
} while (false)

BOOST_AUTO_TEST_CASE(simple_tree) {
    nihstro::SourceTree tree;

    tree.code = "a b c";

    CHECK_TREE(tree);
}

BOOST_AUTO_TEST_CASE(nested_tree) {
    nihstro::SourceTree tree;
    nihstro::SourceTree child1;
    nihstro::SourceTree child2;

    tree.code = "aXbXc";
    child1.code = "child1";
    child2.code = "child2";
	tree.Attach(child1, 1).Attach(child2, 3);

    CHECK_TREE(tree);
}

BOOST_AUTO_TEST_CASE(deep_tree) {
    nihstro::SourceTree tree;
    nihstro::SourceTree child1;
    nihstro::SourceTree child1_child1;
    nihstro::SourceTree child1_child2;
    nihstro::SourceTree child1_child2_child1;
    nihstro::SourceTree child1_child3;
    nihstro::SourceTree child2;
    nihstro::SourceTree child3;
    nihstro::SourceTree child3_child1;
    nihstro::SourceTree child4;

    tree.code = "aaaXaaaXaaaXaaaXaaa";
    child1.code = "FirstChild:bbbXbbbXbbbXbbb\n";
    child1_child1.code = "FirstSubchildOfChild1:ccc";
    child1_child2.code = "SecondSubchildOfChild1:dddXddd";
    child1_child2_child1.code = "FirstSubsubchildOfSubchild2OfChild1:eee";
    child1_child3.code = "ThirdSubchildOfChild1:fff";
    child2.code = "SecondChild:ggg\n";
    child3.code = "ThirdChild:hhhXhhh\n";
    child3_child1.code = "FirstSubchildOfChild3:iii";
    child4.code = "FourthChild:jjj\n";

    child1_child2.Attach(child1_child2_child1, 26);
    child1.Attach(child1_child1, 14).Attach(child1_child2, 18).Attach(child1_child3, 22);
    child3.Attach(child3_child1, 14);
    tree.Attach(child1, 3).Attach(child2, 7).Attach(child3, 11).Attach(child4, 15);

    CHECK_TREE(tree);
}

BOOST_AUTO_TEST_CASE(subtree_at_begin_and_end) {
    nihstro::SourceTree tree;
    nihstro::SourceTree child1;
    tree.code = "aaa";
    child1.code = "bbb";

    tree.Attach(child1, 0);
    CHECK_TREE(tree);

    tree.children.clear();
    tree.Attach(child1, tree.code.length());
    CHECK_TREE(tree);
}
