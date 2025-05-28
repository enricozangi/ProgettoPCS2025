#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include "polyhedron_library.hpp"
#include "Utils.hpp"

using namespace std;
using namespace polyhedron_library;

TEST(isValidTest, ValidFaceTest)
{
    vector<Vertex> vertices_list = {
        {0, 0.0, 0.0, 0.0, false},
        {1, 1.0, 0.0, 0.0, false},
        {2, 0.0, 1.0, 0.0, false}
    };

    Edge e0{0, 0, 1, 0};
    Edge e1{1, 1, 2, 0};
    Edge e2{2, 2, 0, 0};

    vector<Edge> edges_list = {e0, e1, e2};

    Face face1{0, vertices_list, edges_list};

    EXPECT_TRUE(face1.isValid());
}

TEST(isValidTest, MismatchedListsTest)
{
    vector<Vertex> vertices_list = {
        {1, 0.0, 0.0, 0.0, false},
        {0, 1.0, 0.0, 0.0, false},
        {2, 0.0, 1.0, 0.0, false}
    };

    Edge e0{0, 0, 1, 0};
    Edge e1{1, 1, 2, 0};
    Edge e2{2, 2, 0, 0};

    vector<Edge> edges_list = {e0, e1, e2};

    Face face2{1, vertices_list, edges_list};

    EXPECT_FALSE(face2.isValid());
}

TEST(isValidTest, DiscontinuityTest)
{
    vector<Vertex> vertices_list = {
        {0, 0.0, 0.0, 0.0, false},
        {1, 1.0, 0.0, 0.0, false},
        {2, 0.0, 1.0, 0.0, false}
    };

    Edge e0{0, 0, 1, 0};
    Edge e1{1, 1, 2, 0};

    vector<Edge> edges_list = {e0, e1};

    Face face3{2, vertices_list, edges_list};

    EXPECT_FALSE(face3.isValid());
}

// test for sorting function

TEST(SortTest, SortEdgesTest)
{
    vector<Vertex> vertices_list = {
        {0, 0.0, 0.0, 0.0, false},
        {1, 1.0, 0.0, 0.0, false},
        {4, 0.0, 1.0, 0.0, false}
    };

    vector<Edge> edges_list = {
        {0, 0, 1, false},
        {1, 0, 4, false},
        {5, 1, 4, false}
    };

    Face face_test = {0, vertices_list, edges_list};

    vector<Edge> sorted = face_test.sortEdges();
    int E = face_test.numEdges();

    for (size_t i = 0; i < sorted.size() - 1; ++i)
    {
        EXPECT_EQ(sorted[i].end, sorted[(i + 1) % E].origin);
        cerr << "Error between edge " << i << " and " << i + 1 << endl;
    }
}
