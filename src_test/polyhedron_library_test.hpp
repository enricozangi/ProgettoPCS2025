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
    vector<int> vertices_list = {0, 1, 2};

    Edge e0{0, 0, 1, 0};
    Edge e1{1, 1, 2, 0};
    Edge e2{2, 2, 0, 0};

    vector<Edge> edges_list = {e0, e1, e2};

    Face face1{0, vertices_list, edges_list};

    EXPECT_TRUE(face1.isValid());
}

TEST(isValidTest, MismatchedListsTest)
{
    vector<int> vertices_list = {1, 0, 2};

    Edge e0{0, 0, 1, 0};
    Edge e1{1, 1, 2, 0};
    Edge e2{2, 2, 0, 0};

    vector<Edge> edges_list = {e0, e1, e2};

    Face face2{1, vertices_list, edges_list};

    EXPECT_FALSE(face2.isValid());
}

TEST(isValidTest, DiscontinuityTest)
{
    vector<int> vertices_list = {0, 1, 2};

    Edge e0{0, 0, 1, 0};
    Edge e1{1, 1, 2, 0};

    vector<Edge> edges_list = {e0, e1};

    Face face3{2, vertices_list, edges_list};

    EXPECT_FALSE(face3.isValid());
}

TEST(NormalizeTest, NonZeroNorm)
{
    Vertex v = {0, 1.3, -5.9, 2.1};
    Vertex v_norm = normalize(v);
    EXPECT_NEAR(norm(v_norm), 1.0, 1e-3);
}

TEST(NormalizeTest, ZeroNorm)
{
    Vertex v = {1, 0, 0, 0};
    Vertex v_norm = normalize(v);
    EXPECT_NEAR(norm(v_norm), 0.0, 1e-3);
}
