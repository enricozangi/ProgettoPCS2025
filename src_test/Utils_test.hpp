#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include "Utils.hpp"
#include "polyhedron_library.hpp"

using namespace std;
using namespace polyhedron_library;

// tests to check correctness of polyhedra

TEST(PolyTest, TetrahedronTest)
{
    Polyhedron t = tetraedro();
    EXPECT_TRUE(t.isValid());
}

TEST(PolyTest, OctahedronTest)
{
    Polyhedron o = ottaedro();
    EXPECT_TRUE(o.isValid());
}

TEST(PolyTest, IcosahedronTest)
{
    Polyhedron i = icosaedro();
    EXPECT_TRUE(i.isValid());
}

// tests to check correctness of norm and normalize function

TEST(NormalizeTest, NonZeroNorm)
{
    Vertex v = {0, 1.3, -5.9, 2.1, false};
    normalize(v);
    EXPECT_NEAR(norm(v), 1.0, 1e-3);
}

TEST(NormalizeTest, ZeroNorm)
{
    Vertex v = {1, 0, 0, 0, false};
    normalize(v);
    EXPECT_NEAR(norm(v), 0.0, 1e-3);
}