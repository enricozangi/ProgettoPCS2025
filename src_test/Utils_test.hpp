#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include "Utils.hpp"
#include "polyhedron_library.hpp"
#include "triangolazione.hpp"

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

TEST(DualTest, Goldbergb1)
{
    Polyhedron p = tetraedro();
    Polyhedron g = dualPolyhedron(p);
    EXPECT_TRUE(g.isValid());
    EXPECT_EQ(g.numVertices(), p.numFaces()) << "mismatch between vertices and faces";
    EXPECT_EQ(g.numEdges(), p.numEdges()) << "mismatch bitween edges";
    EXPECT_EQ(g.numFaces(), p.numVertices()) << "mismatch between faces and vertices";
}
TEST(ShortestPathTest, SimpleGraph) {

    Polyhedron poly;

    poly.vertices = {
        {0, 0.0, 0.0, 0.0, false},
        {1, 1.0, 1.0, 100.0, false},
        {2, 2.0, 0.0, 0.0, false},
        {3, 3.0, 0.0, 0.0, false},
        {4, 3.0, 1.0, 0.0, false},
        {5, 6.0, 2.0, 1.0, false}
    };

    poly.edges = {
        {0, 0, 1, false},
        {1, 1, 5, false},
        {2, 0, 3, false},
        {3, 3, 4, false},
        {4, 4, 5, false}
    };

    // Calcolo cammino più breve tra 0 e 3
    shortestPath(poly, 0, 5);

    // Verifica dei vertici nel cammino minimo: devono essere 0, 1, 3
    EXPECT_TRUE(poly.vertices[0].ShortPath);
    EXPECT_FALSE(poly.vertices[1].ShortPath);
    EXPECT_TRUE(poly.vertices[3].ShortPath);
    EXPECT_FALSE(poly.vertices[2].ShortPath);
    EXPECT_TRUE(poly.vertices[4].ShortPath);
    EXPECT_TRUE(poly.vertices[5].ShortPath);

    // Verifica degli archi nel cammino minimo: devono essere 0 (0-1) e 1 (1-3)
    EXPECT_FALSE(poly.edges[0].ShortPath);
    EXPECT_FALSE(poly.edges[1].ShortPath);
    EXPECT_TRUE(poly.edges[2].ShortPath);
    EXPECT_TRUE(poly.edges[3].ShortPath);
    EXPECT_TRUE(poly.edges[4].ShortPath);
}

TEST(DualTest, Goldbergb2)
{
    Polyhedron p = tetraedro();
    Polyhedron ptr = triangolazione(p, 2);
    Polyhedron g = dualPolyhedron(ptr);
    EXPECT_TRUE(g.isValid());
    EXPECT_EQ(g.numVertices(), ptr.numFaces()) << "mismatch between vertices and faces";
    EXPECT_EQ(g.numEdges(), ptr.numEdges()) << "mismatch bitween edges";
    EXPECT_EQ(g.numFaces(), ptr.numVertices()) << "mismatch between faces and vertices";
}

TEST(DualTest, Goldbergb3)
{
    Polyhedron p = tetraedro();
    Polyhedron ptr = triangolazione(p, 3);
    Polyhedron g = dualPolyhedron(ptr);
    EXPECT_TRUE(g.isValid());
    EXPECT_EQ(g.numVertices(), ptr.numFaces()) << "mismatch between vertices and faces";
    EXPECT_EQ(g.numEdges(), ptr.numEdges()) << "mismatch bitween edges";
    EXPECT_EQ(g.numFaces(), ptr.numVertices()) << "mismatch between faces and vertices";
}