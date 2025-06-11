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

// Test per verificare la validità del tetraedro
TEST(PolyTest, TetrahedronTest)
{
    Polyhedron t = tetraedro();
    EXPECT_TRUE(t.isValid());
}

// Test per verificare la validità dell'ottaedro
TEST(PolyTest, OctahedronTest)
{
    Polyhedron o = ottaedro();
    EXPECT_TRUE(o.isValid());
}

// Test per verificare la validità dell'icosaedro
TEST(PolyTest, IcosahedronTest)
{
    Polyhedron i = icosaedro();
    EXPECT_TRUE(i.isValid());
}

// Test per verificare la normalizzazione di un vertice con norma non nulla
TEST(NormalizeTest, NonZeroNorm)
{
    Vertex v = {0, 1.3, -5.9, 2.1, false};
    normalize(v);
    EXPECT_NEAR(norm(v), 1.0, 1e-3);
}

// Test per verificare la normalizzazione di un vertice con norma nulla
TEST(NormalizeTest, ZeroNorm)
{
    Vertex v = {1, 0, 0, 0, false};
    normalize(v);
    EXPECT_NEAR(norm(v), 0.0, 1e-3);
}

// Test per verificare le proprietà di Goldberg del poliedro duale (b=1)
TEST(DualTest, Goldbergb1)
{
    Polyhedron p = tetraedro();
    Polyhedron g = dualPolyhedron(p);
    EXPECT_TRUE(g.isValid());
    EXPECT_EQ(g.numVertices(), p.numFaces()) << "mismatch between vertices and faces";
    EXPECT_EQ(g.numEdges(), p.numEdges()) << "mismatch bitween edges";
    EXPECT_EQ(g.numFaces(), p.numVertices()) << "mismatch between faces and vertices";
}

// Test per calcolare il cammino minimo in un grafo semplice
TEST(ShortestPathTest, SimpleGraph)
{

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

// Test per calcolare il cammino minimo in un tetraedro
TEST(ShortestPathTest, tetrahedronPath)
{
    Polyhedron p = tetraedro();
    shortestPath(p, 0, 1);
    EXPECT_TRUE(p.vertices[0].ShortPath);
    EXPECT_TRUE(p.vertices[1].ShortPath);
    EXPECT_FALSE(p.vertices[2].ShortPath);
    EXPECT_FALSE(p.vertices[3].ShortPath);
    EXPECT_TRUE(p.edges[0].ShortPath);
    EXPECT_FALSE(p.edges[1].ShortPath);
    EXPECT_FALSE(p.edges[2].ShortPath);
    EXPECT_FALSE(p.edges[3].ShortPath);
}

// Test per verificare il comportamento con un grafo sconnesso
TEST(ShortestPathTest, DisconnectedGraph)
{
    Polyhedron poly;

    poly.vertices = {
        {0, 0.0, 0.0, 0.0, false},
        {1, 1.0, 0.0, 0.0, false},
        {2, 2.0, 0.0, 0.0, false},
        {3, 3.0, 0.0, 0.0, false}
    };

    poly.edges = {
        {0, 0, 1, false},
        {1, 2, 3, false}
    };

    shortestPath(poly, 0, 3);

    EXPECT_FALSE(poly.vertices[0].ShortPath);
    EXPECT_FALSE(poly.vertices[3].ShortPath);
    EXPECT_FALSE(poly.vertices[1].ShortPath);
    EXPECT_FALSE(poly.vertices[2].ShortPath);

    EXPECT_FALSE(poly.edges[0].ShortPath);
    EXPECT_FALSE(poly.edges[1].ShortPath);
}

// Test per calcolare il cammino minimo in un grafo complesso con percorsi multipli
TEST(ShortestPathTest, ComplexGraphMultiplePaths)
{
    Polyhedron poly;

    poly.vertices = {
        {0, 0.0, 0.0, 0.0, false},
        {1, 1.0, 0.0, 0.0, false},
        {2, 2.0, 0.0, 0.0, false},
        {3, 1.0, 1.0, 0.0, false},
        {4, 2.0, 1.0, 0.0, false},
        {5, 3.0, 0.0, 0.0, false}
    };

    poly.edges = {
        {0, 0, 1, false},
        {1, 1, 2, false},
        {2, 0, 3, false},
        {3, 3, 4, false},
        {4, 4, 5, false},
        {5, 2, 5, false},
        {6, 1, 3, false}
    };

    shortestPath(poly, 0, 5);

    EXPECT_TRUE(poly.vertices[0].ShortPath);
    EXPECT_TRUE(poly.vertices[1].ShortPath);
    EXPECT_TRUE(poly.vertices[2].ShortPath);
    EXPECT_TRUE(poly.vertices[5].ShortPath);
    EXPECT_FALSE(poly.vertices[3].ShortPath);
    EXPECT_FALSE(poly.vertices[4].ShortPath);

    EXPECT_TRUE(poly.edges[0].ShortPath);
    EXPECT_TRUE(poly.edges[1].ShortPath);
    EXPECT_TRUE(poly.edges[5].ShortPath);
    EXPECT_FALSE(poly.edges[2].ShortPath);
    EXPECT_FALSE(poly.edges[3].ShortPath);
    EXPECT_FALSE(poly.edges[4].ShortPath);
    EXPECT_FALSE(poly.edges[6].ShortPath);
}

// Test per verificare le proprietà del poliedro duale con triangolazione (b=2)
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

// Test per verificare le proprietà di Goldberg del poliedro duale con triangolazione (b=3)
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

// Test per calcolare numero di lati e distanza totale del cammino minimo
TEST(ShortPathInfoTest, CalcolaNumeroLatiEDistanzaTotale) {
    // Crea un poliedro semplice con 3 vertici e 2 edge
    Polyhedron poly;

    // Aggiungi vertici
    poly.vertices.push_back({0, 0.0, 0.0, 0.0, false});
    poly.vertices.push_back({1, 1.0, 0.0, 0.0, false});
    poly.vertices.push_back({2, 0.0, 1.0, 0.0, false});

    // Aggiungi edge
    poly.edges.push_back({0, 0, 1, true});  // Parte dello shortest path
    poly.edges.push_back({1, 1, 2, false}); // Non parte dello shortest path

    // Calcola le informazioni sullo shortest path
    int edgeCount = 0;
    double totalDistance = 0.0;
    for (const auto& edge : poly.edges) {
        if (edge.ShortPath) {
            edgeCount++;
            const Vertex& v1 = poly.vertices[edge.origin];
            const Vertex& v2 = poly.vertices[edge.end];
            totalDistance += distanza(v1, v2);
        }
    }

    // Verifica che il numero di edge nello shortest path sia corretto
    EXPECT_EQ(edgeCount, 1);

    // Verifica che la distanza totale sia corretta
    EXPECT_DOUBLE_EQ(totalDistance, 1.0);
}