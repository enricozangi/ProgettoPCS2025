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

    const double phi = (1.0 + sqrt(5.0)) / 2.0;
    const double inv_norm = 1.0 / sqrt(1 + phi * phi);

    vector<Vertex> vertices_list =
    {
        {0, -1 * inv_norm,  phi * inv_norm, 0, false},
        {1,  1 * inv_norm,  phi * inv_norm, 0, false},
        {2, -1 * inv_norm, -phi * inv_norm, 0, false},
        {3,  1 * inv_norm, -phi * inv_norm, 0, false},
        {4, 0, -1 * inv_norm,  phi * inv_norm, false},
        {5, 0,  1 * inv_norm,  phi * inv_norm, false},
        {6, 0, -1 * inv_norm, -phi * inv_norm, false},
        {7, 0,  1 * inv_norm, -phi * inv_norm, false},
        {8,  phi * inv_norm, 0, -1 * inv_norm, false},
        {9,  phi * inv_norm, 0,  1 * inv_norm, false},
        {10, -phi * inv_norm, 0, -1 * inv_norm, false},
        {11, -phi * inv_norm, 0,  1 * inv_norm, false}
    }
    ;

    vector<Edge> edges_list =
    {
        {0, 0, 11, false}, {1, 0, 5, false},  {2, 0, 1, false},  {3, 0, 7, false},  {4, 0, 10, false},
        {5, 0, 2, false},  {6, 1, 5, false},  {7, 1, 9, false},  {8, 1, 8, false},  {9, 1, 7, false},
        {10, 1, 0, false}, {11, 2, 10, false},{12, 2, 3, false}, {13, 2, 6, false}, {14, 2, 0, false},
        {15, 3, 2, false}, {16, 3, 6, false}, {17, 3, 8, false}, {18, 3, 9, false}, {19, 3, 1, false},
        {20, 4, 10, false},{21, 4, 11, false},{22, 4, 5, false}, {23, 4, 6, false}, {24, 4, 2, false},
        {25, 5, 11, false},{26, 5, 9, false}, {27, 6, 10, false},{28, 6, 3, false}, {29, 6, 4, false}
    };


    vector<Face> faces_list =
    {
        {0, {vertices_list[0], vertices_list[11], vertices_list[5]}, {edges_list[0], edges_list[25], edges_list[1]}},
        {1, {vertices_list[0], vertices_list[5], vertices_list[1]}, {edges_list[1], edges_list[6], edges_list[2]}},
        {2, {vertices_list[0], vertices_list[1], vertices_list[7]}, {edges_list[2], edges_list[9], edges_list[3]}},
        {3, {vertices_list[0], vertices_list[7], vertices_list[2]}, {edges_list[3], edges_list[13], edges_list[5]}},
        {4, {vertices_list[0], vertices_list[2], vertices_list[10]}, {edges_list[5], edges_list[11], edges_list[4]}},

        {5, {vertices_list[1], vertices_list[5], vertices_list[9]}, {edges_list[6], edges_list[26], edges_list[7]}},
        {6, {vertices_list[1], vertices_list[9], vertices_list[8]}, {edges_list[7], edges_list[8], edges_list[8]}},
        {7, {vertices_list[1], vertices_list[8], vertices_list[3]}, {edges_list[8], edges_list[17], edges_list[19]}},
        {8, {vertices_list[1], vertices_list[3], vertices_list[7]}, {edges_list[19], edges_list[9], edges_list[9]}},

        {9, {vertices_list[2], vertices_list[6], vertices_list[10]}, {edges_list[13], edges_list[27], edges_list[11]}},
        {10, {vertices_list[2], vertices_list[3], vertices_list[6]}, {edges_list[12], edges_list[16], edges_list[13]}},
        {11, {vertices_list[2], vertices_list[7], vertices_list[3]}, {edges_list[13], edges_list[3], edges_list[12]}},

        {12, {vertices_list[3], vertices_list[8], vertices_list[9]}, {edges_list[17], edges_list[18], edges_list[18]}},
        {13, {vertices_list[3], vertices_list[9], vertices_list[5]}, {edges_list[18], edges_list[26], edges_list[6]}},

        {14, {vertices_list[4], vertices_list[10], vertices_list[11]}, {edges_list[20], edges_list[0], edges_list[21]}},
        {15, {vertices_list[4], vertices_list[11], vertices_list[5]}, {edges_list[21], edges_list[25], edges_list[22]}},
        {16, {vertices_list[4], vertices_list[5], vertices_list[9]}, {edges_list[22], edges_list[26], edges_list[26]}},
        {17, {vertices_list[4], vertices_list[9], vertices_list[3]}, {edges_list[26], edges_list[18], edges_list[28]}},
        {18, {vertices_list[4], vertices_list[3], vertices_list[6]}, {edges_list[28], edges_list[16], edges_list[23]}},
        {19, {vertices_list[4], vertices_list[6], vertices_list[10]}, {edges_list[23], edges_list[27], edges_list[20]}}
    };

    for(size_t j = 0; j < faces_list.size() - 1; j++)
    {
        Face face_test = faces_list[j];

        vector<Edge> sorted = face_test.sortEdges();
        int E = face_test.numEdges();

        for (size_t i = 0; i < sorted.size() - 1; ++i)
        {
            EXPECT_EQ(sorted[i].end, sorted[(i + 1) % E].origin);
            cerr << "Error between edge " << i << " and " << i + 1 << " of face " << j << endl;
        }
    }
}
