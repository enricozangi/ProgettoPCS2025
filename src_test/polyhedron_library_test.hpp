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

/*
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
*/

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

    using std::numbers::phi;

	vector<Vertex> vertices_list = {
        {0, -1, phi, 0, false},
        {1, 1, phi, 0, false},
        {2, -1, -phi, 0, false},
        {3, 1, -phi, 0, false},
        {4, 0, -1, phi, false},
        {5, 0, 1, phi, false},
        {6, 0, -1, -phi, false},
        {7, 0, 1, -phi, false},
        {8, phi, 0, -1, false},
        {9, phi, 0, 1, false},
        {10, -phi, 0, -1, false},
        {11, -phi, 0, 1, false}
    };

    vector<Edge> edges_list = {
        {0, 0, 5, false}, {1, 0, 1, false}, {2, 0, 11, false}, {3, 0, 10, false}, {4, 0, 7, false},
        {5, 1, 5, false}, {6, 1, 7, false}, {7, 1, 8, false}, {8, 1, 9, false}, {9, 5, 9, false},
        {10, 5, 11, false}, {11, 9, 8, false}, {12, 8, 7, false}, {13, 9, 4, false}, {14, 9, 3, false},
        {15, 4, 3, false}, {16, 5, 4, false}, {17, 4, 11, false}, {18, 2, 4, false}, {19, 2, 11, false},
        {20, 2, 3, false}, {21, 2, 6, false}, {22, 6, 3, false}, {23, 8, 3, false}, {24, 8, 6, false},
        {25, 7, 6, false}, {26, 2, 10, false}, {27, 10, 6, false}, {28, 10, 7, false}, {29, 11, 10, false}
    };

	vector<Face> faces_list = {
		{0, {vertices_list[5], vertices_list[0], vertices_list[11]}, {edges_list[0], edges_list[2], edges_list[10]}},
		{1, {vertices_list[0], vertices_list[5], vertices_list[1]}, {edges_list[0], edges_list[5], edges_list[1]}},
		{2, {vertices_list[5], vertices_list[1], vertices_list[9]}, {edges_list[5], edges_list[8], edges_list[9]}},
		{3, {vertices_list[5], vertices_list[4], vertices_list[9]}, {edges_list[16], edges_list[13], edges_list[9]}},
		{4, {vertices_list[4], vertices_list[5], vertices_list[11]}, {edges_list[16], edges_list[10], edges_list[17]}},
		{5, {vertices_list[4], vertices_list[3], vertices_list[9]}, {edges_list[15], edges_list[14], edges_list[13]}},
		{6, {vertices_list[4], vertices_list[11], vertices_list[2]}, {edges_list[17], edges_list[19], edges_list[18]}},
		{7, {vertices_list[2], vertices_list[3], vertices_list[6]}, {edges_list[20], edges_list[22], edges_list[21]}},
		{8, {vertices_list[6], vertices_list[2], vertices_list[10]}, {edges_list[21], edges_list[26], edges_list[27]}},
		{9, {vertices_list[3], vertices_list[8], vertices_list[6]}, {edges_list[23], edges_list[24], edges_list[22]}},
		{10, {vertices_list[8], vertices_list[9], vertices_list[3]}, {edges_list[11], edges_list[14], edges_list[23]}},
		{11, {vertices_list[9], vertices_list[8], vertices_list[1]}, {edges_list[11], edges_list[7], edges_list[8]}},
		{12, {vertices_list[8], vertices_list[7], vertices_list[1]}, {edges_list[12], edges_list[6], edges_list[7]}},
		{13, {vertices_list[6], vertices_list[7], vertices_list[8]}, {edges_list[25], edges_list[12], edges_list[24]}},
		{14, {vertices_list[7], vertices_list[10], vertices_list[6]}, {edges_list[28], edges_list[27], edges_list[25]}},
		{15, {vertices_list[7], vertices_list[10], vertices_list[0]}, {edges_list[28], edges_list[3], edges_list[4]}},
		{16, {vertices_list[0], vertices_list[10], vertices_list[11]}, {edges_list[3], edges_list[29], edges_list[2]}},
		{17, {vertices_list[4], vertices_list[2], vertices_list[3]}, {edges_list[18], edges_list[20], edges_list[15]}},
		{18, {vertices_list[1], vertices_list[0], vertices_list[7]}, {edges_list[1], edges_list[4], edges_list[6]}},
		{19, {vertices_list[11], vertices_list[2], vertices_list[10]}, {edges_list[19], edges_list[26], edges_list[29]}}
	};

    for(size_t j = 0; j < faces_list.size() - 1; j++)
    {
        Face face_test = faces_list[j];

        vector<Edge> sorted = face_test.sortEdges();
        int E = face_test.numEdges();

        for (size_t i = 0; i < sorted.size() - 1; ++i)
        {
            EXPECT_EQ(sorted[i].end, sorted[(i + 1) % E].origin) << "Error between edge " << i << " and " << (i + 1) % E << " of face " << j << endl;
        }
    }
}
