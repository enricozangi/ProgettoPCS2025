#pragma once

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

namespace polyhedron_library
{

    struct Vertex
    {
        int id;
        double x, y, z;
        bool ShortPath;
    };
    
    struct Edge
    {
        int id;
        int origin;
        int end;
        bool ShortPath;

        void swapVertices()
        {
            swap(origin, end);
        }
    };
    
    struct Face
    {
        int id;
        vector<int> vertices;
        vector<Edge> edges;
        
        int numVertices() const { return vertices.size(); }
        int numEdges() const { return edges.size(); }

        vector<Edge> sortEdges() const
        {
            int E = numEdges();
            vector<Edge> edges_copy = edges;
            vector<Edge> sorted_list;

            for(int e = 0; e < E; e++)
            {
                for(int j = e + 1; j < E; j++)
                {
                    int v0 = edges_copy[e].end;
                    int next_v1 = edges_copy[j].origin;
                    int next_v2 = edges_copy[j].end;

                    if(v0 == next_v1)
                    {
                        sorted_list.push_back(edges_copy[j]);
                    }

                    if(v0 == next_v2)
                    {
                        edges_copy[j].swapVertices();
                        sorted_list.push_back(edges_copy[j]);
                    }
                }
            }
            return sorted_list;
        }

        bool isValid() const
        {
            int E = numEdges();
            vector<Edge> edges_list= sortEdges();

            for (int e = 0; e < E; e++)
            {
                if (vertices[e] != edges_list[e].origin)
                {
                    cerr << "Vertices and edges don't match." << endl;
                    return false;
                }
            }

            Edge e0 = edges_list[0];

            if(e0.end != edges_list[1].origin && e0.end != edges_list[1].end)
            {
                e0.swapVertices();
            }

            for (int e = 0; e < E; e++)
            {
                int v1 = edges_list[e].end;

                int next_v1 = edges_list[(e + 1) % E].origin;
                int next_v2 = edges_list[(e + 1) % E].end;

                if (v1 != next_v1 && v1 != next_v2)
                {
                    cerr << "Discontinuity in the edges." << endl;
                    return false;
                }
            }
            return true;
        }
    };

    struct Polyhedron
    {
        int id;
        vector<Vertex> vertices;
        vector<Edge> edges;
        vector<Face> faces;
        
        int numVertices() const { return vertices.size(); }
        int numEdges() const { return edges.size(); }
        int numFaces() const { return faces.size(); }

        bool isValid() const
        {
            int E = numFaces();

            for (int e = 0; e < E; e++)
            {
                if (!faces[e].isValid())
                {
                    cerr << "Invalid polyhedron." << endl;
                    return false;
                }
            }
            return true;
        }
    };

}