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
        vector<Vertex> vertices;
        vector<Edge> edges;
        
        int numVertices() const { return vertices.size(); }
        int numEdges() const { return edges.size(); }

        vector<Edge> sortEdges() const
        {
            if (edges.empty()) return {};

            vector<Edge> unsorted = edges;            
            size_t E = unsorted.size();
            vector<Edge> sorted;

            sorted.push_back(unsorted[0]);
            unsorted.erase(unsorted.begin());

                while (!unsorted.empty())
                {
                    int current_end = sorted.back().end;
                    bool found = false;

                    for (auto it = unsorted.begin(); it != unsorted.end(); ++it)
                    {
                        if (it->origin == current_end)
                        {
                            sorted.push_back(*it);
                            unsorted.erase(it);
                            found = true;
                            break;
                        }
                        else if (it->end == current_end)
                        {
                            Edge flipped = *it;
                            flipped.swapVertices();
                            sorted.push_back(flipped);
                            unsorted.erase(it);
                            found = true;
                            break;
                        }
                    }

                    if (!found)
                    {
                        cerr << "sort error" << endl;
                        break;
                    }
                }  
                if (sorted.size() != E)
                {
                    cerr << "size error" << endl;
                }
            return sorted;
        }

        vector<Vertex> sortVertices() const
        {
            vector<Vertex> unsorted = vertices;
            vector<Edge> edges_list = sortEdges();
            size_t E = edges_list.size();
            vector<Vertex> sorted(E);


            for(size_t e = 0; e < E; e++)
                {
                    for(size_t j = 0; j < E; j++)
                    {
                        if(unsorted[e].id == edges_list[j].origin)
                        {
                            sorted[j] = unsorted[e];
                        }

                    }
                }  

            return sorted;
        }

        bool isValid() const
        {
            int E = numEdges();
            vector<Edge> edges_list = sortEdges();
            vector<Vertex> vertices_list = sortVertices();

            for (int e = 0; e < E; e++)
            {
                if (vertices_list[e].id != edges_list[e].origin)
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