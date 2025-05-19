#pragma once

#include <iostream>
#include <vector>

using namespace std;

namespace polyhedron_library
{

    // vertices (Cells0D)

    struct Vertex
    {
        int id;
        double x, y, z;
        bool ShortPath;
    };
    
    // edges (Cells1D)

    struct Edge
    {
        int id;
        int origin;
        int end;
        bool ShortPath;
    };
    
    // faces (Cells2D)

    struct Face
    {
        int id;
        vector<int> vertices;
        vector<Edge> edges;
        
        int numVertices() const
        {
            return vertices.size();
        }

        int numEdges() const
        {
            return edges.size();
        }

        bool isValid() const
        {
            int E = numEdges();

            for (int e = 0; e < E; e++)
            {             
                if (edges[e].end != edges[(e + 1)%E].origin)
                {
                    cerr << "Discontinuity in the edges." << endl;
                    return false;
                }

                if (vertices[e] != edges[e].origin)
                {
                    cerr << "Vertices and edges don't match." << endl;
                    return false;
                }
                
            }
            return true;
        }
    };

    // polyhedra (Cells3D)

    struct Polyhedron
    {
        int id;
        vector<int> vertices;
        vector<int> edges;
        vector<Face> faces;
        
        int numVertices() const
        {
            return vertices.size();
        }

        int numEdges() const
        {
            return edges.size();
        }

        int numFaces() const
        {
            return faces.size();
        }

        bool isValid() const
        {
            int E = numFaces();

            for (int e = 0; e < E; e++)
            {
                if (! faces[e].isValid())
                {
                    cerr << "Invalid polyhedron." << endl;
                    return false;
                }
              
            return true;
            }
            
        }
    };
    








}