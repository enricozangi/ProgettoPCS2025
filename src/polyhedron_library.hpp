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

        void swapVertices() // O(1)
        {
            swap(origin, end);
        }
    };
    
    struct Face
    {
        int id;
        vector<Vertex> vertices;
        vector<Edge> edges;
        
        int numVertices() const { return vertices.size(); } // O(1)
        int numEdges() const { return edges.size(); } // O(1)

        vector<Edge> sortEdges() const // O(E^2), dove E = edges.size()
        {
            if (edges.empty()) return {};

            vector<Edge> unsorted = edges;            
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
                        cerr << "errore nell'ordinamento dei lati" << endl;
                        break;
                    }
                }  

            if (sorted.front().origin != sorted.back().end)
            {
                cerr << "la faccia non è chiusa" << endl;
            }
            return sorted;
        }

        vector<Vertex> sortVertices() const // O(E^2), dove E = edges.size()
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

        bool isValid() const // O(E^2), dove E = edges.size()
        {
            int E = numEdges();
            if (E < 3)
            {
                cerr << "La faccia con id " << id << " ha meno di 3 spigoli." << endl;
                return false;
            }
            vector<Edge> edges_list = sortEdges();
            vector<Vertex> vertices_list = sortVertices();

            for (int e = 0; e < E; e++)
            {
                int v1 = edges_list[e].end;

                int next_v1 = edges_list[(e + 1) % E].origin;
                int next_v2 = edges_list[(e + 1) % E].end;

                if (v1 != next_v1 && v1 != next_v2)
                {
                    cerr << "Discontinuità dei lati." << endl;
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
        
        int numVertices() const { return vertices.size(); } // O(1)
        int numEdges() const { return edges.size(); } // O(1)
        int numFaces() const { return faces.size(); } // O(1)

        bool isValid() const // O(F * E^2)
        {
            int F = numFaces();

            for (int e = 0; e < F; e++)
            {
                if (!faces[e].isValid())
                {
                    cerr << "Poliedro non valido: la faccia " << faces[e].id << " non è valida." << endl;
                    return false;
                }
            }
            return true;
        }
    };

}