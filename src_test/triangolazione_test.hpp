#include <gtest/gtest.h>
#include "polyhedron_library.hpp"
#include "Utils.hpp"
#include "triangolazione.hpp"

using namespace polyhedron_library;

TEST(TriangolazioneTest, SuddivisioneTetraedro) {
    Polyhedron p = ottaedro();

    int b = 2;

    // Applica la suddivisione
    Polyhedron p_triangolato = triangolazione(p, b);

    // Il tetraedro ha 4 facce, quindi ci aspettiamo 4 * 4 = 16 facce
    EXPECT_EQ(p_triangolato.faces.size(), 32);

    // Verifica che il numero di vertici sia aumentato
    EXPECT_EQ(p_triangolato.vertices.size(), 18);
    
    EXPECT_EQ(p_triangolato.edges.size(), 48);

 //tutta sta manfrina l'abbiamo fatta a mente potrebbe essere sbagliato :p
    int x = 3;
    Polyhedron p_triangolato2 = triangolazione(p, x);

    EXPECT_EQ(p_triangolato2.faces.size(), 72);

    EXPECT_EQ(p_triangolato2.vertices.size(), 38);

    EXPECT_EQ(p_triangolato2.edges.size(), 108);
}

TEST(TriangolazioneTest, numTestq3)
{
    Polyhedron p = tetraedro();
    int c = 0;
    for (int b = 1; b <= 3; ++b) {
        int T = b * b + c * b + c * c;
        Polyhedron p_triangolato = triangolazione(p, b);

        EXPECT_EQ(2 * T + 2, p_triangolato.numVertices()) << "Failed for b = " << b;
        EXPECT_EQ(6 * T, p_triangolato.numEdges()) << "Failed for b = " << b;
        EXPECT_EQ(4 * T, p_triangolato.numFaces()) << "Failed for b = " << b;
    }
}
TEST(TriangolazioneTest, numTestq4)
{
    Polyhedron p = ottaedro();
    int c = 0;
    for (int b = 1; b <= 3; ++b) {
        int T = b * b + c * b + c * c;
        Polyhedron p_triangolato = triangolazione(p, b);

        EXPECT_EQ(4 * T + 2, p_triangolato.numVertices()) << "Failed for b = " << b;
        EXPECT_EQ(12 * T, p_triangolato.numEdges()) << "Failed for b = " << b;
        EXPECT_EQ(8 * T, p_triangolato.numFaces()) << "Failed for b = " << b;
    }
}
TEST(TriangolazioneTest, numTestq5)
{
    Polyhedron p = icosaedro();
    int c = 0;
    for (int b = 1; b <= 3; ++b) {
        int T = b * b + c * b + c * c;
        Polyhedron p_triangolato = triangolazione(p, b);

        EXPECT_EQ(10 * T + 2, p_triangolato.numVertices()) << "Failed for b = " << b;
        EXPECT_EQ(30 * T, p_triangolato.numEdges()) << "Failed for b = " << b;
        EXPECT_EQ(20 * T, p_triangolato.numFaces()) << "Failed for b = " << b;
    }
}
TEST(TriangolazioneTest, triangolazione2Test) {
    Polyhedron p = tetraedro();

    int b = 1;

    // Applica la suddivisione
    Polyhedron p_triangolato = triangolazione2(p, b);

    // Il tetraedro ha 4 facce, quindi ci aspettiamo 4 * 4 = 16 facce
    EXPECT_EQ(p_triangolato.faces.size(), 72);

    // Verifica che il numero di vertici sia aumentato
    EXPECT_EQ(p_triangolato.vertices.size(), 14);
    
    EXPECT_EQ(p_triangolato.edges.size(), 108);
// Aggiungere test su valenza dei vertici
}