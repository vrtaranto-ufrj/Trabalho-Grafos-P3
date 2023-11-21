#include <iostream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <random>
#include <unordered_map>

#include "./graph/graph.hh"

using namespace std;


int main( int argc, char *argv[] ) {

    

    float distancia;
    Graph* grafo = new Graph();

    string arquivo = "grafo_rf_", extensao = ".txt";

    delete grafo;
    for ( int i = 1; i < 6; i++ ) {
        grafo = new Graph();
        cout << "Carregando grafo : " << i << endl;
        grafo->loadDictionary( arquivo + to_string( i ) + extensao );
        cout << "Fluxo maximo entre vertices 1 e 2: " << grafo->ford_fulkerson( 1, 2 ) << endl;
       
        
        auto start = chrono::high_resolution_clock::now();
        for ( int j = 0; j < 10; j++ ) {
            grafo->ford_fulkerson( rand() % grafo->getNumVertices() + 1, rand() % grafo->getNumVertices() + 1 );
        }
        auto finish = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = finish - start;
        std::cout << std::fixed << std::setprecision(3);
        cout << "Tempo medio grafo " << i << " : " << elapsed.count() / 10 << "s" << endl << endl;

        delete grafo;
    }

    return 0;
}
