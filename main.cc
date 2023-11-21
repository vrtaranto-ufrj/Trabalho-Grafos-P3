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


    cout << "Carregando grafo : " << 1 << endl;
    grafo->loadListWeight( arquivo + to_string( 0 ) + extensao, true );

    //cout << grafo->ford_fulkerson( 0, grafo->getNumVertices() - 1 ) << endl;
    cout << grafo->ford_fulkerson( 2, 3 ) << endl;

    delete grafo;

    return 0;
}
