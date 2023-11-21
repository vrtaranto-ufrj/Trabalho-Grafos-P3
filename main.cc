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


    cout << "Carregando grafo : " << argv[1] << endl;
    grafo->loadListWeight( arquivo + argv[1] + extensao, true );

    //cout << grafo->ford_fulkerson( 0, grafo->getNumVertices() - 1 ) << endl;
    cout << grafo->ford_fulkerson( 0, 1 ) << endl;

    delete grafo;

    return 0;
}
