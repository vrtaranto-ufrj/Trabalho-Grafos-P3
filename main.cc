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

    string arquivo = "grafo_W_", extensao = ".txt";


    for ( int i = 1; i <= 5; i++ ) {
        cout << "Carregando grafo : " << i << endl;
        grafo->loadListWeight( arquivo + to_string( i ) + extensao );

        for ( int j = 20; j <= 60; j += 10 ) {
            cout << "10 -> " << j << endl;
            distancia = grafo->dijkstra_heap( 10, j );
            cout << "Distancia: " << distancia << endl << endl;
        }
        


        int k = 20;
        auto start = chrono::high_resolution_clock::now();
        for ( int j = 0; j < k; j++ ) {
            grafo->dijkstra( rand() % grafo->getNumVertices() + 1 );
        }
        auto finish = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = finish - start;
        std::cout << std::fixed << std::setprecision(3);
        cout << "Tempo medio usando vetor: " << elapsed.count() / k << "s" << endl << endl;


        k = 200;
        start = std::chrono::high_resolution_clock::now();
        for ( int j = 0; j < k; j++ ) {
            grafo->dijkstra_heap( rand() % grafo->getNumVertices() + 1 );
        }
        finish = chrono::high_resolution_clock::now();
        elapsed = finish - start;
        cout << "Tempo medio usando heap: " << elapsed.count() / k << "s" << endl << endl;

        cout << "--------------------------------------------------------------------------------------------------" << endl << endl;
    }

    delete grafo;

    grafo = new Graph();

    unordered_map<string, int> mapNomes;
    unordered_map<int, string> mapIds;
    string nome;
    ifstream file;
    file.open( "rede_colaboracao_vertices.txt" );

    int id = 0;
    while ( !file.eof() ) {
        getline( file, nome, ',' );
        getline( file, nome );
        mapNomes[nome] = id;
        mapIds[id] = nome;
        id++;
    }

    mapNomes.erase( to_string(id - 1) );
    
    file.close();

    grafo->loadListWeight( "rede_colaboracao.txt" );

    string raizNome = "Edsger W. Dijkstra";
    vector<string> destinoNomes = vector<string>( { "Alan M. Turing", "J. B. Kruskal", "Jon M. Kleinberg", "Éva Tardos", "Daniel R. Figueiredo" } );
    
    grafo->printPath = false;
    for ( auto& nome : destinoNomes ) {
        cout << "Distancia de " << raizNome << " para " << nome << ": " << grafo->dijkstra_heap( mapNomes[raizNome], mapNomes[nome] ) << endl;
        cout << "Caminho: ";
        auto caminho = grafo->getPath();
        for (int i = caminho.size() - 1; i >= 0; i--) {
            cout << mapIds[caminho[i]+1];
            if ( i != 0 )
                cout << " -> ";
        }
        cout << endl << endl;
    }

    delete grafo;
    
    

    return 0;
}
