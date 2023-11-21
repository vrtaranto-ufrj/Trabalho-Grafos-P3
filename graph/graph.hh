#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <queue>
#include <stack>
#include <algorithm>
#include <omp.h>
#include <iomanip>
#include <ctime>
#include <utility>
#include <sstream>



#include "../list/list.hh"
#include "../heap/binary_heap.hh"

#define FILE_COULD_NOT_OPEN     1
#define NEGATIVE_WEIGHT         2
#define VERTEX_LESS_ZERO        3

using namespace std;

struct residual_graph {
    residual_graph( int _num_vertices ) : adjacency_list( _num_vertices ){
        for ( int i = 0; i < _num_vertices; i++ ) {
            adjacency_list[i] = new List();
        }
    };
    vector<List*> adjacency_list;
};

class Graph {
    public:
        Graph();
        ~Graph();

        void loadMatrix( string file );
        void loadList( string file );
        void loadListWeight( string file, bool directed = false );
        
        float dijkstra( int root, int destiny = -1 );
        float dijkstra_heap( int root, int destiny = -1 );
        void printCaminho( int root, int destiny, vector<int>& parent );

        int ford_fulkerson( int source, int sink );

        int getMin( vector<bool>& visitados, vector<float> &distancias );

        vector<int>& getPath();

        void printList();

        void bfs( int root );
        void dfs( int root );
        int distance( int root, int destiny );
        int diameter( int cores = -1 );  // Padrão usar todos threads
        int diameter_aprox( int precision = 50, int cores = -1 ); // Padrão 50 Bfs e usando todos threads
        void write_informations();
        int getNumVertices();

        bool printPath;

    private:
        bool loaded;
        char type_representation;  // 'M' para Matriz, 'L' para Lista
        int num_vertices;
        int max;  // Para otimizar o calculo do diametro
        int current_component;
        vector<int> vertices;
        vector<List*> adjacency_list;
        vector<vector<char>> adjacency_matrix;  // char para usar apenas 1 byte ao inves de 4 bytes (int)
        vector< vector< pair< float, int > > > adjacent_vector;
        vector< int > path;

        void clearGraphRepresentation();

        vector<vector<int>> bfs_matrix( int root, int destiny = -1 );
        vector<vector<int>> bfs_list( int root, int destiny = -1 );
        vector<vector<int>> dfs_matrix( int root, int destiny = -1 );
        vector<vector<int>> dfs_list( int root, int destiny = -1 );

        vector<int>* connected_components();

        int bfs_matrix_parallel( int root );
        int bfs_list_parallel( int root );

        void write_tree( vector<vector<int>>&tree_information, int root, string _fs );

        int augment( int source, int sink, vector<int>& parent, residual_graph& residual );
        int bottleneck( int source, int sink, vector<int>& parent, residual_graph& residual );
        vector<int> getPath( int source, int sink, residual_graph& residual );
};



#endif
