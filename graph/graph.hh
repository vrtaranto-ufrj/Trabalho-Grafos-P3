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
#include <unordered_map>
#include <cmath>



#include "../list/list.hh"
#include "../heap/binary_heap.hh"

#define FILE_COULD_NOT_OPEN     1
#define NEGATIVE_WEIGHT         2
#define VERTEX_LESS_ZERO        3

using namespace std;


struct Edge {
    int vertex;
    int capacity;
    int flux;
};

struct Edge_residual {
    int vertex;
    int capacity;
    bool original;
};

struct residual_graph {
    residual_graph( int _num_vertices ) : dictionary( _num_vertices ){
        for ( int i = 0; i < _num_vertices; i++ ) {
            dictionary[i] = new unordered_map<int, Edge_residual>;
        }
    };
    ~residual_graph() {
        for ( int i = 0; i < dictionary.size(); i++ ) {
            delete dictionary[i];
        }
    }
    vector<unordered_map<int, Edge_residual>*> dictionary;
};


class Graph {
    public:
        Graph();
        ~Graph();

        void loadMatrix( string file );
        void loadList( string file );
        void loadListWeight( string file, bool directed = false );
        void loadDictionary( string file );
        
        float dijkstra( int root, int destiny = -1 );
        float dijkstra_heap( int root, int destiny = -1 );
        void printCaminho( int root, int destiny, vector<int>& parent );

        int ford_fulkerson( int source, int sink, bool write = false );

        int getMin( vector<bool>& visitados, vector<float> &distancias );

        vector<int>& getPath();

        void printList();
        void printDictionary();
        void printResidual( residual_graph& residual );

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
        vector<unordered_map<int, Edge>*> dictionary;

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

        vector<pair<int, int>> augment(int source, int sink, vector<int>& parent, residual_graph& residual);
        int bottleneck( int source, int sink, vector<int>& parent, residual_graph& residual );
        vector<int> getPathWithDelta(int source, int sink, residual_graph& residual, int delta);
};



#endif
