#include "graph.hh"


Graph::Graph() : printPath(true) {
    // Construtor padrão
    loaded = false;
    current_component = 1;
    max = -1;
}

Graph::~Graph() {
    // Destrutor padrão
    clearGraphRepresentation();
}

void Graph::printList() {
    for ( unsigned long i = 0; i < adjacency_list.size(); i++ ) {
        cout << "V: " << i << " -> ";
        adjacency_list[i]->printList();
    }
}

void Graph::loadList( string file ) {
    // Método para carregar um grafo a partir de um arquivo de texto no formato lista de adjacência
    // Argumentos:
    //     file: string com o nome do arquivo a ser carregado
    // Retorno:
    //     void


    if (loaded)
        clearGraphRepresentation();

    type_representation = 'L';

    ifstream inputFile(file);

    if (!inputFile.is_open())
        exit(FILE_COULD_NOT_OPEN);

    string line, value;
    int edge[2];

    getline(inputFile, line);
    int _size = stoi(line);

    adjacency_list.resize(_size);

    for (int i = 0; i < _size; i++)
        adjacency_list[i] = new List;

    // Agora, vamos ler até o fim do arquivo.
    while (getline(inputFile, line)) {
        value = "";

        for (unsigned long c = 0; c < line.size(); c++) {
            if (line[c] != ' ')
                value += line[c];
            else {
                edge[0] = stoi(value) - 1;
                value = "";
            }
        }

        edge[1] = stoi(value) - 1;
        adjacency_list[edge[0]]->insertNode(edge[1]);
        adjacency_list[edge[1]]->insertNode(edge[0]);
    }

    inputFile.close();
    loaded = true;
    num_vertices = _size;
}

void Graph::loadListWeight( string file, bool directed ) {
    // Método para carregar um grafo a partir de um arquivo de texto no formato lista de adjacência
    // Argumentos:
    //     file: string com o nome do arquivo a ser carregado
    // Retorno:
    //     void


    if (loaded)
        clearGraphRepresentation();

    type_representation = 'L';

    ifstream inputFile(file);

    if (!inputFile.is_open()) {
        exit(FILE_COULD_NOT_OPEN);
    }

    string line, value;

    getline(inputFile, line);
    int _size = stoi(line);

    adjacency_list.resize(_size);

    for (int i = 0; i < _size; i++)
        adjacency_list[i] = new List;

    int numPalavras;
    int edge[2];
    float peso;

    // Agora, vamos ler até o fim do arquivo.
    while (getline(inputFile, line)) {
        value = "";
        numPalavras = 0;

        for (unsigned long c = 0; c < line.size(); c++) {
            if ( c == line.size() - 1 ) {
                value += line[c];
                peso = stof(value);
                if (peso < 0) {
                    cout << "Grafo possui peso negativo: " << peso << endl;
                    exit(NEGATIVE_WEIGHT);
                }
                value = "";
                numPalavras++;
            }
            if (line[c] != ' ') {
                value += line[c];
            }
            else {
                if (numPalavras == 0) {
                    edge[0] = stoi(value) - 1;
                    value = "";
                    numPalavras++;
                }
                else if (numPalavras == 1) {
                    edge[1] = stoi(value) - 1;
                    value = "";
                    numPalavras++;
                }
            }
        }
        adjacency_list[edge[0]]->insertNode(edge[1], peso);
        if (!directed)
            adjacency_list[edge[1]]->insertNode(edge[0], peso);
    }

    inputFile.close();
    loaded = true;
    num_vertices = _size;
}

void Graph::loadDictionary( string file) {
    // Método para carregar um grafo a partir de um arquivo de texto no formato dicionario de adjacência
    // Argumentos:
    //     file: string com o nome do arquivo a ser carregado
    // Retorno:
    //     void


    if (loaded)
        clearGraphRepresentation();

    type_representation = 'D';

    ifstream inputFile(file);

    if (!inputFile.is_open()) {
        exit(FILE_COULD_NOT_OPEN);
    }

    string line, value;

    getline(inputFile, line);
    int _size = stoi(line);

    dictionary.resize(_size);

    for (int i = 0; i < _size; i++) {
        dictionary[i] = new unordered_map<int, Edge>;
    }

    int numPalavras;
    int edge[2];
    float peso;
    Edge aresta;

    // Agora, vamos ler até o fim do arquivo.
    while (getline(inputFile, line)) {
        value = "";
        numPalavras = 0;

        for (unsigned long c = 0; c < line.size(); c++) {
            if ( c == line.size() - 1 ) {
                value += line[c];
                peso = stof(value);
                if (peso < 0) {
                    cout << "Grafo possui peso negativo: " << peso << endl;
                    exit(NEGATIVE_WEIGHT);
                }
                value = "";
                numPalavras++;
            }
            if (line[c] != ' ') {
                value += line[c];
            }
            else {
                if (numPalavras == 0) {
                    edge[0] = stoi(value) - 1;
                    value = "";
                    numPalavras++;
                }
                else if (numPalavras == 1) {
                    edge[1] = stoi(value) - 1;
                    value = "";
                    numPalavras++;
                }
            }
        }
        aresta.vertex = edge[1];
        aresta.capacity = peso;
        aresta.flux = 0;

        dictionary[edge[0]]->insert( {edge[1], aresta} );
    }

    inputFile.close();
    loaded = true;
    num_vertices = _size;
}

int Graph::bottleneck( int source, int sink, vector<int>& parent, residual_graph& residual ) {
    int gargalo = 1e9;
    int current_vertex = sink;
    Edge_residual current_edge;
    while ( current_vertex != source ) {
        current_edge = residual.dictionary[parent[current_vertex]]->find( current_vertex )->second;
        if ( current_edge.capacity < gargalo )
            gargalo = current_edge.capacity;
        current_vertex = parent[current_vertex];
    }
    return gargalo;
}

vector<int> Graph::getPathWithDelta(int source, int sink, residual_graph& residual, int delta) {
    vector<int> parent(num_vertices, -1);
    vector<bool> visited(num_vertices, false);
    queue<int> known;
    int current_vertex, neighbor;
    Edge_residual current_edge;

    known.push(source);
    visited[source] = true;

    while (!known.empty()) {
        current_vertex = known.front();
        known.pop();

        if (current_vertex == sink)
            break;

        for (auto it = residual.dictionary[current_vertex]->begin(); it != residual.dictionary[current_vertex]->end(); ++it) {
            neighbor = it->first;
            current_edge = it->second;
            // Verifica se a capacidade é maior ou igual a delta e se o vizinho ainda não foi visitado
            if (!visited[neighbor] && current_edge.capacity >= delta) {
                known.push(neighbor);
                visited[neighbor] = true;
                parent[neighbor] = current_vertex;
            }
        }
    }
    return parent;
}


vector<pair<int, int>> Graph::augment(int source, int sink, vector<int>& parent, residual_graph& residual) {
    int gargalo = bottleneck(source, sink, parent, residual);

    int current_vertex = sink;
    vector<pair<int, int>> affected_edges; // Para armazenar as arestas afetadas

    while (current_vertex != source) {
        int parent_vertex = parent[current_vertex];
        auto it = residual.dictionary[parent_vertex]->find(current_vertex);
        if (it->second.original) {
            auto it2 = dictionary[parent_vertex]->find(current_vertex);
            it2->second.flux += gargalo;
        }
        else {
            auto it2 = dictionary[current_vertex]->find(parent_vertex);
            it2->second.flux -= gargalo;
        }

        affected_edges.push_back({parent_vertex, current_vertex}); // Armazena a aresta afetada
        current_vertex = parent_vertex;
    }

    return affected_edges; // Retorna as arestas afetadas
}


void Graph::printDictionary() {
    for ( unsigned long i = 0; i < dictionary.size(); i++ ) {
        cout << "Vertex " << i + 1 << ": ";
        for ( auto it = dictionary[i]->begin(); it != dictionary[i]->end(); ++it ) {
            cout << "(" << it->first + 1 << ", " << it->second.capacity << ", " << it->second.flux << ") ";
        }
        cout << endl;
    }
}

void Graph::printResidual( residual_graph& residual ) {
    for ( unsigned long i = 0; i < residual.dictionary.size(); i++ ) {
        cout << "Vertex " << i + 1 << ": ";
        for ( auto it = residual.dictionary[i]->begin(); it != residual.dictionary[i]->end(); ++it ) {
            cout << "(" << it->first + 1 << ", " << it->second.capacity << ") ";
        }
        cout << endl;
    }
}



int Graph::ford_fulkerson( int source, int sink, bool write ) {
    source--;
    sink--;
    residual_graph residual( num_vertices );
    int fluxo = 0;

    
    for ( int i = 0; i < num_vertices; i++ ) {
        for (auto it = dictionary[i]->begin(); it != dictionary[i]->end(); ++it) {
            residual.dictionary[i]->insert( {it->first, {it->second.vertex, it->second.capacity, true}} );
            residual.dictionary[it->first]->insert( {i, {i, 0, false}} );
        }
    }

    /*printDictionary();
    cout << endl;
    printResidual( residual );
    cout << "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-" << endl;*/
    int delta = 0;
    for ( auto it = dictionary[source]->begin(); it != dictionary[source]->end(); ++it ) {
        delta += it->second.capacity;
    }
    delta = pow(2, floor(log2( delta )));
    
    while ( delta >= 1 ) {
        while ( true ) {
            vector<int>parents = getPathWithDelta( source, sink, residual, delta );
            if ( parents[sink] == -1 )
                break;
            auto affected_edges = augment( source, sink, parents, residual );
            
            for (auto& edge : affected_edges) {
                int u = edge.first, v = edge.second;
                if (residual.dictionary[u]->find(v)->second.original) {
                    auto it2 = dictionary[u]->find(v);
                    residual.dictionary[u]->find(v)->second.capacity = it2->second.capacity - it2->second.flux;
                } else {
                    auto it2 = dictionary[v]->find(u);
                    residual.dictionary[u]->find(v)->second.capacity = it2->second.flux;
                }
            }
            /*printDictionary();
            cout << endl;
            printResidual( residual );
            cout << "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-" << endl;*/

        }
        delta /= 2;
    }
    for ( auto it = dictionary[source]->begin(); it != dictionary[source]->end(); ++it ) {
        fluxo += it->second.flux;
    }
    if ( write ) {
        ofstream outFile( "fluxo_maximo.txt" );
        if ( !outFile.is_open() )
            exit( FILE_COULD_NOT_OPEN );
        
        outFile << "V1 V2 Fluxo" << endl;
        for ( int i = 0; i < num_vertices; i++ ) {
            for ( auto it = dictionary[i]->begin(); it != dictionary[i]->end(); ++it ) {
                outFile << i + 1 << " " << it->first + 1 << " " << it->second.flux << endl;
            }
        }


        outFile.close();
    }
    return fluxo;
}


float Graph::dijkstra( int root, int destiny ) {
    root--;
    destiny--;
    if ( root < 0 || destiny < -2 )
        exit( VERTEX_LESS_ZERO );
    vector<float> distance( num_vertices, 1e100 );
    vector<int> parent( num_vertices, -1 );
    vector<bool> visited( num_vertices, false );
    int current_vertex, neighbor, contador = 0;
    float weight;
    distance[root] = 0;
    Node* current_edge;

    while ( contador != num_vertices ) {
        contador++;
        current_vertex = getMin( visited, distance );

        if ( current_vertex == destiny ) {
            printCaminho( root, destiny, parent );
            return distance[destiny];
        }
        
        visited[current_vertex] = true;

        current_edge = adjacency_list[current_vertex]->getHead();
        while ( current_edge != nullptr ) {
            neighbor = current_edge->getKey();
            weight = current_edge->getCapacity();
            if ( distance[neighbor] > distance[current_vertex] + weight ) {
                distance[neighbor] = distance[current_vertex] + weight;
                parent[neighbor] = current_vertex;
            }
            current_edge = current_edge->getNext();
        }
    }

    return -1;

}

int Graph::getMin( vector<bool>& visitados, vector<float> &distancias ) {
    int minPos = -1;
    float minimo = 1e100;
    //#pragma omp parallel for shared(minPos, minimo)
    for ( int i = 0; i < num_vertices; i++ ) {
        if ( !visitados[i] && distancias[i] < minimo ) {
            //#pragma omp critical
            {
                if (distancias[i] < minimo) {
                    minPos = i;
                    minimo = distancias[i];
                }
            }
        }
    }
    return minPos;
}

float Graph::dijkstra_heap( int root, int destiny ) {
    root--;
    destiny--;
    if ( root < 0 || destiny < -2 )
        exit( VERTEX_LESS_ZERO );
    BinaryHeap* heap = new BinaryHeap( num_vertices );
    vector<float> distance( num_vertices, 1e20 );
    vector<int> parent( num_vertices, -1 );
    vector<bool> visited( num_vertices, false );
    Node* current_edge;
    int current_vertex, neighbor;
    float weight;
    distance[root] = 0;
    pair<float, int> current_pair;
    current_pair.first = 0;
    current_pair.second = root;
    heap->push( current_pair );

    while ( !heap->empty() ) {
        current_vertex = heap->top().second;

        if ( current_vertex == destiny ) {
            printCaminho( root, destiny, parent );
            return distance[destiny];
        }
        
        heap->pop();
        visited[current_vertex] = true;

        current_edge = adjacency_list[current_vertex]->getHead();
        while ( current_edge != nullptr ) {
            neighbor = current_edge->getKey();
            weight = current_edge->getCapacity();
            if ( distance[neighbor] > distance[current_vertex] + weight ) {
                distance[neighbor] = distance[current_vertex] + weight;
                current_pair.first = distance[neighbor];
                current_pair.second = neighbor;
                if ( parent[neighbor] == -1 )
                    heap->push( current_pair );
                else
                    heap->decreaseKey( neighbor, distance[neighbor] );
                parent[neighbor] = current_vertex;
                
            }
            current_edge = current_edge->getNext();
        }
    }
    return -1;
}

void Graph::printCaminho( int root, int destiny, vector<int>& parent ) {
    path.clear();

    cout << "Caminho: ";
    int current_vertex = destiny;
    vector<int> caminho;
    while ( current_vertex != root ) {
        caminho.push_back( current_vertex );
        current_vertex = parent[current_vertex];
    }
    caminho.push_back( current_vertex );
    if ( printPath ) {
    for ( int i = caminho.size() - 1; i >= 0; i-- ) {
        cout << caminho[i] + 1;
        if ( i != 0 )
            cout << " -> ";
    }
    
    cout << endl;
    }
    path = caminho;
}

vector<int>& Graph::getPath() {
    return path;
}

int Graph::getNumVertices() {
    return num_vertices;
}

void Graph::loadMatrix( string file ) {
    // Método para carregar um grafo a partir de um arquivo de texto no formato matriz de adjacência
    // Argumentos:
    //     file: string com o nome do arquivo a ser carregado
    // Retorno:
    //     void


    if (loaded)
        clearGraphRepresentation();

    type_representation = 'M';

    ifstream inputFile(file);

    if (!inputFile.is_open())
        exit(FILE_COULD_NOT_OPEN);

    string line, value;
    int edge[2];

    getline(inputFile, line);
    int _size = stoi(line);

    adjacency_matrix = vector<vector<char>>(_size, vector<char>(_size, 0));  // Inicializando tudo com 0

    // Agora, vamos ler até o fim do arquivo.
    while (getline(inputFile, line)) {
        value = "";

        for (unsigned long c = 0; c < line.size(); c++) {
            if (line[c] != ' ')
                value += line[c];
            else {
                edge[0] = stoi(value) - 1;
                value = "";
            }
        }

        edge[1] = stoi(value) - 1;
        adjacency_matrix[edge[0]][edge[1]] = 1;
        adjacency_matrix[edge[1]][edge[0]] = 1;
    }

    inputFile.close();
    loaded = true;
    num_vertices = _size;
}

void Graph::clearGraphRepresentation() {
    // Método para limpar a representação interna do grafo
    // Argumentos:
    //     void
    // Retorno:
    //     void


    // Limpar a matriz de adjacência
    adjacency_matrix.clear();

    // Limpar e liberar a memória das listas de adjacência existentes
    for (auto ptr : adjacency_list) {
        delete ptr;
    }
    adjacency_list.clear();

    // Limpar e liberar a memória dos dicionários de adjacência existentes
    for (auto ptr : dictionary) {
        delete ptr;
    }
    dictionary.clear();

    // Resetar outras variáveis associadas, se necessário
    loaded = false;
}

void Graph::bfs( int root ) {
    // Método para executar uma busca em largura no grafo
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    // Retorno:
    //     void


    vector<vector<int>> tree_information;
    
    switch ( type_representation )
    {
    case 'M':
        vertices = vector<int>( num_vertices, 0 );
        tree_information = bfs_matrix( root );
        write_tree( tree_information, root, "BFS");
        break;
    case 'L':
        vertices = vector<int>( num_vertices, 0 );
        tree_information = bfs_list( root );
        write_tree( tree_information, root, "BFS");
        break;
    default:
        break;
    }
}

vector<int>* Graph::connected_components() {
    // Método para encontrar os componentes conexos do grafo
    // Argumentos:
    //     void
    // Retorno:
    //     vector<int>*: ponteiro para um vetor de inteiros com os componentes conexos do grafo


    vector<int>* components;
    vertices = vector<int>( num_vertices, 0 );
    int current_vertex = 0;

    switch ( type_representation )
    {
    case 'M':
        while ( true ) {
            if ( current_vertex == num_vertices )
                break;
            if ( vertices[current_vertex] != 0 ) {
                current_vertex++;
                continue;
            }
            bfs_matrix( current_vertex );
            current_component++;
        }
        components = new vector<int>(vertices);
        current_component = 1;
        return components;
    case 'L':
        while ( true ) {
            if ( current_vertex == num_vertices )
                break;
            if ( vertices[current_vertex] != 0 ) {
                current_vertex++;
                continue;
            }
            bfs_list( current_vertex );
            current_component++;
        }
        components = new vector<int>(vertices);
        current_component = 1;
        return components;
    default:
        return nullptr;
    }
}

void Graph::dfs( int root ) {
    // Método para executar uma busca em profundidade no grafo
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    // Retorno:
    //     void

    vector<vector<int>> tree_information;
    switch ( type_representation )
    {
    case 'M':
        vertices = vector<int>( num_vertices, 0 );
        tree_information = dfs_matrix( root );
        write_tree( tree_information, root, "DFS" );
        break;
    case 'L':
        vertices = vector<int>( num_vertices, 0 );
        tree_information = dfs_list( root );
        write_tree( tree_information, root, "DFS" );
        break;
    default:
        break;
    }
}

int Graph::distance( int root, int destiny ) {
    // Método para calcular a distância entre dois vértices
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    //     destiny: inteiro com o índice do vértice destino da busca
    // Retorno:
    //     int: inteiro com a distância entre os vértices

    vector<vector<int>> tree_information;
    switch ( type_representation )
    {
    case 'M':
        vertices = vector<int>( num_vertices, 0 );
        tree_information = bfs_matrix( root, destiny );

        return tree_information[1][destiny];  // tree_information[1][destiny] = vetor dos niveis do destino
    case 'L':
        vertices = vector<int>( num_vertices, 0 );
        tree_information = bfs_list( root, destiny );

        return tree_information[1][destiny];  // tree_information[1][destiny] = vetor dos niveis do destino
    default:
        return -1;
    }
}

vector<vector<int>> Graph::bfs_matrix( int root, int destiny ) {
    // Método para executar uma busca em largura no grafo representado por matriz de adjacência
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    //     destiny: inteiro com o índice do vértice destino da busca
    // Retorno:
    //     vector<vector<int>>: vetor de vetores de inteiros com as informações da árvore gerada pela busca
    //         tree_information[0]: vetor de inteiros com os pais de cada vértice
    //         tree_information[1]: vetor de inteiros com os níveis de cada vértice


    queue<int> known;
    int current_vertex;
    vector<int> parent( num_vertices, -1 );
    vector<int> level( num_vertices, -1 );
    vector<vector<int>> tree_information;

    vertices[root] = 1;
    level[root] = 0;
    known.push( root );

    while ( !known.empty() ) {
        current_vertex = known.front();
        known.pop();

        if ( current_vertex == destiny )
            break;

        for ( unsigned long i = 0; i < adjacency_matrix[current_vertex].size(); i++ )
            if ( adjacency_matrix[current_vertex][i] == 1 && vertices[i] == 0 ) {
                vertices[i] = current_component;  // Marca o vertice no vetor
                known.push(i);  // Adiciona o vertice à lista
                parent[i] = current_vertex;  // Guarda o pai
                level[i] = level[current_vertex] + 1;  // Guarda o nivel
                if ( level[i] > max )
                    max = level[i];
            }
    }

    tree_information = { parent, level };
    return tree_information;
}

vector<vector<int>> Graph::bfs_list( int root, int destiny ) {
    // Método para executar uma busca em largura no grafo representado por lista de adjacência
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    //     destiny: inteiro com o índice do vértice destino da busca
    // Retorno:
    //     vector<vector<int>>: vetor de vetores de inteiros com as informações da árvore gerada pela busca
    //         tree_information[0]: vetor de inteiros com os pais de cada vértice
    //         tree_information[1]: vetor de inteiros com os níveis de cada vértice
    

    queue<int> known;
    int current_vertex, neighbor;
    vector<int> parent( num_vertices, -1 );
    vector<int> level( num_vertices, -1 );
    Node* current_edge;
    vector<vector<int>> tree_information;
    
    vertices[root] = 1;
    level[root] = 0;
    known.push( root );
    
    while ( !known.empty() ) {
        current_vertex = known.front();
        known.pop();

        if ( current_vertex == destiny )
            break;

        current_edge = adjacency_list[current_vertex]->getHead();  // Pega o endereço do vizinho atual

        while ( current_edge != nullptr ) {
            neighbor = current_edge->getKey(); // Pega o valor do vertice vizinho
            if ( vertices[neighbor] == 0 ) {
                vertices[neighbor] = current_component;  // Marca o vertice no vetor
                known.push(neighbor);  // Adiciona o vertice à lista
                parent[neighbor] = current_vertex;  // Guarda o pai
                level[neighbor] = level[current_vertex] + 1;  // Guarda o nivel
                if ( level[neighbor] > max )
                    max = level[neighbor];
            }

            current_edge = current_edge->getNext();  // Avança na lista
        }
    }

    tree_information = { parent, level };
    return tree_information;
}

vector<vector<int>> Graph::dfs_matrix( int root, int destiny ) {
    // Método para executar uma busca em profundidade no grafo representado por matriz de adjacência
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    //     destiny: inteiro com o índice do vértice destino da busca
    // Retorno:
    //     vector<vector<int>>: vetor de vetores de inteiros com as informações da árvore gerada pela busca
    //         tree_information[0]: vetor de inteiros com os pais de cada vértice
    //         tree_information[1]: vetor de inteiros com os níveis de cada vértice

    stack<int> known;
    int current_vertex;
    vector<int> parent( num_vertices, -1 );
    vector<int> level( num_vertices, -1 );
    vector<vector<int>> tree_information;

    vertices[root] = 1;
    level[root] = 0;
    known.push( root );


    while ( !known.empty() ) {
        current_vertex = known.top();
        known.pop();

        if ( current_vertex == destiny )
            break;
        
        for ( unsigned long i = 0; i < adjacency_matrix[current_vertex].size(); i++ ) {
            if ( adjacency_matrix[current_vertex][i] == 1 && vertices[i] == 0 ) {
                vertices[i] = current_component;  // Marca o vertice no vetor
                known.push(i);  // Adiciona o vertice à lista
                parent[i] = current_vertex;  // Guarda o pai
                level[i] = level[current_vertex] + 1;  // Guarda o nivel
                if ( level[i] > max )
                    max = level[i];
            }
        }
    }

    tree_information = { parent, level };
    return tree_information;
}

vector<vector<int>> Graph::dfs_list( int root, int destiny ) {
    // Método para executar uma busca em profundidade no grafo representado por lista de adjacência
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    //     destiny: inteiro com o índice do vértice destino da busca
    // Retorno:
    //     vector<vector<int>>: vetor de vetores de inteiros com as informações da árvore gerada pela busca
    //         tree_information[0]: vetor de inteiros com os pais de cada vértice
    //         tree_information[1]: vetor de inteiros com os níveis de cada vértice

    stack<int> known;
    int current_vertex, neighbor;
    vector<int> parent( num_vertices, -1 );
    vector<int> level( num_vertices, -1 );
    Node* current_edge;
    vector<vector<int>> tree_information;

    vertices[root] = 1;
    level[root] = 0;
    known.push( root );
    
    while ( !known.empty() ) {
        current_vertex = known.top();
        known.pop();

        if ( current_vertex == destiny )
            break;

        current_edge = adjacency_list[current_vertex]->getHead();  // Pega o endereço do vizinho atual

        while ( current_edge != nullptr ) {
            neighbor = current_edge->getKey(); // Pega o valor do vertice vizinho
            if ( vertices[neighbor] == 0 ) {
                vertices[neighbor] = current_component;  // Marca o vertice no vetor
                known.push(neighbor);  // Adiciona o vertice à lista
                parent[neighbor] = current_vertex;  // Guarda o pai
                level[neighbor] = level[current_vertex] + 1;  // Guarda o nivel
                if ( level[neighbor] > max )
                    max = level[neighbor];
            }

            current_edge = current_edge->getNext();  // Avança na lista
        }
    }

    tree_information = { parent, level };
    return tree_information;
}

void Graph::write_tree( vector<vector<int>>&tree_information, int root, string _fs ) {
    // Método para escrever as informações da árvore gerada pela busca em largura/ profundidade
    // Argumentos:
    //     tree_information: vetor de vetores de inteiros com as informações da árvore gerada pela busca
    //         tree_information[0]: vetor de inteiros com os pais de cada vértice
    //         tree_information[1]: vetor de inteiros com os níveis de cada vértice
    //     root: inteiro com o índice do vértice raiz da busca
    //     _fs: string com o nome do arquivo a ser escrito
    // Retorno:
    //     void


    string file = _fs + "vertex_" + to_string( root ) + ".txt";
    ofstream outFile( file );
    vector<int> parents = tree_information[0];
    vector<int> levels = tree_information[1];

    if ( !outFile.is_open() )
        exit( FILE_COULD_NOT_OPEN );

    outFile << "Vertex " << "Parent " << "Level " << endl;

    for ( int i = 0; i < num_vertices; i++ ) {
        if ( levels[i] != -1 )
            outFile << i << " " << parents[i] << " " << levels[i] << endl;
    }

    outFile.close();
}

void Graph::write_informations() {
    // Método para escrever as informações do grafo
    // Argumentos:
    //     void
    // Retorno:
    //     void


    ofstream outFile( "graph_informations.txt" );
    vector<int> degree( num_vertices );
    vector<int> size_components;
    Node* current_edge;
    int c, num_edges = 0, min_degree, max_degree, num_components = 0;
    float average_degree = 0, median_degree;
    vector<int>* components = connected_components();


    if ( !outFile.is_open() )
        exit( FILE_COULD_NOT_OPEN );

    for ( int i = 0; i < num_vertices; i++ ) {
        if ( components->at(i) > num_components )
            num_components = components->at(i);
    }

    size_components = vector<int>( num_components, 0 );

    for ( int i = 0; i < num_vertices; i++ ) {
        size_components[components->at(i) - 1]++;
    }
    

    switch ( type_representation )
    {
    case 'M':
        for ( int vertex = 0; vertex < num_vertices; vertex++ ) {
            degree[vertex] = count( adjacency_matrix[vertex].begin(), adjacency_matrix[vertex].end(), 1 );
        }
        break;
    case 'L':
        for ( int vertex = 0; vertex < num_vertices; vertex++ ) {
            c = 0;
            current_edge = adjacency_list[vertex]->getHead();
            while ( current_edge != nullptr ) {
                current_edge = current_edge->getNext();
                c++;
            }
            degree[vertex] = c;
        }
        break;
    default:
        break;
    }

    for ( int vertex = 0; vertex < num_vertices; vertex++ ) {
        num_edges += degree[vertex];
        //cout << "vertice: " << vertex << " grau: " << degree[vertex] << " soma ate agora: " << num_edges << endl;
    }
    num_edges /= 2;

    for ( int vertex = 0; vertex < num_vertices; vertex++ ) {
        if ( vertex == 0 || degree[vertex] < min_degree )
            min_degree = degree[vertex];
    }

    for ( int vertex = 0; vertex < num_vertices; vertex++ ) {
        if ( vertex == 0 || degree[vertex] > max_degree )
            max_degree = degree[vertex];
    }

    for ( int vertex = 0; vertex < num_vertices; vertex++ ) {
        average_degree += degree[vertex];
    }
    average_degree /= num_vertices;

    sort( degree.begin(), degree.end() );
    if ( num_vertices % 2 == 1 )
        median_degree = static_cast<float>(degree[num_vertices / 2]);
    else 
        median_degree = (static_cast<float>(degree[(num_vertices - 1) / 2]) + static_cast<float>(degree[num_vertices / 2])) / 2.0;



    outFile << "Number of vertices: " << num_vertices << endl;

    outFile << "Number of edges: " << num_edges << endl;

    outFile << "Min degree: " << min_degree << endl;
    outFile << "Max degree: " << max_degree << endl;
    outFile << "Average degree: " << average_degree << endl;
    outFile << "Median degree: " << median_degree << endl;
    outFile << "Number of components: " << num_components << endl << endl;

    sort( size_components.begin(), size_components.end() );
    for ( int i = num_components -1; i >= 0; i-- ) {
        ofstream *comp = new ofstream( "component" + to_string(num_components - i) + "_informations.txt" );
        *comp << "Component " << num_components - i << " size: " << size_components[i] << endl;
        *comp << "Vertices: ";
        for ( int j = 0; j < num_vertices; j++ ) {
            if ( components->at(j) == num_components - i )
                *comp << j << " ";
        }
        *comp << endl;
        comp->close();
        delete comp;
        outFile << "Component " << right << setw(2) << num_components - i << " size: " << size_components[i] << endl;
    }


    
    delete components;
    outFile.close();
}

int Graph::bfs_matrix_parallel( int root ) {
    // Método para executar uma busca em largura no grafo representado por matriz de adjacência de forma paralela
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    // Retorno:
    //     int: inteiro com o nível máximo da árvore gerada pela busca
    

    queue<int> known;
    int current_vertex;
    vector<char> local_vertices( num_vertices, 0 );
    vector<int> level( num_vertices, -1 );
    int local_max = -1;

    local_vertices[root] = 1;
    level[root] = 0;
    known.push(root);

    while ( !known.empty() ) {
        current_vertex = known.front();
        known.pop();

        for ( unsigned long i = 0; i < adjacency_matrix[current_vertex].size(); i++ )
            if (adjacency_matrix[current_vertex][i] == 1 && local_vertices[i] == 0) {
                local_vertices[i] = 1;
                known.push(i);
                level[i] = level[current_vertex] + 1;
                if ( level[i] > local_max )
                    local_max = level[i];
            }
    }

    return local_max;
}

int Graph::bfs_list_parallel( int root ) {
    // Método para executar uma busca em largura no grafo representado por lista de adjacência de forma paralela
    // Argumentos:
    //     root: inteiro com o índice do vértice raiz da busca
    // Retorno:
    //     int: inteiro com o nível máximo da árvore gerada pela busca


    queue<int> known;
    int current_vertex, neighbor;
    vector<char> local_vertices( num_vertices, 0 );
    vector<int> level( num_vertices, -1 );
    Node* current_edge;
    int local_max = -1;

    local_vertices[root] = 1;
    level[root] = 0;
    known.push(root);

    while ( !known.empty() ) {
        current_vertex = known.front();
        known.pop();

        current_edge = adjacency_list[current_vertex]->getHead();

        while ( current_edge != nullptr ) {
            neighbor = current_edge->getKey();
            if ( local_vertices[neighbor] == 0 ) {
                local_vertices[neighbor] = 1;
                known.push(neighbor);
                level[neighbor] = level[current_vertex] + 1;
                if ( level[neighbor] > local_max )
                    local_max = level[neighbor];
            }

            current_edge = current_edge->getNext();
        }
    }

    return local_max;
}

int Graph::diameter( int cores ) {
    // Método para calcular o diâmetro do grafo
    // Argumentos:
    //     cores: inteiro com o número de threads a serem utilizadas
    // Retorno:
    //     int: inteiro com o diâmetro do grafo


    int max_diameter = -1;

    // Define o número de threads baseado em sua CPU
    if ( cores != -1) 
        omp_set_num_threads( cores );

    switch (type_representation) {
    case 'M':
        #pragma omp parallel for reduction(max: max_diameter)
        for (int vertex = 0; vertex < num_vertices; vertex++) {
            int local_diameter = bfs_matrix_parallel(vertex);
            //cout << "Thread : " << omp_get_thread_num() << " fez vertice: " << vertex << endl;
            if (local_diameter > max_diameter) {
                max_diameter = local_diameter;
                //cout << "Diametro atualizado para " << max_diameter << endl;
            }
        }
        break;

    case 'L':
        #pragma omp parallel for reduction(max: max_diameter)
        for (int vertex = 0; vertex < num_vertices; vertex++) {
            int local_diameter = bfs_list_parallel(vertex);
            //cout << "Thread : " << omp_get_thread_num() << " fez vertice: " << vertex << endl;
            if (local_diameter > max_diameter) {
                max_diameter = local_diameter;
                //cout << "Diametro atualizado para " << max_diameter << endl;
            }
        }
        break;
    
    default:
        return -1;
    }

    return max_diameter;
}

int Graph::diameter_aprox( int precision, int cores ) {
    // Método para calcular o diâmetro do grafo de forma aproximada
    // Argumentos:
    //     precision: inteiro com o número de vezes que o algoritmo será executado
    //     cores: inteiro que definirá o número de cores a ser usado, padrão = todos threads
    // Retorno:
    //     int: inteiro com o diâmetro do grafo

    int num_components = 0;
    int max_diameter = -1;
    vector<int>* all_components = connected_components();
    vector<vector<int>> components;

    // Define o número de threads baseado em sua CPU
    if ( cores != -1) 
        omp_set_num_threads( cores );

    srand( time(nullptr) );

    for ( int i = 0; i < num_vertices; i++ ) {
        if ( all_components->at(i) > num_components )
            num_components = all_components->at(i);
    }

    components.resize( num_components );

    // Criando um vector em que seus elementos sejam cada conjunto conexo do grafo
    for ( int i = 0; i < num_vertices; i++ ) {
        components[(*all_components)[i] - 1].push_back( i );
    }

    switch ( type_representation ) {
    case 'M': 
        for ( auto c : components ) {
            #pragma omp parallel for reduction( max: max_diameter )
            for ( int i = 0; i < precision; i++ ) {
                int local_diameter = bfs_matrix_parallel( rand() % c.size() );
                if ( local_diameter > max_diameter ) {
                max_diameter = local_diameter;
            }
            }
        }
        break;

    case 'L':
        for ( auto c : components ) {
            #pragma omp parallel for reduction( max: max_diameter )
            for ( int i = 0; i < precision; i++ ) {
                int local_diameter = bfs_list_parallel( rand() % c.size() );
                if ( local_diameter > max_diameter ) {
                max_diameter = local_diameter;
            }
            }
        }
        break;
    
    default:
        return -1;
    }

    return max_diameter;
}
