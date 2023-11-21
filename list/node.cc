#include "node.hh"

Node::Node( int _key, float _weight ) {
    // Inicializa o nó com a chave passada como argumento
    // Argumentos:
    //     _key: chave do nó a ser inicializado

    weight = _weight;
    key = _key;
}

float Node::getWeight() {
    // Método para retornar o peso da aresta
    // Retorno:
    //     weight: peso da aresta

    return weight;
}

void Node::setNext( Node* _next ){
    // Método para setar o próximo nó
    // Argumentos:
    //     _next: próximo nó

    next = _next;
}

int Node::getKey() {
    // Método para retornar a chave do nó
    // Retorno:
    //     key: chave do nó

    return key;
}

Node* Node::getNext() {
    // Método para retornar o próximo nó
    // Retorno:
    //     next: próximo nó
    
    return next;
}
