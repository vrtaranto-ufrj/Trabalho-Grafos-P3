#include "node.hh"

Node::Node( int _key, int _capacity, int _flux ) {
    // Inicializa o nó com a chave passada como argumento
    // Argumentos:
    //     _key: chave do nó a ser inicializado

    capacity = _capacity;
    key = _key;
    flux = _flux;
}

int Node::getCapacity() {
    // Método para retornar o capacidade da aresta
    // Retorno:
    //     capacity: capacidade da aresta

    return capacity;
}

void Node::setCapacity( int _capacity ) {
    // Método para setar a capacidade da aresta
    // Argumentos:
    //     _capacity: capacidade da aresta

    capacity = _capacity;
}

int Node::getFlux() {
    // Método para retornar o fluxo da aresta
    // Retorno:
    //     flux: fluxo da aresta

    return flux;
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
