#include "node.hh"

Node::Node( int _key, int _capacity, int _flux, bool _original ) {
    // Inicializa o nó com a chave passada como argumento
    // Argumentos:
    //     _key: chave do nó a ser inicializado

    capacity = _capacity;
    key = _key;
    flux = _flux;
    original = _original;
}

int Node::getCapacity() {
    // Método para retornar o capacidade da aresta
    // Retorno:
    //     capacity: capacidade da aresta

    return capacity;
}

bool Node::isOriginal() {
    // Método para retornar se o nó é original
    // Retorno:
    //     original: se o nó é original

    return original;
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

void Node::setFlux( int _flux ) {
    // Método para setar o fluxo da aresta
    // Argumentos:
    //     _flux: fluxo da aresta

    flux = _flux;
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
