#include "list.hh"

List::List() {
    // Inicializa a lista vazia

    head = nullptr;
}

List::~List() {
    // Deleta todos os nós da lista

    Node* no;

    while ( head != nullptr ) {
        no = head->getNext();
        delete head;
        head = no;
    }

}

void List::insertNode( int key, int capacity, int flux, bool original ) {
    // Método para inserir um nó na lista
    // Argumentos:
    //     key: chave do nó a ser inserido
    //     weight: peso da aresta

    Node* no = new Node( key, capacity, flux, original );
    no->setNext( head );
    head = no;
}

void List::printList() {
    // Método para imprimir a lista

    Node* no = head;

    while ( no != nullptr ) {
        cout << "V: " << no->getKey() << " , Capacidade:" << no->getCapacity() << " , Fluxo: " << no->getFlux() << " -> ";
        no = no->getNext();
    }

    cout << endl;
}

Node* List::getHead() {
    // Método para retornar o nó cabeça da lista
    // Retorno:
    //     head: nó cabeça da lista
    
    return head;
}
