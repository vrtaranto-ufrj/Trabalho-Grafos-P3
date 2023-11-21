#ifndef LIST_HH
#define LIST_HH
#include "node.hh"
#include <iostream>

using namespace std;

class List {
    public:
        List();
        ~List();

        void insertNode( int key, int weight = 0, int flux = 0 );

        // Getters
        Node* getHead();
        void printList();

    private:
        Node* head;

};

#endif