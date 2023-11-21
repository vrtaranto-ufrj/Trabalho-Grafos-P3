#ifndef LIST_HH
#define LIST_HH
#include "node.hh"
#include <iostream>

using namespace std;

class List {
    public:
        List();
        ~List();

        void insertNode( int key, float weight = 0 );

        // Getters
        Node* getHead();
        void printList();

    private:
        Node* head;

};

#endif