#ifndef NODE_HH
#define NODE_HH

class Node {
    public:
        Node( int _key, int _capacity = 0, int _flux = 0 );

        // Setters
        void setNext( Node* _next );

        // Getters
        int getKey();
        int getCapacity();
        void setCapacity( int _capacity );
        int getFlux();
        Node* getNext();

    private:
        int key;
        int capacity;
        int flux;
        Node* next;
};

#endif
