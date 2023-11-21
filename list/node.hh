#ifndef NODE_HH
#define NODE_HH

class Node {
    public:
        Node( int _key, int _capacity = 0, int _flux = 0, bool _original = true );

        // Setters
        void setNext( Node* _next );

        // Getters
        int getKey();
        int getCapacity();
        void setCapacity( int _capacity );
        int getFlux();
        void setFlux( int _flux );
        bool isOriginal();
        Node* getNext();

    private:
        int key;
        int capacity;
        int flux;
        bool original;
        Node* next;
};

#endif
