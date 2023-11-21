#ifndef NODE_HH
#define NODE_HH

class Node {
    public:
        Node( int _key, float _weight = 0 );

        // Setters
        void setNext( Node* _next );

        // Getters
        int getKey();
        float getWeight();
        Node* getNext();

    private:
        int key;
        float weight;
        Node* next;
};

#endif
