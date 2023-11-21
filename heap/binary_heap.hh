#ifndef BINARY_HEAP_HH
#define BINARY_HEAP_HH

#include <vector>
#include <utility>
#include <stdexcept>

class BinaryHeap {
public:
    // Construtor: inicializa o heap com um tamanho específico.
    BinaryHeap(int tamanho);
    
    // Insere um elemento no heap.
    void push(std::pair<float, int> no);
    
    // Retorna o elemento no topo do heap sem removê-lo.
    std::pair<float, int> top() const;
    
    // Remove o elemento no topo do heap.
    void pop();
    
    // Retorna 'true' se o heap estiver vazio e 'false' caso contrário.
    bool empty() const;

    // Atualiza a chave de um elemento no heap.
    void decreaseKey(int elemento, float novaChave);

private:
    // Calcula o índice do pai de um elemento específico no heap.
    inline int calculaPai(int posicao) const;
    
    // Calcula os índices dos filhos de um elemento específico no heap.
    inline std::pair<int, int> calculaFilhos(int posicao) const;
    
    // Compara dois elementos no heap e retorna 'true' se o elemento na 'posicao1' for menor.
    bool isMenor(int posicao1, int posicao2) const;
    
    // Troca a posição de dois elementos no heap.
    void swap(int posicao1, int posicao2);

    // Tamanho máximo do heap.
    int tamanho;

    // Índice do último elemento no heap.
    int ultimo;
    
    // Vetor para armazenar os elementos do heap.
    std::vector<std::pair<float, int>> array;

    // Mapa para armazenar a posição de cada elemento no heap.
    std::vector<int> mapPosicao;
};

#endif
