
# Biblioteca para Manipulação de Grafos

Esta biblioteca fornece uma série de ferramentas para manipulação e análise de grafos. Os grafos podem ser representados tanto através de matrizes de adjacência quanto listas de adjacência.

## Como Utilizar

1. Inclua `./graph/graph.hh` em seu projeto.
2. Crie um objeto do tipo `Graph`.
3. Carregue o grafo desejado usando um dos métodos de carregamento.
4. Utilize os métodos disponíveis para analisar e manipular o grafo.

## Para Utilizar o programa de Testes

1. Execute o comando `make` para compilar o programa
2. Execute o comando `make run` ou `./grafo` para executar o programa

## Métodos Disponíveis

### Métodos de Carregamento
- `void loadMatrix(string file)`: Carrega um grafo a partir de um arquivo utilizando matriz de adjacência.
- `void loadList(string file)`: Carrega um grafo a partir de um arquivo utilizando lista de adjacência.

### Análise e Buscas
- `void bfs(int root)`: Realiza uma busca em largura (BFS) a partir de um vértice raiz.
- `void dfs(int root)`: Realiza uma busca em profundidade (DFS) a partir de um vértice raiz.
- `int distance(int root, int destiny)`: Retorna a distância entre dois vértices.
  
### Cálculo do Diâmetro
- `int diameter(unsigned core = 1)`: Calcula o diâmetro do grafo. Por padrão, utiliza 1 thread.
- `int diameter_aprox(int precision = 50)`: Calcula uma aproximação do diâmetro do grafo utilizando BFS. Por padrão, realiza 50 buscas em largura.

### Outros Métodos
- `void write_informations()`: Escreve informações do grafo.

## Grafos Não Direcionados

Todos os métodos acima são compatíveis com grafos não direcionados.

## Grafos Com Peso

- `void loadMatrixWeighted(string file)`: Carrega um grafo a partir de um arquivo utilizando matriz de adjacência com pesos.
- `void loadListWeighted(string file)`: Carrega um grafo a partir de um arquivo utilizando lista de adjacência com pesos.
- `void dijkstra(int root, int destiny)`: Realiza o algoritmo de Dijkstra a partir de um vértice raiz.
- `void dijkstra_heap(int root, int destiny)`: Realiza o algoritmo de Dijkstra a partir de um vértice raiz.
---

**Nota**: Certifique-se de que os arquivos de entrada estejam no formato correto e que as dependências estejam corretamente configuradas em seu ambiente de desenvolvimento.
