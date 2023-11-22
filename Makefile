all: main.o graph.o list.o node.o binary_heap.o
	g++ -fopenmp -O3 -o grafo main.o graph.o list.o node.o binary_heap.o

main.o: main.cc ./graph/graph.hh ./list/list.hh
	g++ -fopenmp -O3 -c main.cc

binary_heap.o: ./heap/binary_heap.cc ./heap/binary_heap.hh
	g++ -fopenmp -O3 -c ./heap/binary_heap.cc

graph.o: ./graph/graph.cc ./graph/graph.hh ./list/list.hh
	g++ -fopenmp -O3 -c ./graph/graph.cc

list.o: ./list/list.cc ./list/list.hh ./list/node.hh
	g++ -fopenmp -O3 -c ./list/list.cc

node.o: ./list/node.cc ./list/node.hh
	g++ -fopenmp -O3 -c ./list/node.cc

clean:
	rm -f grafo *.o

run: all
	./grafo
