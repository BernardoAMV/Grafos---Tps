#include <iostream>
#include <vector>
#include <set>
#include <stack>
#include <unordered_set>

using namespace std;

class Edge {
public:
    int u, v;
    Edge(int u, int v) : u(min(u, v)), v(max(u, v)) {}  // Ordena u e v

    bool operator==(const Edge& other) const {
        return u == other.u && v == other.v;
    }

    bool operator<(const Edge& other) const {
        return tie(u, v) < tie(other.u, other.v);  // Comparação ordenada
    }
};

class Graph {
public:
    int vertices;
    vector<set<int>> adjList;

    Graph(int vertices) : vertices(vertices) {
        adjList.resize(vertices);  // Certifique-se de que o tamanho seja o correto
    }

    void addEdge(int u, int v) {
        if (u >= vertices || v >= vertices) return; // Proteção contra índices inválidos
        adjList[u].insert(v);
        adjList[v].insert(u);  // Para um grafo não direcionado
    }

    const set<int>& getVizinhos(int v) const {
        return adjList[v];
    }

    int getN() const {
        return vertices;
    }
};

class Conjuntos {
public:
    vector<set<Edge>> componentes;  // Usar set para evitar duplicação de arestas

    Conjuntos(int n) {
        componentes.resize(n);
    }

    void add(int component, const Edge& edge) {
        if (component < componentes.size()) {
            componentes[component].insert(edge);  // Arestas duplicadas serão ignoradas automaticamente
        }
    }
};

class M1 {
private:
    Graph& lista;
    vector<bool> visited;
    unordered_set<int> addedVertices;
    set<Edge> edges;

    bool targetedDFS(int initial, int target, int avoiding) {
        stack<int> stack;
        stack.push(initial);

        while (!stack.empty()) {
            int current = stack.top();
            stack.pop();

            if (current == target) {
                return true;
            }

            if (!visited[current]) {
                visited[current] = true;

                for (int neighbor : lista.getVizinhos(current)) {
                    if (neighbor == avoiding) continue;
                    if (current == initial && neighbor == target) continue;
                    if (!visited[neighbor]) {
                        stack.push(neighbor);
                    }
                }
            }
        }
        return false;
    }

    bool areAdjacentsInSameCycle(int v, int w) {
        visited.assign(lista.getN(), false);  // Garante o tamanho correto de 'visited'
        return targetedDFS(v, w, -1);
    }

    bool areGrandparentAndGrandchildInSameCycle(int v, int w, int son) {
        visited.assign(lista.getN(), false);  // Garante o tamanho correto de 'visited'
        return targetedDFS(v, w, son);
    }

    void adicionarVertice(int start, unordered_set<int>& currentCycleVertices) {
        stack<int> stack;
        stack.push(start);

        while (!stack.empty()) {
            int v = stack.top();
            stack.pop();

            if (!currentCycleVertices.count(v)) {
                currentCycleVertices.insert(v);
                addedVertices.insert(v);

                for (int neighbor : lista.getVizinhos(v)) {
                    if (currentCycleVertices.count(neighbor)) continue;
                    if (currentCycleVertices.size() < 2) {
                        if (areAdjacentsInSameCycle(v, neighbor)) {
                            stack.push(neighbor);
                        }
                    } else {
                        if (areGrandparentAndGrandchildInSameCycle(start, neighbor, v)) {
                            stack.push(neighbor);
                        }
                    }
                }
            }
        }
    }

public:
    M1(Graph& lista) : lista(lista) {}

    Conjuntos executar() {
        Conjuntos conjuntos(lista.getN());
        unordered_set<int> currentCycleVertices;

        // Preencher o conjunto de arestas
        for (int i = 0; i < lista.getN(); i++) {
            for (int neighbor : lista.getVizinhos(i)) {
                if (i != neighbor) {  // Evitar adicionar arestas de laço
                    edges.insert(Edge(i, neighbor));
                }
            }
        }

        int currentComponent = -1;

        for (int i = 0; i < lista.getN(); i++) {
            if (!addedVertices.count(i)) {
                currentCycleVertices.clear();
                visited.assign(lista.getN(), false);  // Reinicializar visited com o tamanho correto

                adicionarVertice(i, currentCycleVertices);

                currentComponent++;
                for (int vertex : currentCycleVertices) {
                    for (int neighbor : lista.getVizinhos(vertex)) {
                        if (currentCycleVertices.count(neighbor)) {
                            conjuntos.add(currentComponent, Edge(vertex, neighbor));
                            edges.erase(Edge(vertex, neighbor));
                        }
                    }
                }
            }
        }

        // Adiciona as arestas restantes que não formam ciclos
        for (const Edge& edge : edges) {
            currentComponent++;
            conjuntos.add(currentComponent, edge);
        }

        return conjuntos;
    }
};

// Função para imprimir os componentes sem duplicatas
void imprimirComponentes(const Conjuntos& componentes) {
    for (int i = 0; i < componentes.componentes.size(); i++) {
        if (!componentes.componentes[i].empty()) {
            cout << "Componente " << i << ": ";
            for (const Edge& edge : componentes.componentes[i]) {
                cout << "(" << edge.u << ", " << edge.v << ") ";
            }
            cout << endl;
        }
    }
}

Graph build_example_graph() {
    Graph g(12);

    g.addEdge(1, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 3);
    g.addEdge(2, 4);
    g.addEdge(3, 5);
    g.addEdge(3, 6);
    g.addEdge(4, 7);
    g.addEdge(5, 6);
    g.addEdge(4, 8);
    g.addEdge(7, 8);
    g.addEdge(7, 10);
    g.addEdge(8, 11);
    g.addEdge(10, 11);
    g.addEdge(5, 9);
    g.addEdge(9, 6);

    return g;
}

int main() {
    Graph g = build_example_graph();

    M1 m(g);
    Conjuntos componentes = m.executar();

    imprimirComponentes(componentes);

    return 0;
}
