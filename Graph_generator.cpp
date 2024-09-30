#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <ctime>

using namespace std;

class Graph {
public:
    Graph(int vertices) : vertices(vertices) {
        adjList.resize(vertices);
    }

    void addEdge(int u, int v) {
        adjList[u].insert(v);
        adjList[v].insert(u); // Para um grafo não direcionado
    }

    void printGraph() {
        for (int i = 0; i < vertices; ++i) {
            std::cout << "Vértice " << i << ": ";
            for (int neighbor : adjList[i]) {
                std::cout << neighbor << " ";
            }
            std::cout << std::endl;
        }
    }
    void ChamadaInicial(int primeiroVertice)
{
    t = 0;
    for (int i = 0; i < TD.size(); i++)
    {
        if (TD[i] == 0)
            buscaProfundidade(i, primeiroVertice);
    }
}




void buscaProfundidade(int v, int primeiroVertice)
{
    t += 1;
    TD[v] = t;
    for (int w : GetSucessores(v))
    {
        if (TD[w] == 0)
        {
            cout << "A aresta {(" << (v + 1) << "," << (w + 1) << ")} é de árvore" << endl;
            buscaProfundidade(w, primeiroVertice);
        }
        else
        {
            if (v == primeiroVertice)
            {
                if (TT[w] == 0 and w != pai[v])
                {
                    cout << "A aresta {(" << (v + 1) << ", " << (w + 1) << ")} é de retorno" << endl;
                }
            }
        }
    }
    t += 1;
    TT[v] = t;
}

public:
    int vertices;
    std::vector<std::set<int>> adjList; // Usando set para evitar arestas duplicadas
    int t = 0;
    vector<int> TD;
    vector<int> TT;
    vector<int> pai;

   vector<int> GetSucessores(int vertice){
    std:: vector<int> sucessores;
    for(int neighbor : adjList[vertice]) {
        sucessores.push_back(neighbor);
    }
    return sucessores;
}

};








Graph generateGraph(int vertices, int edges) {
    Graph graph(vertices);

    // Inicializando o random_device e mt19937
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, vertices - 1);

    std::set<std::pair<int, int>> edgeSet;

    while (edgeSet.size() < edges) {
        int u = dis(gen);
        int v = dis(gen);
        if (u != v) { // Evita loops
            edgeSet.insert({std::min(u, v), std::max(u, v)});
        }
    }

    for (const auto& edge : edgeSet) {
        graph.addEdge(edge.first, edge.second);
    }

    return graph;
}

int main() {
    // Defina o número de vértices e arestas
    std::vector<int> verticesList = {30};
    for (int vertices : verticesList) {
        int edges = vertices * 2; // Por exemplo, duas arestas para cada vértice
        Graph graph = generateGraph(vertices, edges);
        std::cout << "Grafo com " << vertices << " vértices e " << edges << " arestas gerado." << std::endl;
         graph.printGraph(); // Descomente para imprimir o grafo
    }

    return 0;
}
