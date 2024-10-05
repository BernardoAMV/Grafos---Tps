#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <ctime>
#include <stack>
#include <memory>

using namespace std;

class Graph
{
public:
    Graph(int vertices) : vertices(vertices)
    {
        adjList.resize(vertices);
        TT.resize(vertices);
        TD.resize(vertices);
        pai.resize(vertices - 1);
        LOWPT.resize(vertices);
    }

    void addEdge(int u, int v)
    {
        adjList[u].insert(v);
        adjList[v].insert(u); // Para um grafo não direcionado
    }

    void removeEdge(int u, int v)
    {
        adjList[u].erase(v);
        adjList[v].erase(u);
    }

    void removeVertice(int u)
    {
        for (int i = 0; i < vertices; i++)
        {
            adjList[i].erase(u);
        }
        adjList[u].clear();
    }

    vector<int> getVertices()
    {
        vector<int> verticesArray;
        for (int i = 0; i < vertices; ++i)
        {
            verticesArray.push_back(i); // Adiciona cada vértice ao vetor
        }
        return verticesArray;
    }

    void printGraph()
    {
        for (int i = 0; i < vertices; ++i)
        {
            cout << "Vértice " << i << ": ";
            for (int neighbor : adjList[i])
            {
                cout << neighbor << " ";
            }
            cout << endl;
        }
    }

public:
    int vertices;
    vector<set<int>> adjList; // Usando set para evitar arestas duplicadas
    int t = 0;
    vector<int> TD;
    vector<int> TT;
    vector<int> pai;
    vector<int> LOWPT;
    stack<pair<int, int>> edges;

    vector<int> GetSucessores(int vertice)
    {
        vector<int> sucessores;
        for (int neighbor : adjList[vertice])
        {
            sucessores.push_back(neighbor);
        }
        return sucessores;
    }
};

Graph generateGraph(int vertices, int edges)
{
    Graph graph(vertices);

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, vertices - 1);

    set<pair<int, int>> edgeSet;

    while (edgeSet.size() < edges)
    {
        int u = dis(gen);
        int v = dis(gen);
        if (u != v)
        {
            edgeSet.insert({min(u, v), max(u, v)});
        }
    }

    for (const auto &edge : edgeSet)
    {
        graph.addEdge(edge.first, edge.second);
    }

    return graph;
}

Graph build_example_graph()
{
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

int main()
{
    int userOption = 0;
    int vertexNumber = 0;
    int edgeNumber = 0;

    while (userOption != 5)
    {
        if (userOption != 5)
        {
            cout << "Por favor, selecione o tamanho do grafo desejado:\n";
            cout << " 1 - 100 \n 2 - 1000 \n 3 - 10000 \n 4 - 100000 \n 5 - sair\n";

            cin >> userOption;

            switch (userOption)
            {
            case 1:
                vertexNumber = 100;
                edgeNumber = 500; // Defina o número de arestas que você deseja gerar
                break;
            case 2:
                vertexNumber = 1000;
                edgeNumber = 5000;
                break;
            case 3:
                vertexNumber = 10000;
                edgeNumber = 50000;
                break;
            case 4:
                vertexNumber = 100000;
                edgeNumber = 500000;
                break;
            case 5:
                // caso em que a função build_example é usada
                break;
            case 6:
                cout << "Saindo...\n";
                break;
            default:
                cout << "Opção inválida. Tente novamente.\n";
                break;
            }

            if (userOption >= 1 && userOption <= 4)
            {
                Graph graph = generateGraph(vertexNumber, edgeNumber);
                graph.printGraph();
            }
            else if (userOption == 5)
            {
                Graph graph = build_example_graph();
                graph.printGraph();
            }
        }
    }

    return 0;
}
