#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <ctime>
#include <stack>
#include <chrono>
#include <queue>

using namespace std;

class Graph
{
public:
    Graph(int vertices) : vertices(vertices)
    {
        adjList.resize(vertices);
        TD.resize(vertices);
        pai.resize(vertices - 1);
        LOWPT.resize(vertices);
    }

    void addEdge(int u, int v)
    {
        adjList[u].insert(v);
        adjList[v].insert(u); // Para um grafo não direcionado
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
            std::cout << "Vértice " << i << ": ";
            for (int neighbor : adjList[i])
            {
                std::cout << neighbor << " ";
            }
            std::cout << std::endl;
        }
    }

    void ChamadaInicial(int &cont, vector<int> &subgraph)
    {
        t = 0;

        for (int i = 0; i < TD.size(); i++)
        {
            if (TD[i] == 0)
                dfs(i, cont, subgraph);
        }
        t = 0;
        TD.clear();
        adjList.clear();
        TD.clear();
        pai.clear();
        LOWPT.clear();
    }

    void dfs(int v, int &cont, vector<int> &subgraph)
    {
        t += 1;
        TD[v] = t;
        subgraph.push_back(v);
        for (int w : GetSucessores(v))
        {
            if (TD[w] == 0)
            {
                // cout << "A aresta {(" << (v + 1) << "," << (w + 1) << ")} é de árvore" << endl;
                pai[w] = v;
                cont++;
                dfs(w, cont, subgraph);
            }
            else
            {
                if (TT[w] == 0 and w != pai[v])
                {
                    // cout << "A aresta {(" << (v + 1) << ", " << (w + 1) << ")} é de retorno" << endl;
                }
            }
        }
        t += 1;
        TT[v] = t;
    }

    bool isArticulation(int vertice, vector<int> &subgraph)
    {
        set<int> originalAdjList = adjList[vertice];
        for (int neighbor : originalAdjList)
        {
            adjList[neighbor].erase(vertice);
        }
        adjList[vertice].clear();

        int Count = 0;

        ChamadaInicial(Count, subgraph);

        adjList[vertice] = originalAdjList;
        for (int neighbor : originalAdjList)
        {
            adjList[neighbor].insert(vertice);
        }

        return Count < vertices - 1; // Um vértice a menos por causa da remoção
    }

    pair<Graph, Graph> separarGrafo(vector<int> &vertexList)
    {
        // Criação dos subgrafos com o mesmo número de vértices
        Graph subgraph1(vertices);
        Graph subgraph2(vertices);

        // Cria conjuntos para checar rapidamente se um vértice está no vertexList
        set<int> vertexSet(vertexList.begin(), vertexList.end());

        // Para cada vértice no grafo original
        for (int u = 0; u < vertices; ++u)
        {
            for (int v : adjList[u])
            {
                // Se ambos os vértices estão na lista, adiciona a aresta ao subgrafo 1
                if (vertexSet.count(u) && vertexSet.count(v))
                {
                    subgraph1.addEdge(u, v);
                }
                // Se ambos os vértices não estão na lista, adiciona ao subgrafo 2
                else if (!vertexSet.count(u) && !vertexSet.count(v))
                {
                    subgraph2.addEdge(u, v);
                }
            }
        }

        return make_pair(subgraph1, subgraph2);
    }
    bool BFS(int v, int w)
    {
        
        vector<int> L;
        L.resize(vertices);
        vector<int> nivel;
        nivel.resize(vertices);
        pai.clear();

        queue<int> fila;
        

        for(int i = 0; i < vertices; i++){
            t += 1;
            fila.push(i);
            while(!fila.empty()){
                for(int wL : GetSucessores(i)){
                    if(L[wL] == 0){

                    }
                }
            }


        }

    }

public:
    int vertices;
    std::vector<std::set<int>> adjList; // Usando set para evitar arestas duplicadas
    int t = 0;
    vector<int> TD;
    vector<int> TT;
    vector<int> pai;
    vector<int> LOWPT;
    stack<pair<int, int>> edges;

    vector<int> GetSucessores(int vertice)
    {
        std::vector<int> sucessores;
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

    // Inicializando o random_device e mt19937
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, vertices - 1);

    std::set<std::pair<int, int>> edgeSet;

    while (edgeSet.size() < edges)
    {
        int u = dis(gen);
        int v = dis(gen);
        if (u != v)
        { // Evita loops
            edgeSet.insert({std::min(u, v), std::max(u, v)});
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

void Tarjan(Graph &g, int v, int u, vector<vector<pair<int, int>>> &Components)

{
    g.t += 1;
    g.TD[v] = g.t;
    g.LOWPT[v] = g.TD[v];
    for (int w : g.GetSucessores(v))
    {
        if (g.TD[w] == 0)
        {
            g.edges.push(make_pair(v, w));
            Tarjan(g, w, v, Components);
            g.LOWPT[v] = min(g.LOWPT[v], g.LOWPT[w]);
            if (g.LOWPT[w] >= g.TD[v]) // identificação do componente
            {
                vector<pair<int, int>> Component;
                pair<int, int> edge;
                int u1, u2;

                do
                {
                    edge = g.edges.top();
                    g.edges.pop();
                    Component.push_back(edge);
                    u1 = edge.first;
                    u2 = edge.second;
                } while (!g.edges.empty() && g.TD[u1] >= g.TD[w]);

                Components.push_back(Component);
            }
        }
        else if (w != u and g.TD[w] < g.TD[v])
        {
            g.edges.push(make_pair(v, w));
            g.LOWPT[v] = min(g.LOWPT[v], g.TD[w]);
        }
    }
}

void TarjanInicial(Graph &g, int v, int u, vector<vector<pair<int, int>>> &Components)
{
    for (int i = 0; i < g.TD.size(); i++)
    {
        if (g.TD[i] == 0)
        {
            g.t = 0;
            while (!g.edges.empty())
            {
                g.edges.pop();
            }
            Tarjan(g, v, -1, Components);
        }
    }
}

void printComponents(const vector<vector<pair<int, int>>> &Components)
{
    for (int i = 0; i < Components.size(); ++i)
    {
        cout << "Componente " << i + 1 << ": " << endl;
        for (const auto &edge : Components[i])
        {
            cout << "(" << edge.first << ", " << edge.second << ")" << endl;
        }
        cout << endl;
    }
}

bool isComponent(Graph g)
{
}

void getComponentByCut(Graph g, vector<vector<pair<int, int>>> &Components, vector<int> &vertices)
{
    stack<Graph> pilhaDeGrafos;
    pilhaDeGrafos.push(g);
    for (int i = 0; i < g.TD.size(); i++)
    {
        Graph gLinha = pilhaDeGrafos.top();
        pilhaDeGrafos.pop();

        if (gLinha.isArticulation(i, vertices))
        {
            auto [founded, notFounded] = g.separarGrafo(vertices);
            int j = 0;
            while (!founded.isArticulation(j))
        }
    }
}

int main()

{

    std::vector<int> verticesList = {100, 1000, 10000, 100000}; // 100, 1.000, 10.000 e 100.000 vértices

    for (int j = 0; j < verticesList.size(); j++)
    {
        int vertices = verticesList[j];
        vector<vector<pair<int, int>>> Components;
        auto startFunction = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 30; i++)
        {
            int edges = vertices * 1.5; // Por exemplo, duas arestas para cada vértice
            Graph graph = generateGraph(vertices, edges);
            TarjanInicial(graph, 0, -1, Components);
            // printComponents(Components); Descomente se quiser ver os componentes e suas arestas
            Components.clear();
        }

        auto endFunction = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = endFunction - startFunction;
        std::cout << "Tempo de execução usando o Algoritmo proposto por Tarjan com 30 grafos de : " << verticesList[j] << " vértices " << "e " << verticesList[j] * 1.5 << " arestas : " << (duration.count() / 30) << " Milissegundos" << std::endl;
    }

    return 0;
}
