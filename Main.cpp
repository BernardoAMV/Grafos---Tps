#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <ctime>
#include <stack>

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
    void ChamadaInicial()
    {
        t = 0;
        int cont = 0;

        for (int i = 0; i < TD.size(); i++)
        {
            if (TD[i] == 0)
                buscaProfundidade(i, &cont);
        }

        cout << "O número de ciclos do grafo é: " << cont << endl;
    }

    void buscaProfundidade(int v, int *cont)
    {
        t += 1;
        TD[v] = t;
        for (int w : GetSucessores(v))
        {
            if (TD[w] == 0)
            {
                cout << "A aresta {(" << (v + 1) << "," << (w + 1) << ")} é de árvore" << endl;
                pai[w] = v;
                buscaProfundidade(w, cont);
            }
            else
            {
                if (TT[w] == 0 and w != pai[v])
                {
                    cout << "A aresta {(" << (v + 1) << ", " << (w + 1) << ")} é de retorno" << endl;
                    (*cont)++;
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
    vector<int> LOWPT;
    stack<pair<int, int>> edges;

    vector<int> GetSucessores(int vertice) const
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

Graph build_example_graph2()
{

    Graph g(3);
    g.addEdge(1, 2);
    g.addEdge(1, 3);

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

void printAllEdges(const Graph &g)
{
    for (int i = 0; i < g.vertices; ++i)
    {
        for (int neighbor : g.adjList[i])
        {
            cout << "(" << i << ", " << neighbor << ")" << endl;
        }
    }
}

void depthFirstSearch(Graph &g, int current, int target, vector<int> &visited, int original, bool &found)
{
    // Marca o nó atual como visitado
    visited[current] = 1;

    // Verifica se o nó alvo é alcançado
    if (current == target)
    {
        // Se o nó alvo for alcançado, começamos a buscar novamente para o nó original
        for (int neighbor : g.GetSucessores(current))
        {
            if (neighbor == original)
            {
                found = true; // Se o original for alcançado, indica que um ciclo foi encontrado
                return;
            }
        }
        // Continua a busca a partir do nó alvo
        for (int neighbor : g.GetSucessores(current))
        {
            if (!visited[neighbor])
            {
                depthFirstSearch(g, neighbor, target, visited, original, found);
            }
        }
        return;
    }

    // Continua a busca a partir do nó atual
    for (int neighbor : g.GetSucessores(current))
    {
        if (!visited[neighbor])
        {
            depthFirstSearch(g, neighbor, target, visited, original, found);
        }
    }
}

void findCycle(Graph &g, int S, int T)
{
    vector<int> visited(g.vertices, 0); // Inicializa o array visited com 0
    bool found = false;                 // Variável para verificar se o ciclo foi encontrado

    // Inicia a busca em profundidade
    depthFirstSearch(g, S, T, visited, S, found);

    // Verifica se um ciclo foi encontrado
    if (found)
    {
        cout << "Ciclo encontrado entre os vértices " << S << " e " << T << ": Yes" << endl; // Se um ciclo foi encontrado
    }
    else
    {
        cout << "Ciclo entre os vértices " << S << " e " << T << ": No" << endl; // Se não foi encontrado
    }
}

int main()

{

    // Defina o número de vértices e arestas
    // vector<vector<pair<int, int>>> Components;
    Graph graph = build_example_graph();
    // TarjanInicial(graph,1,-1,Components);

    // printComponents(Components);

    printAllEdges(graph);

    findCycle(graph, 1, 11);
    return 0;
}
