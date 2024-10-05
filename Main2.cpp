#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <ctime>
#include <stack>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

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

    void printSuccessors(int vertice) const
    {
        std::cout << "Sucessores de " << vertice << ": ";
        for (int neighbor : adjList[vertice])
        {
            std::cout << neighbor << " ";
        }
        std::cout << std::endl;
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

    Graph g(4);
    g.addEdge(1, 2);
    g.addEdge(2, 3);

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

    g.printSuccessors(current);
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

Graph createDuplicatedGraph(Graph &g, int S, int T)
{
    int originalVertices = g.vertices;
    Graph newGraph(2 * originalVertices); // Novo grafo com nós duplicados

    for (int i = 0; i < originalVertices; ++i)
    {
        for (int neighbor : g.GetSucessores(i))
        {
            if (i != S && i != T)
            {                                                     // Ignora S e T
                newGraph.addEdge(i, originalVertices + neighbor); // Sender -> Receiver
            }
        }
        if (i != S && i != T)
        {
            newGraph.addEdge(originalVertices + i, i); // Receiver -> Sender
        }
    }

    // Conecta S e T no novo grafo
    for (int i = 0; i < originalVertices; ++i)
    {
        if (i == S)
        {
            newGraph.addEdge(S, originalVertices + S); // S -> sender_S
        }
        if (i == T)
        {
            newGraph.addEdge(originalVertices + T, T); // receiver_T -> T
        }
    }

    return newGraph;
}

bool bfs(Graph &g, int start, int target, vector<int> &pred)
{
    vector<bool> visited(g.vertices, false);
    queue<int> q;

    q.push(start);
    visited[start] = true;

    while (!q.empty())
    {
        int current = q.front();
        q.pop();

        if (current == target)
        {
            return true; // Caminho encontrado
        }

        for (int neighbor : g.GetSucessores(current))
        {
            if (!visited[neighbor])
            {
                visited[neighbor] = true;
                pred[neighbor] = current; // Armazena o predecessor
                q.push(neighbor);
            }
        }
    }
    return false; // Caminho não encontrado
}

void memorizeFlow(int target, vector<int> &pred, std::unordered_map<int, int> &flow)
{
    for (int cur = target; cur != -1; cur = pred[cur])
    {
        int prev = pred[cur];
        if (prev != -1)
        {
            flow[cur] = prev; // Memoriza o fluxo
        }
    }
}

void findTwoPaths(Graph &g, int S, int T)
{
    Graph newGraph = createDuplicatedGraph(g, S, T);
    vector<int> pred(newGraph.vertices, -1);
    std::unordered_map<int, int> flow;

    // Primeira BFS
    if (bfs(newGraph, S, T, pred))
    {
        memorizeFlow(T, pred, flow);
    }
    else
    {
        cout << "Caminho não encontrado de S a T" << endl;
        return;
    }

    // Seguindo a segunda BFS, respeitando as condições de fluxo
    fill(pred.begin(), pred.end(), -1); // Reseta o predecessor
    if (bfs(newGraph, S, T, pred))
    {
        memorizeFlow(T, pred, flow);
    }
    else
    {
        cout << "Caminho não encontrado de S a T na segunda busca" << endl;
        return;
    }

    // Construindo os caminhos finais
    vector<int> path1, path2;

    // Primeiro caminho
    for (int cur = T; cur != -1; cur = flow[cur])
    {
        path1.push_back(cur);
    }
    reverse(path1.begin(), path1.end());

    // Segundo caminho
    for (int cur = T; cur != -1; cur = flow[cur])
    {
        if (flow[cur] != -1)
        {
            path2.push_back(flow[cur]);
        }
    }
    reverse(path2.begin(), path2.end());

    // Exibir resultados
    cout << "Caminho 1 de S a T: ";
    for (int node : path1)
    {
        cout << node << " ";
    }
    cout << endl;

    cout << "Caminho 2 de T a S: ";
    for (int node : path2)
    {
        cout << node << " ";
    }
    cout << endl;
}

int main()
{
    // Exemplo de criação do grafo
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(1, 2);
    g.addEdge(2, 3);

    // Encontrar dois caminhos entre S e T
    findTwoPaths(g, 0, 3);
    return 0;
}
