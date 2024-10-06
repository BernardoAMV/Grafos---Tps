#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <ctime>
#include <stack>
#include <chrono>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

using namespace std;

class Graph
{
public:
    int vertices;
    vector<set<int>> adjList; // Usando set para evitar arestas duplicadas

    Graph(int vertices) : vertices(vertices)
    {
        adjList.resize(vertices); // Certifique-se de que o tamanho seja o correto
        TT.resize(vertices);
        TD.resize(vertices);
        pai.resize(vertices - 1);
        LOWPT.resize(vertices);
    }

    void addEdge(int u, int v)
    {
        if (u >= vertices || v >= vertices)
            return; // Proteção contra índices inválidos
        adjList[u].insert(v);
        adjList[v].insert(u); // Para um grafo não direcionado
    }

    const set<int> &getVizinhos(int v) const
    {
        return adjList[v];
    }

    int getNumVertices() const
    {
        return vertices;
    }

    set<int> getVertices()
    {
        set<int> Vertices;
        for (int i = 0; i < vertices; ++i)
        {

            if (!adjList[i].empty())
            {
                Vertices.insert(i);
            }
        }
        return Vertices;
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

    void ChamadaInicial(int &cont, vector<int> &subgraph)
    {
        t = 0;
        for (int i = 0; i < vertices; ++i)
        {
            if (!adjList[i].empty())
            { // Verifica se o vértice ainda tem vizinhos, ou seja, ainda está no grafo
                dfs(i, cont, subgraph);
                break;
            }
        }

        t = 0;
        TD.clear();
        pai.clear();
        TT.clear();
        LOWPT.clear();
        TD.resize(vertices);
        pai.resize(vertices - 1);
        LOWPT.resize(vertices);
        TT.resize(vertices);
    }

    void dfs(int v, int &cont, vector<int> &subgraph)
    {
        t += 1;
        TD[v] = t;
        subgraph.push_back(v);
        for (int w : getSucessores(v))
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
        set<int> size = getVertices();
        return Count + 1 < size.size() - 1; // 1 a mais por conta do metodo nao contar o nó raiz e da remoção
    }

    pair<Graph, Graph> separarGrafo(const vector<int> &Vertices, int verticeComum)
    {
        // Inicializa os subgrafos com o mesmo número de vértices que o grafo original
        Graph subgraph1(vertices);
        Graph subgraph2(vertices);
        // Verifica se há exatamente 3 vértices e se o vértice comum tem grau 2
        if (adjList[verticeComum].size() == getVertices().size() - 1)
        {
            set<int> sucessor1 = adjList[verticeComum];
            int cont = 0;
            // Adiciona o vértice comum (articulação) a ambos os subgrafos
            for (set<int>::iterator it = sucessor1.begin(); it != sucessor1.end(); it++)
            {
                if (cont % 2 == 0)
                {
                    subgraph1.addEdge(verticeComum, *it);
                }
                else
                {
                    subgraph2.addEdge(verticeComum, *it);
                }
                cont++;
            }

            return {subgraph1, subgraph2};
        }
        else
        {
            // Conjunto de vértices do subgrafo 1
            set<int> verticesSet(Vertices.begin(), Vertices.end());

            // Se a lista de vértices estiver vazia, adicionar o vértice comum ao conjunto para garantir que o vértice seja separado corretamente
            if (verticesSet.empty())
            {
                verticesSet.insert(verticeComum);
            }

            // Adiciona arestas ao subgrafo 1
            for (int v : Vertices)
            {
                for (int neighbor : adjList[v])
                {
                    // Só adiciona aresta se o vizinho também estiver no subgrafo 1
                    if (verticesSet.find(neighbor) != verticesSet.end())
                    {
                        subgraph1.addEdge(v, neighbor);
                    }
                }
            }

            // Adiciona arestas ao subgrafo 2 (vértices que não estão no subgrafo 1)
            for (int v = 0; v < vertices; ++v)
            {
                if (verticesSet.find(v) == verticesSet.end())
                { // Se o vértice não estiver no subgrafo 1
                    for (int neighbor : adjList[v])
                    {
                        if (verticesSet.find(neighbor) == verticesSet.end())
                        { // Ambos v e seu vizinho não estão no subgrafo 1
                            subgraph2.addEdge(v, neighbor);
                        }
                    }
                }
            }

            // Adiciona o vértice comum (articulação) a ambos os subgrafos
            for (int neighbor : adjList[verticeComum])
            {
                if (verticesSet.find(neighbor) != verticesSet.end())
                {
                    subgraph1.addEdge(verticeComum, neighbor);
                }
                else
                {
                    subgraph2.addEdge(verticeComum, neighbor);
                }
            }

            return {subgraph1, subgraph2};
        }
    }

    int t = 0;
    vector<int> TD;
    vector<int> TT;
    vector<int> pai;
    vector<int> LOWPT;
    stack<pair<int, int>> edges;

    vector<int> getSucessores(int vertice) const
    {
        vector<int> sucessores;
        for (int neighbor : adjList[vertice])
        {
            sucessores.push_back(neighbor);
        }
        return sucessores;
    }

    void printSucessores(int vertice) const
    {
        cout << "Sucessores de " << vertice << ": ";
        for (int neighbor : adjList[vertice])
        {
            cout << neighbor << " ";
        }
        cout << endl;
    }
};

class Edge
{
public:
    int u, v;
    Edge(int u, int v) : u(min(u, v)), v(max(u, v)) {} // Ordena u e v

    // O método abaixo é necessário para que o set de arestas funcione corretamente e o que ele faz é comparar as arestas
    bool operator==(const Edge &other) const
    {
        return u == other.u && v == other.v;
    }

    // O método abaixo é necessário para que o set de arestas funcione corretamente e o que ele faz é ordenar as arestas
    bool operator<(const Edge &other) const
    {
        return tie(u, v) < tie(other.u, other.v); // Comparação ordenada
    }
};

// Função para gerar um grafo aleatório
Graph generateGraph(int vertices, int edges)
{
    Graph graph(vertices);

    // Inicializando o random_device e mt19937
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, vertices - 1);

    set<pair<int, int>> edgeSet;

    while (edgeSet.size() < edges)
    {
        int u = dis(gen);
        int v = dis(gen);
        if (u != v)
        { // Evita loops
            edgeSet.insert({min(u, v), max(u, v)});
        }
    }

    for (const auto &edge : edgeSet)
    {
        graph.addEdge(edge.first, edge.second);
    }

    return graph;
}

// Grafo teórico introduzido nas orientações do trabalho
Graph zenilton()
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

// Grafo teórico de 3 vértices lineares acíclicos
Graph casoTeste1()
{

    Graph g(4);
    g.addEdge(1, 2);
    g.addEdge(2, 3);

    return g;
}

// Grafo teórico de 3 vértices lineares com ciclo (triângulo)
Graph casoTeste2()
{

    Graph g(4);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 1);

    return g;
}

class M1Broken
{ // Função para imprimir as arestas sem duplicatas
    void printAllEdges(const Graph &g)
    {
        vector<vector<bool>> printed(g.vertices, vector<bool>(g.vertices, false)); // Matriz de controle para arestas já impressas
        cout << "Arestas do grafo:" << endl;

        for (int i = 0; i < g.vertices; ++i)
        {
            for (int neighbor : g.adjList[i])
            {
                if (!printed[i][neighbor] && !printed[neighbor][i]) // Verifica se a aresta (i, neighbor) já foi impressa
                {
                    cout << "(" << i << ", " << neighbor << ")" << endl;
                    printed[i][neighbor] = printed[neighbor][i] = true; // Marca as arestas como impressas
                }
            }
        }
    }

    void depthFirstSearch(Graph &g, int current, int target, vector<int> &visited, int original, int &numPaths, bool &foundCycle, int parent)
    {
        // Marca o nó atual como visitado
        visited[current] = 1;

        // cout << "Visitando " << current << " , filho de " << parent << endl;

        // g.printSucessores(current);

        // cout << "Alvo atual: " << target << endl;

        // Verifica se o alvo está na vizinhança do nó atual
        bool targetInNeighbourhood = false;
        // Array de vizinhos não-explorados (armazenados para caso ciclo não seja encontrado no caminho realizado na primeira busca)

        for (int neighbor : g.getSucessores(current))
        {
            if (neighbor == target)
            {
                targetInNeighbourhood = true;
                break;
            }
        }

        // Se o alvo estiver na vizinhança, força a ida para o alvo
        if (targetInNeighbourhood)
        {
            // cout << "Alvo " << target << " encontrado na vizinhanca de " << current << ". Priorizando ida para " << target << endl;
            parent = current;
            visited[target] = 1;
            // cout << "Pai de " << target << " atualizado para " << parent << endl;
            ++numPaths;
            // cout << "Numero de caminhos: " << numPaths << endl;
            if (numPaths > 1)
            {
                foundCycle = true;
                return;
            }
            for (int neighbor : g.getSucessores(target))
            {
                // Evita revisitar o nó pai (necessário para evitar ciclos falsos)
                if (!visited[neighbor] && neighbor != parent)
                {
                    // cout << "No target == current. Iniciando nova busca partindo de " << target << " e indo para " << neighbor << endl;
                    depthFirstSearch(g, neighbor, original, visited, target, numPaths, foundCycle, target);
                }
                else if (neighbor == original && neighbor != parent)
                {
                    // cout << "Ciclo encontrado vindo de " << current << " para " << neighbor << endl;
                    foundCycle = true; // Se o original for alcançado, indica que um ciclo foi encontrado
                    return;
                }
            }
            return;
        }

        // cout << endl;

        // Continua a busca a partir do nó atual
        for (int neighbor : g.getSucessores(current))
        {
            if (neighbor == target && neighbor != parent)
            {
                ++numPaths;
                foundCycle = true;
            }
            // Evita revisitar o nó pai (necessário para evitar ciclos falsos)
            if (!visited[neighbor] && neighbor != parent)
            {
                depthFirstSearch(g, neighbor, target, visited, original, numPaths, foundCycle, current);
            }
        }
    }

    void printCycle(Graph &g, int S, int T)
    {
        vector<int> visited(g.vertices, 0); // Inicializa o array visited com 0
        bool foundCycle = false;            // Variável para verificar se o ciclo foi encontrado
        int numPaths = 0;

        // depthFirstSearch(Graph &g, int current, int target, vector<int> &visited, int original, bool &numPaths, bool &foundCycle, int parent)
        // Inicia a busca em profundidade
        depthFirstSearch(g, S, T, visited, S, numPaths, foundCycle, -1);

        // Verifica se um ciclo foi encontrado
        if (foundCycle)
        {
            cout << "CICLO ENCONTRADO ENTRE OS VERTICES " << S << " E " << T << endl; // Se um ciclo foi encontrado
        }
        else
        {
            cout << "NAO HA CICLO ENTRE OS VERTICES " << S << " E " << T << endl; // Se não foi encontrado
        }
    }

    void printAllCycles(Graph &g)
    {
        vector<vector<bool>> printed(g.vertices, vector<bool>(g.vertices, false)); // Matriz de controle para arestas já impressas

        for (int i = 1; i < g.vertices; ++i)
        {
            for (int j = 1; j < g.vertices; ++j)
            {
                if (i != j && !printed[i][j] && !printed[j][i])
                {
                    printCycle(g, i, j);
                    printed[i][j] = printed[j][i] = true; // Marca as arestas como impressas
                }
            }
        }
    }

    bool isCycle(Graph &g, int S, int T)
    {
        vector<int> visited(g.vertices, 0); // Inicializa o array visited com 0
        bool foundCycle = false;            // Variável para verificar se o ciclo foi encontrado
        int numPaths = 0;

        // depthFirstSearch(Graph &g, int current, int target, vector<int> &visited, int original, bool &numPaths, bool &foundCycle, int parent)
        // Inicia a busca em profundidade
        depthFirstSearch(g, S, T, visited, S, numPaths, foundCycle, -1);

        // Verifica se um ciclo foi encontrado
        if (foundCycle)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

class Conjuntos
{
public:
    vector<set<Edge>> componentes; // Usar set para evitar duplicação de arestas

    Conjuntos(int n)
    {
        componentes.resize(n);
    }

    void add(int component, const Edge &edge)
    {
        if (component < componentes.size())
        {
            componentes[component].insert(edge); // Arestas duplicadas serão ignoradas automaticamente
        }
    }
};

class M1
{
private:
    Graph &lista;
    vector<bool> visited;
    unordered_set<int> addedVertices;
    set<Edge> edges;

    bool specialDFS(int initial, int target, int avoiding)
    {
        stack<int> stack;
        stack.push(initial);

        while (!stack.empty())
        {
            int current = stack.top();
            stack.pop();

            if (current == target)
            {
                return true;
            }

            if (!visited[current])
            {
                visited[current] = true;

                for (int neighbor : lista.getVizinhos(current))
                {
                    if (neighbor == avoiding)
                        continue;
                    if (current == initial && neighbor == target)
                        continue;
                    if (!visited[neighbor])
                    {
                        stack.push(neighbor);
                    }
                }
            }
        }
        return false;
    }

    bool isNeighbourhoodInCycle(int v, int w)
    {
        visited.assign(lista.getNumVertices(), false); // Garante o tamanho correto de 'visited'
        return specialDFS(v, w, -1);
    }

    bool isParentAndSonInCycle(int v, int w, int son)
    {
        visited.assign(lista.getNumVertices(), false); // Garante o tamanho correto de 'visited'
        return specialDFS(v, w, son);
    }

    void addVertice(int start, unordered_set<int> &currentCycleVertices)
    {
        stack<int> stack;
        stack.push(start);

        while (!stack.empty())
        {
            int v = stack.top();
            stack.pop();

            if (!currentCycleVertices.count(v))
            {
                currentCycleVertices.insert(v);
                addedVertices.insert(v);

                for (int neighbor : lista.getVizinhos(v))
                {
                    if (currentCycleVertices.count(neighbor))
                        continue;
                    if (currentCycleVertices.size() < 2)
                    {
                        if (isNeighbourhoodInCycle(v, neighbor))
                        {
                            stack.push(neighbor);
                        }
                    }
                    else
                    {
                        if (isParentAndSonInCycle(start, neighbor, v))
                        {
                            stack.push(neighbor);
                        }
                    }
                }
            }
        }
    }
    // Função para imprimir os componentes sem duplicatas
    void printM1Components(const Conjuntos &componentes)
    {
        for (int i = 0; i < componentes.componentes.size(); i++)
        {
            if (!componentes.componentes[i].empty())
            {
                cout << "Componente " << i << ": ";
                for (const Edge &edge : componentes.componentes[i])
                {
                    cout << "(" << edge.u << ", " << edge.v << ") ";
                }
                cout << endl;
            }
        }
    }

public:
    M1(Graph &lista) : lista(lista) {}

    void execute()
    {
        Conjuntos conjuntos(lista.getNumVertices());
        unordered_set<int> currentCycleVertices;

        // Preencher o conjunto de arestas
        for (int i = 0; i < lista.getNumVertices(); i++)
        {
            for (int neighbor : lista.getVizinhos(i))
            {
                if (i != neighbor)
                { // Evitar adicionar arestas de laço
                    edges.insert(Edge(i, neighbor));
                }
            }
        }

        int currentComponent = -1;

        for (int i = 0; i < lista.getNumVertices(); i++)
        {
            if (!addedVertices.count(i))
            {
                currentCycleVertices.clear();
                visited.assign(lista.getNumVertices(), false); 

                addVertice(i, currentCycleVertices);

                currentComponent++;
                for (int vertex : currentCycleVertices)
                {
                    for (int neighbor : lista.getVizinhos(vertex))
                    {
                        if (currentCycleVertices.count(neighbor))
                        {
                            conjuntos.add(currentComponent, Edge(vertex, neighbor));
                            edges.erase(Edge(vertex, neighbor));
                        }
                    }
                }
            }
        }

        // Adiciona as arestas restantes que não formam ciclos
        for (const Edge &edge : edges)
        {
            currentComponent++;
            conjuntos.add(currentComponent, edge);
        }

        // Pode ocorrer de imprimir componentes vazios
        // Impressão também pode ser feita na main, mas por propósito de centralização do código, foi mantida aqui
        // printM1Components(conjuntos);
    }

    Conjuntos executeAndReturnComponents()
    {
        Conjuntos conjuntos(lista.getNumVertices());
        unordered_set<int> currentCycleVertices;

        // Preencher o conjunto de arestas
        for (int i = 0; i < lista.getNumVertices(); i++)
        {
            for (int neighbor : lista.getVizinhos(i))
            {
                if (i != neighbor)
                { // Evitar adicionar arestas de laço
                    edges.insert(Edge(i, neighbor));
                }
            }
        }

        int currentComponent = -1;

        for (int i = 0; i < lista.getNumVertices(); i++)
        {
            if (!addedVertices.count(i))
            {
                currentCycleVertices.clear();
                visited.assign(lista.getNumVertices(), false); // Reinicializar visited com o tamanho correto

                addVertice(i, currentCycleVertices);

                currentComponent++;
                for (int vertex : currentCycleVertices)
                {
                    for (int neighbor : lista.getVizinhos(vertex))
                    {
                        if (currentCycleVertices.count(neighbor))
                        {
                            conjuntos.add(currentComponent, Edge(vertex, neighbor));
                            edges.erase(Edge(vertex, neighbor));
                        }
                    }
                }
            }
        }

        // Adiciona as arestas restantes que não formam ciclos
        for (const Edge &edge : edges)
        {
            currentComponent++;
            conjuntos.add(currentComponent, edge);
        }

        // printM1Components(conjuntos);
        return conjuntos;
    }
};

class M2
{
    Graph &lista;
    vector<vector<set<int>>> subgraphs;

private:
    bool BFS(int v, int w, const Graph &G, vector<int> &parent)
    {
        vector<bool> visited(G.adjList.size(), false);
        queue<int> q;

        // Verifica se o vértice inicial ou final estão vazios
        if (G.adjList[v].empty() || G.adjList[w].empty())
        {
            return false; // Não pode haver caminho se um dos vértices estiver vazio
        }

        // Inicializa a BFS a partir de 'v'
        visited[v] = true;
        parent[v] = -1; // v não tem pai
        q.push(v);

        while (!q.empty())
        {
            int curr = q.front();
            q.pop();

            // Se encontrar o vértice 'w', retorne true
            if (curr == w)
            {
                return true;
            }

            // Explora os vizinhos do vértice atual
            for (int neighbor : G.adjList[curr])
            {
                if (!visited[neighbor])
                {
                    visited[neighbor] = true;
                    parent[neighbor] = curr; // Registra o pai
                    q.push(neighbor);
                }
            }
        }

        return false; // Não encontrou caminho de 'v' até 'w'
    }

    bool isThereTwoDisjointPaths(int v, int w, const Graph &G)
    {
        // Verifica se existem dois caminhos disjuntos entre v e w usando BFS

        // Primeiro caminho
        vector<bool> visited1(G.vertices, false);
        vector<int> parent1(G.vertices, -1);
        queue<int> q1;

        q1.push(v);
        visited1[v] = true;

        bool foundFirstPath = false;

        // Encontrando o primeiro caminho entre v e w
        while (!q1.empty() && !foundFirstPath)
        {
            int curr = q1.front();
            q1.pop();

            for (int neighbor : G.adjList[curr])
            {
                if (!visited1[neighbor])
                {
                    visited1[neighbor] = true;
                    parent1[neighbor] = curr;

                    if (neighbor == w)
                    {
                        foundFirstPath = true;
                        break;
                    }

                    q1.push(neighbor);
                }
            }
        }

        // Se não encontrou o primeiro caminho, não existem dois caminhos disjuntos
        if (!foundFirstPath)
        {
            return false;
        }

        // Marcar os vértices que fazem parte do primeiro caminho
        vector<bool> path1(G.vertices, false);
        int curr = w;
        while (curr != -1)
        {
            path1[curr] = true;
            curr = parent1[curr];
        }

        // Segundo caminho (evitar vértices do primeiro caminho)
        vector<bool> visited2(G.vertices, false);
        queue<int> q2;

        q2.push(v);
        visited2[v] = true;

        bool foundSecondPath = false;

        while (!q2.empty() && !foundSecondPath)
        {
            int curr = q2.front();
            q2.pop();

            for (int neighbor : G.adjList[curr])
            {
                // Evitar usar vértices que fazem parte do primeiro caminho
                if (!visited2[neighbor] && !path1[neighbor])
                {
                    visited2[neighbor] = true;

                    if (neighbor == w)
                    {
                        foundSecondPath = true;
                        break;
                    }

                    q2.push(neighbor);
                }
            }
        }

        return foundSecondPath; // Retorna true se encontrou dois caminhos disjuntos
    }

    bool isComponent(Graph &g)
    {
        // Verifica se o subgrafo é vazio
        bool isEmpty = true;
        for (int i = 0; i < g.vertices; i++)
        {
            if (!g.adjList[i].empty())
            {
                isEmpty = false;
                break;
            }
        }

        // Se o subgrafo for vazio, retorna false
        if (isEmpty)
        {
            return false;
        }

        // Verifica se existem dois caminhos disjuntos entre todos os pares de vértices conectados
        for (int i = 0; i < g.vertices; i++)
        {
            for (int j = i + 1; j < g.vertices; j++)
            {
                // Verifica se ambos os vértices têm vizinhos (não são vazios)
                if (!g.adjList[i].empty() && !g.adjList[j].empty())
                {
                    // Deve haver dois caminhos disjuntos entre i e j
                    if (!isThereTwoDisjointPaths(i, j, g))
                    {
                        // cout << "Vértices " << i << " e " << j << " não possuem dois caminhos disjuntos." << endl;
                        return false; // Se não houver dois caminhos disjuntos, não é um componente
                    }
                }
            }
        }

        return true; // Se passou por todos os pares de vértices conectados, é um componente
    }

    void getComponentByCut(Graph g, vector<vector<set<int>>> &Blocos)
    {
        stack<Graph> pilhaDeGrafos;
        pilhaDeGrafos.push(g);
        vector<int> vertices;

        while (!pilhaDeGrafos.empty())
        {
            Graph gLinha = pilhaDeGrafos.top();
            pilhaDeGrafos.pop();

            bool encontrouArticulacao = false;

            for (int v : gLinha.getVertices())
            {
                int a = 0;

                if (gLinha.isArticulation(v, vertices))
                {

                    auto [subgrafo1, subgrafo2] = gLinha.separarGrafo(vertices, v);

                    if (isComponent(subgrafo1) && isComponent(subgrafo2))
                    {

                        Blocos.push_back(subgrafo1.adjList);
                        Blocos.push_back(subgrafo2.adjList);
                    }
                    else if (isComponent(subgrafo1))
                    {

                        Blocos.push_back(subgrafo1.adjList);
                        pilhaDeGrafos.push(subgrafo2);
                    }
                    else if (isComponent(subgrafo2))
                    {
                        Blocos.push_back(subgrafo2.adjList);
                        pilhaDeGrafos.push(subgrafo1);
                    }
                    else
                    {

                        pilhaDeGrafos.push(subgrafo1);
                        pilhaDeGrafos.push(subgrafo2);
                    }

                    encontrouArticulacao = true;
                    break;
                }
                vertices.clear();
            }

            if (!encontrouArticulacao)
            {
                Blocos.push_back(gLinha.adjList);
            }

            vertices.clear();
        }
    }
    void printAdjListArray(const vector<vector<set<int>>> &adjListArray)
    {
        for (int i = 0; i < adjListArray.size(); ++i)
        {
            cout << "Componente " << i + 1 << ":" << endl;
            for (int j = 0; j < adjListArray[i].size(); ++j)
            {
                if (!adjListArray[i][j].empty())
                {
                    cout << "  Vértice " << j << ": ";
                    for (const auto &neighbor : adjListArray[i][j])
                    {
                        cout << neighbor << " ";
                    }
                    cout << endl;
                }
            }
            cout << endl;
        }
    }

public:
    M2(Graph &lista) : lista(lista) {}

    void execute()
    {
        getComponentByCut(lista, subgraphs);
        // printAdjListArray(subgraphs);
    }
};

class M3
{
private:
    Graph &lista;
    vector<vector<pair<int, int>>> Components;

    void Tarjan(Graph &g, int v, int u, vector<vector<pair<int, int>>> &Components)
    {
        g.t += 1;
        g.TD[v] = g.t;
        g.LOWPT[v] = g.TD[v];
        for (int w : g.getSucessores(v))
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
            cout << "Componente " << i + 1 << ": ";
            for (const auto &edge : Components[i])
            {
                cout << "(" << edge.first << ", " << edge.second << ") ";
            }
            cout << endl;
        }
    }

public:
    M3(Graph &lista, vector<vector<pair<int, int>>> Components) : lista(lista) {}

    void execute()
    {
        TarjanInicial(lista, 1, -1, Components);
        // printComponents(Components);
    }
};

int main()
{
    // Graph g = zenilton();
    // Graph g = generateGraph(10000, 2000);
    // vector<vector<pair<int, int>>> Components;

    // cout << "METODO 1" << endl;
    // M1 metodo1(g);
    // metodo1.execute();

    // cout << "FIM METODO 1" << endl;

    // cout << "METODO 2" << endl;

    // M2 metodo2(g);
    // metodo2.execute();

    // cout << "FIM METODO 2" << endl;

    // cout << "METODO 3" << endl;

    // M3 metodo3(g, Components);

    // metodo3.execute();

    // cout << "FIM METODO 3" << endl;

    vector<int> verticesList = {10, 100, 500}; // 100, 1.000, 10.000 e 100.000 vértices

    int vertices = verticesList[0];
    vector<vector<set<int>>> subgraphs;

    for (int j = 0; j < verticesList.size(); j++)
    {
        int vertices = verticesList[j];
        vector<vector<pair<int, int>>> Components;
        auto startFunction = chrono::high_resolution_clock::now();
        auto endFunction1 = chrono::high_resolution_clock::now();
        auto endFunction2 = chrono::high_resolution_clock::now();
        auto endFunction3 = chrono::high_resolution_clock::now();

        for (int i = 0; i < 5; i++)
        {
            int edges = vertices * 1.5; // Por exemplo, duas arestas para cada vértice
            Graph graph = generateGraph(vertices, edges);
            Graph graph2 = graph;
            Graph graph3 = graph;

            M1 metodo1(graph);
            M2 metodo2(graph2);
            M3 metodo3(graph3, Components);
            startFunction = chrono::high_resolution_clock::now();
            metodo1.execute();
            endFunction1 = chrono::high_resolution_clock::now();
            metodo2.execute();
            endFunction2 = chrono::high_resolution_clock::now();
            metodo3.execute();
            endFunction3 = chrono::high_resolution_clock::now();

            Components.clear();
            subgraphs.clear();
        }

        auto endFunction = chrono::high_resolution_clock::now();
        chrono::duration<double, milli> duration1 = endFunction1 - startFunction;
        chrono::duration<double, milli> duration2 = endFunction2 - endFunction1;
        chrono::duration<double, milli> duration3 = endFunction3 - endFunction2;

        cout << "[1]. Tempo de execução usando um método que verifica a existência de dois caminhos internamente disjuntos (ou um ciclo) entre cada par de vértices do grafo, com 30 grafos de : " << verticesList[j] << " vértices " << "e " << verticesList[j] * 1.5 << " arestas : " << (duration1.count() / 5) << " Milissegundos" << endl;

        cout << "[2]. Tempo de execução identificando articulações e testando a conectividade após a remoção de cada vértice, com 5 grafos de : " << verticesList[j] << " vértices " << "e " << verticesList[j] * 1.5 << " arestas : " << (duration2.count() / 5) << " Milissegundos" << endl;

        cout << "[3]. Tempo de execução usando o Algoritmo proposto por Tarjan com 5 grafos de : " << verticesList[j] << " vértices " << "e " << verticesList[j] * 1.5 << " arestas : " << (duration3.count() / 5) << " Milissegundos" << endl;
    }

    return 0;
}