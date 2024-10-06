#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <ctime>
#include <stack>
#include <chrono>
#include <queue>
#include <unordered_map>

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
        TT.resize(vertices);
    }

    void addEdge(int u, int v)
    {
        adjList[u].insert(v);
        adjList[v].insert(u);
    }
    std::set<int> getVertices()
    {
        std::set<int> Vertices;
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
        set<int> size = getVertices();
        return Count + 1 < size.size() - 1; // 1 a mais por conta do metodo nao contar o nó raiz e da remoção
        }
    

    std::pair<Graph, Graph> separarGrafo(const std::vector<int> &Vertices, int verticeComum) {
    // Inicializa os subgrafos com o mesmo número de vértices que o grafo original
    Graph subgraph1(vertices);
    Graph subgraph2(vertices);
    // Verifica se há exatamente 3 vértices e se o vértice comum tem grau 2
    if (adjList[verticeComum].size() == getVertices().size() - 1) {
        set<int> sucessor1 = adjList[verticeComum];
        int cont = 0;
        // Adiciona o vértice comum (articulação) a ambos os subgrafos
        for(std::set<int>::iterator it = sucessor1.begin(); it != sucessor1.end(); it++){
            if(cont % 2 == 0){
                subgraph1.addEdge(verticeComum, *it);
            }
            else{
                subgraph2.addEdge(verticeComum, *it);
            }
            cont++;
        }
            
            return {subgraph1,subgraph2};
        
    }   else{
    // Conjunto de vértices do subgrafo 1
    std::set<int> verticesSet(Vertices.begin(), Vertices.end());

    // Se a lista de vértices estiver vazia, adicionar o vértice comum ao conjunto para garantir que o vértice seja separado corretamente
    if (verticesSet.empty()) {
        verticesSet.insert(verticeComum);
    }

    // Adiciona arestas ao subgrafo 1
    for (int v : Vertices) {
        for (int neighbor : adjList[v]) {
            // Só adiciona aresta se o vizinho também estiver no subgrafo 1
            if (verticesSet.find(neighbor) != verticesSet.end()) {
                subgraph1.addEdge(v, neighbor);
            }
        }
    }

    // Adiciona arestas ao subgrafo 2 (vértices que não estão no subgrafo 1)
    for (int v = 0; v < vertices; ++v) {
        if (verticesSet.find(v) == verticesSet.end()) { // Se o vértice não estiver no subgrafo 1
            for (int neighbor : adjList[v]) {
                if (verticesSet.find(neighbor) == verticesSet.end()) { // Ambos v e seu vizinho não estão no subgrafo 1
                    subgraph2.addEdge(v, neighbor);
                }
            }
        }
    }

    // Adiciona o vértice comum (articulação) a ambos os subgrafos
    for (int neighbor : adjList[verticeComum]) {
        if (verticesSet.find(neighbor) != verticesSet.end()) {
            subgraph1.addEdge(verticeComum, neighbor);
        } else {
            subgraph2.addEdge(verticeComum, neighbor);
        }
    }

    return {subgraph1, subgraph2};
    }
}


    bool BFS(int v, int w, const Graph& G, vector<int>& parent) {
    vector<bool> visited(G.adjList.size(), false);
    queue<int> q;

    // Verifica se o vértice inicial ou final estão vazios
    if (G.adjList[v].empty() || G.adjList[w].empty()) {
        return false;  // Não pode haver caminho se um dos vértices estiver vazio
    }

    // Inicializa a BFS a partir de 'v'
    visited[v] = true;
    parent[v] = -1;  // v não tem pai
    q.push(v);

    while (!q.empty()) {
        int curr = q.front();
        q.pop();

        // Se encontrar o vértice 'w', retorne true
        if (curr == w) {
            return true;
        }

        // Explora os vizinhos do vértice atual
        for (int neighbor : G.adjList[curr]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = curr;  // Registra o pai
                q.push(neighbor);
            }
        }
    }

    return false;  // Não encontrou caminho de 'v' até 'w'
}

bool findSecondPath(int v, int w, const Graph& G, const vector<int>& firstPath) {
    vector<bool> visited(G.adjList.size(), false);
    queue<int> q;
    vector<int> parent(G.adjList.size(), -1);

    // Verifica se o vértice inicial ou final estão vazios
    if (G.adjList[v].empty() || G.adjList[w].empty()) {
        return false;  // Não pode haver caminho se um dos vértices estiver vazio
    }

    // Marca os vértices do primeiro caminho como visitados (exceto 'v' e 'w')
    for (int vertex : firstPath) {
        if (vertex != v && vertex != w) {
            visited[vertex] = true;
        }
    }

    // Executa uma nova BFS para encontrar o segundo caminho disjunto
    visited[v] = true;
    q.push(v);

    while (!q.empty()) {
        int curr = q.front();
        q.pop();

        // Se encontrar 'w', um segundo caminho foi encontrado
        if (curr == w) {
            return true;
        }

        // Explora os vizinhos
        for (int neighbor : G.adjList[curr]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = curr;
                q.push(neighbor);
            }
        }
    }

    return false;  // Não encontrou segundo caminho disjunto
}

bool isThereTwoDisjointPaths(int v, int w, const Graph& G) {
    // Verifica se existem dois caminhos disjuntos entre v e w usando BFS

    // Primeiro caminho
    vector<bool> visited1(G.vertices, false);
    vector<int> parent1(G.vertices, -1);
    queue<int> q1;

    q1.push(v);
    visited1[v] = true;

    bool foundFirstPath = false;

    // Encontrando o primeiro caminho entre v e w
    while (!q1.empty() && !foundFirstPath) {
        int curr = q1.front();
        q1.pop();

        for (int neighbor : G.adjList[curr]) {
            if (!visited1[neighbor]) {
                visited1[neighbor] = true;
                parent1[neighbor] = curr;

                if (neighbor == w) {
                    foundFirstPath = true;
                    break;
                }

                q1.push(neighbor);
            }
        }
    }

    // Se não encontrou o primeiro caminho, não existem dois caminhos disjuntos
    if (!foundFirstPath) {
        return false;
    }

    // Marcar os vértices que fazem parte do primeiro caminho
    vector<bool> path1(G.vertices, false);
    int curr = w;
    while (curr != -1) {
        path1[curr] = true;
        curr = parent1[curr];
    }

    // Segundo caminho (evitar vértices do primeiro caminho)
    vector<bool> visited2(G.vertices, false);
    queue<int> q2;

    q2.push(v);
    visited2[v] = true;

    bool foundSecondPath = false;

    while (!q2.empty() && !foundSecondPath) {
        int curr = q2.front();
        q2.pop();

        for (int neighbor : G.adjList[curr]) {
            // Evitar usar vértices que fazem parte do primeiro caminho
            if (!visited2[neighbor] && !path1[neighbor]) {
                visited2[neighbor] = true;

                if (neighbor == w) {
                    foundSecondPath = true;
                    break;
                }

                q2.push(neighbor);
            }
        }
    }

    return foundSecondPath; // Retorna true se encontrou dois caminhos disjuntos
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

    const set<int> &GetSucessores(int vertice)
    {
        return adjList[vertice];
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
        { 
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
            Tarjan(g, i, -1, Components);
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

bool isComponent(Graph& g) {
    // Verifica se o subgrafo é vazio
    bool isEmpty = true;
    for (int i = 0; i < g.vertices; i++) {
        if (!g.adjList[i].empty()) {
            isEmpty = false;
            break;
        }
    }

    // Se o subgrafo for vazio, retorna false
    if (isEmpty) {
        return false;
    }

    // Verifica se existem dois caminhos disjuntos entre todos os pares de vértices conectados
    for (int i = 0; i < g.vertices; i++) {
        for (int j = i + 1; j < g.vertices; j++) {
            // Verifica se ambos os vértices têm vizinhos (não são vazios)
            if (!g.adjList[i].empty() && !g.adjList[j].empty()) {
                // Deve haver dois caminhos disjuntos entre i e j
                if (!g.isThereTwoDisjointPaths(i, j, g)) {
                    //std::cout << "Vértices " << i << " e " << j << " não possuem dois caminhos disjuntos." << std::endl;
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
void printAdjListArray(const std::vector<std::vector<std::set<int>>>& adjListArray) {
    for (int i = 0; i < adjListArray.size(); ++i) {
        std::cout << "Componente " << i + 1 << ":" << std::endl;
        for (int j = 0; j < adjListArray[i].size(); ++j) {
            if (!adjListArray[i][j].empty()) {
                std::cout << "  Vértice " << j << ": ";
                for (const auto& neighbor : adjListArray[i][j]) {
                    std::cout << neighbor << " ";
                }
                std::cout << std::endl;
            }
        }
        std::cout << std::endl;
    }
}
Graph buildProblem(){
    Graph G(4);
    G.addEdge(1,2);
    G.addEdge(2,3);
    return G;
}


int main()

{

    std::vector<int> verticesList = {100, 1000, 10000, 100000}; // 100, 1.000, 10.000 e 100.000 vértices

        int vertices = verticesList[0];
        vector<vector<pair<int, int>>> Components;
        vector<vector<set<int>>> subgraphs;
        auto startFunction = std::chrono::high_resolution_clock::now();
        auto endFunction1 = std::chrono::high_resolution_clock::now();
        auto endFunction2 = std::chrono::high_resolution_clock::now();
         for (int i = 0; i < 5; i++)
        {
          int edges = vertices * 1.5;
        Graph graph = generateGraph(vertices,edges);
        Graph graph2 = graph;
        getComponentByCut(graph, subgraphs);
        TarjanInicial(graph2, 0, -1, Components);
        printAdjListArray(subgraphs);
        printComponents(Components);
         Components.clear();
         subgraphs.clear();
         }

         
         std::chrono::duration<double, std::milli> duration = endFunction1 - startFunction;
         std::cout << "Tempo de execução usando o Algoritmo proposto por Tarjan com 30 grafos de : " << verticesList[0] << " vértices " << "e " << verticesList[0] * 1.5 << " arestas : " << (duration.count() / 10) << " Milissegundos" << std::endl;
    

    return 0;
}
