# Python3 program to print DFS traversal  
# from a given given graph
import numpy as np

# pilha da ordenação
stack = []


def addEdge(u, v, w, graph, adj):
    graph[u, v] = w
    adj[u].append(v)


def show(graph, adj):
    print("\n\n--------------------------Grafo--------------------------")
    print('\t', end='')
    for i in range(len(graph)):
        print(i, end='\t')
    for i in range(len(graph)):
        print('\n', i, end='->\t')
        for j in range(len(graph)):
            print(graph[i, j], end='\t')
    print("\n---------------------------------------------------------")


    print('\n\n')

    print("-----------------Adjacencias---------------")
    print(adj)
    print("-------------------------------------------")

def topo(u, visited, graph, adj):
    visited[u] = True

    for v in adj[u]:
        if graph[u, v]:
            if not (visited[v]):
                topo(v, visited, graph, adj)

    stack.append(u)


def longestPath(start, visited, graph, adj, N):
    # Vetores de distancia
    b_dist = [0] * (N)
    s_dist = [0] * (N)

    # Maior distancia partindo do nó start
    d = 0
    # Nó mais longe de start
    target = 0

    # Organiza o grafo
    for i in range(N):

        if not visited[i]:
            topo(i, visited, graph, adj)

    # Atribui distancia de saida infinita para os nós menos para o primeiro
    for i in range(N):
        b_dist[i] = -1
        b_dist[start] = 0

    path = [0]

    while (len(stack) != 0):

        # Começa pelo ultimo vertice
        nextV = stack[-1]
        # Libera a pilha
        stack.pop()

        if (b_dist[nextV] != -1):
            for i in range(N):
                # calcula a maior distancia do no inicial ate o nó final
                if (b_dist[nextV] + graph[nextV][i] > b_dist[nextV]):
                    if (b_dist[nextV] + graph[nextV][i] > b_dist[i]):
                        b_dist[i] = b_dist[nextV] + graph[nextV][i]



    for i in range(N):

        if (d < b_dist[i]):
            target = i
        d = max(d, b_dist[i])
    #tamanho e onde o fim termina
    return d, target


def size_path(path, graph, adj):
    last = path[0]
    path_size = 0
    for i in path:
        path_size += graph[last][i]
        last = i
    return path_size


def find_all_paths(start, end, path, size, graph, adj):
    path = path + [start]
    if start == end:
        return [path]

    if not adj[start]:
        return []

    paths = []
    for node in adj[start]:
        if node not in path:
            newpaths = find_all_paths(node, end, path, size, graph, adj)
            for newpath in newpaths:

                if size_path(newpath, graph, adj) == size:
                    paths.append(newpath)
    return paths


def main():
    N = int(input('Entre o numero de arestas: '))

    graph = np.zeros([N, N]).astype(int)

    adj = [[] for i in range(N)]

    edge = [0]
    while True:
        edge = input('Entre com a aresta[Enter para sair]:').split()
        print(edge)
        if edge == []: break
        addEdge(int(edge[0]), int(edge[1]), int(edge[2]), graph, adj)

    show(graph, adj)

    # vetor de visitados
    visited = [False] * N
    start = 0
    size = 0
    max_size = 0
    path = []
    max_path = []
    #for (i =0 ; i<n ; i++);
    for i in range(N):
        size, end_point = longestPath(i, visited, graph, adj, N)

        if max_size < size:
            max_size = size
            max_path = find_all_paths(i, end_point, path, size, graph, adj)

        path = []
        visited = [False] * N

    print("\n\n----A maior distacia é", max_size)
    print("\n----E o caminho é ", max_path[0])



if __name__ == '__main__':
    main()
