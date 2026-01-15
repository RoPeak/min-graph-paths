# Min-Graph-Paths

Java implementation of fundamental graph algorithms for finding paths in weighted and unweighted graphs. Includes **Depth-First Search (DFS)**, **Breadth-First Search (BFS)**, and **Dijkstra's shortest path algorithm**.

## Features

- **Graph Representation** - Adjacency list implementation
- **DFS** - Depth-First Search for graph traversal
- **BFS** - Breadth-First Search for shortest paths (unweighted)
- **Dijkstra's Algorithm** - Shortest paths in weighted graphs
- **Path Reconstruction** - Backtracking using predecessor pointers
- **File-based Graph Input** - Load graphs from text files

## Project Structure

```
Min-Graph-Paths/
├── Graph.java              # Main graph class with DFS/BFS
├── Vertex.java             # Vertex representation
├── AdjListNode.java        # Adjacency list node
├── dijkstra/
│   └── DijkstraAlgorithm.java
├── backtrack/
│   └── PathFinder.java
├── resource/
│   └── *.txt              # Sample graph files
└── README.md
```

## Algorithms Implemented

### 1. Depth-First Search (DFS)

**Purpose:** Explore as far as possible along each branch before backtracking

**Time Complexity:** O(V + E) where V = vertices, E = edges

**Space Complexity:** O(V) for recursion stack

**Use Cases:**

- Finding connected components
- Topological sorting
- Cycle detection
- Path existence

```java
graph.dfs();  // Traverses entire graph depth-first
```

### 2. Breadth-First Search (BFS)

**Purpose:** Explore all neighbors at current depth before moving deeper

**Time Complexity:** O(V + E)

**Space Complexity:** O(V) for queue

**Use Cases:**

- Shortest path in unweighted graphs
- Level-order traversal
- Finding all nodes within k distance
- Web crawling

```java
graph.bfs();  // Traverses entire graph breadth-first
```

**Key Property:** BFS finds the shortest path (minimum edges) in unweighted graphs

### 3. Dijkstra's Algorithm

**Purpose:** Find shortest paths from source to all vertices in weighted graphs

**Time Complexity:** O((V + E) log V) with priority queue

**Space Complexity:** O(V)

**Use Cases:**

- GPS navigation
- Network routing
- Social network analysis (degrees of separation)

**Constraints:** Requires non-negative edge weights

## Graph File Format

Input graphs are stored in text files with the following format:

```
<number_of_vertices>
<vertex1> <vertex2> <weight>
<vertex1> <vertex2> <weight>
...
```

**Example:**

```
5
0 1 4
0 2 1
1 3 1
2 1 2
2 3 5
3 4 3
```

This represents:

- 5 vertices (0-4)
- Edge from vertex 0 to 1 with weight 4
- Edge from vertex 0 to 2 with weight 1
- etc.

## Building and Running

### Compilation

```bash
javac *.java
javac dijkstra/*.java
javac backtrack/*.java
```

### Running

```bash
# Run DFS/BFS example
java Graph

# Run Dijkstra's algorithm
java dijkstra.DijkstraAlgorithm
```

> **Note:** Make sure to clean up `.class` files before committing:
>
> ```bash
> find . -name "*.class" -delete
> ```

## Usage Example

```java
// Create a graph with 5 vertices
Graph g = new Graph(5);

// Add edges (adjacency list)
g.getVertex(0).addToAdjList(1, 4);   // 0 -> 1, weight 4
g.getVertex(0).addToAdjList(2, 1);   // 0 -> 2, weight 1
g.getVertex(1).addToAdjList(3, 1);   // 1 -> 3, weight 1
// ... add more edges

// Perform traversals
g.dfs();  // Depth-first search
g.bfs();  // Breadth-first search

// Get path information from predecessors
for (int i = 0; i < g.size(); i++) {
    Vertex v = g.getVertex(i);
    System.out.println("Vertex " + i + " predecessor: " + v.getPredecessor());
}
```

## Class Overview

### `Graph.java`

- Manages array of vertices
- Implements DFS and BFS traversal algorithms
- Handles graph initialisation

**Key Methods:**

- `dfs()` - Depth-first search traversal
- `bfs()` - Breadth-first search traversal
- `Visit(Vertex v, int p)` - Recursive DFS helper
- `getVertex(int i)` - Access specific vertex

### `Vertex.java`

- Represents a single vertex in the graph
- Stores adjacency list of neighbors
- Tracks visited status and predecessor

**Key Methods:**

- `addToAdjList(int vertex, int weight)` - Add edge
- `getAdjList()` - Get neighbors
- `setVisited(boolean)` / `getVisited()` - Mark visited
- `setPredecessor(int)` / `getPredecessor()` - Path tracking

### `AdjListNode.java`

- Represents an edge in the adjacency list
- Stores destination vertex index and edge weight

### `DijkstraAlgorithm.java` (if implemented)

- Implements Dijkstra's shortest path algorithm
- Uses priority queue for efficient vertex selection
- Returns shortest distances and paths

## Algorithm Comparison

| Algorithm | Time Complexity | Space | Weighted? | Finds Shortest Path? |
| --------- | --------------- | ----- | --------- | -------------------- |
| DFS       | O(V + E)        | O(V)  | No        | No (finds _a_ path)  |
| BFS       | O(V + E)        | O(V)  | No        | Yes (min edges)      |
| Dijkstra  | O((V+E) log V)  | O(V)  | Yes       | Yes (min weight)     |

## Path Reconstruction

After running BFS or Dijkstra, reconstruct the path from source to destination:

```java
void printPath(int source, int destination, Graph g) {
    List<Integer> path = new ArrayList<>();
    int current = destination;

    while (current != source) {
        path.add(current);
        current = g.getVertex(current).getPredecessor();
    }
    path.add(source);

    Collections.reverse(path);
    System.out.println("Path: " + path);
}
```

## Requirements

- **Java Development Kit (JDK)** 8 or higher
- No external libraries required

## Future Enhancements

- [ ] A\* search algorithm
- [ ] Bellman-Ford (negative weight support)
- [ ] Floyd-Warshall (all-pairs shortest paths)
- [ ] Kruskal's/Prim's (minimum spanning tree)
- [ ] Visualisation of graph traversal
- [ ] Performance benchmarking suite
- [ ] Graph generation utilities

## Common Use Cases

### Finding Shortest Paths

- **Unweighted graphs** → Use BFS
- **Weighted graphs (non-negative)** → Use Dijkstra
- **Weighted graphs (negative allowed)** → Use Bellman-Ford

### Graph Exploration

- **Find all reachable nodes** → DFS or BFS
- **Check connectivity** → DFS from any vertex
- **Find cycles** → DFS with back edge detection

## References

- Introduction to Algorithms (CLRS) - Chapters 22-24
- Algorithms, 4th Edition by Robert Sedgewick
- [Dijkstra's Algorithm Wikipedia](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
- [Graph Traversal Visualisation](https://visualgo.net/en/dfsbfs)

## License

This is a personal educational project.
