# Graph-Evolve

## Overview

Graph-Evolve is a Python project dedicated to the creation, manipulation, and evolution of graph structures. It leverages genetic algorithms to explore and evolve graphs for various tasks such as grasping, stacking, and reaching. The project integrates with the Pyevolve library for genetic algorithms and NetworkX for graph creation and manipulation.

## Features

- **Genetic Algorithms**: Uses Pyevolve for optimizing graph structures.
- **Graph Manipulation**: Utilizes NetworkX for constructing and analyzing graphs.
- **State Machines**: Integrates with SMACH for creating and managing state machines.
- **Simulation Environments**: Includes simulators for various tasks (grasping, stacking, reaching).
- **Flexible and Extensible**: Easily extendable to new tasks and objective functions.

## Installation Instructions

### Prerequisites

Ensure you have Python installed on your system. You will also need the following Python libraries:

- Pyevolve
- NetworkX
- SMACH
- pyrecurrentnet
- Other dependencies as mentioned in the code summaries and source files.

### Steps

1. **Clone the repository:**

    ```bash
    git clone https://github.com/Yuriy/Graph-Evolve.git
    ```
    
2. **Navigate to the project directory:**

    ```bash
    cd Graph-Evolve
    ```

3. **Install dependencies:**

    You can use `pip` to install the required dependencies:

    ```bash
    pip install pyevolve networkx smach pyrecurrentnet
    ```
   
4. **Update paths as necessary:**

    Depending on your operating system and project setup, you might need to update the `sys.path.append` paths in the source files:

    - For Windows:
    
        ```python
        sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')
        sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src\\pyevolve')
        ```

    - For Linux:
    
        ```python
        sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src')
        sys.path.append('/home/pezzotto/Projects/Graph-Evolve/src/pyevolve')
        ```

## Usage Examples

Here are some examples of how to use the project. 

### Example: Evolving a Graph for Grasping

```python
import sys
sys.path.append('/path/to/Graph-Evolve/src')

from pyevolve import GSimpleGA, G1DList, Selectors, Initializators, Mutators, DBAdapters, Crossovers
import graph_evolve
from graph_evolve import graph_genome, smach_grasp

# Initialize the genetic algorithm
genome = G1DList.G1DList(10)
genome.evaluator.set(graph_evolve.evaluate)
genome.mutator.set(Mutators.G1DListMutatorSwap)
genome.initializator.set(Initializators.G1DListInitializatorInteger)

ga = GSimpleGA.GSimpleGA(genome)
ga.setGenerations(100)
ga.setPopulationSize(80)
ga.setCrossoverRate(0.9)
ga.setMutationRate(0.02)

ga.evolve(freq_stats=10)
best = ga.bestIndividual()
print(best)
```

## Code Summary

The project structure and key files are as follows:

- **cluster_aware**: Contains scripts for evolving graphs with awareness to clusters.
    - `evolve_for_exploring.py`
    - `evolve_for_exploring_grasping.py`
    - `evolve_for_grasping.py`
    - `evolve_for_stacking.py`
- **evolutions**: Contains various evolution scripts for different tasks.
    - `evolve_for_grasping.py`
    - `evolve_for_reaching.py`
    - `evolve_svm_world.py`
    - `nn_svm_world.py`
    - `py_evolve_for_reaching.py`
- **src**:
    - **graph_evolve**:
        - `chromosome_smach.py`: Handles conversion of chromosomes for state machines.
        - `graph_genome.py`: Handles the graph genome representation.
        - `smach_explore.py`
        - `smach_explore_grasp.py`
        - `smach_grasp.py`
        - `smach_stack.py`
        - `smach_svm_grasp.py`
        - `__init__.py`: Initialization script.
    - **networkx**: NetworkX related scripts for graph manipulation.
        - `convert.py`
        - `exception.py`
        - `relabel.py`
        - `release.py`
        - `utils.py`
        - Other utility scripts for graph algorithms.

## Contributing Guidelines

We welcome contributions to the project! Here’s how you can help:

1. **Fork the repository:**

    ```bash
    git fork https://github.com/Yuriy/Graph-Evolve.git
    ```

2. **Create a feature branch:**

    ```bash
    git checkout -b new-feature
    ```

3. **Commit your changes:**

    ```bash
    git commit -m "Add some feature"
    ```

4. **Push the branch:**

    ```bash
    git push origin new-feature
    ```

5. **Submit a pull request** detailing your changes.

## License

Graph-Evolve is licensed under the [BSD License](LICENSE).
# Graph-Evolve

## Overview
Graph-Evolve is a comprehensive library for analyzing, evolving, and manipulating different types of graphs using various algorithms. This project builds upon the NetworkX library to provide advanced functionalities such as computing minimum spanning trees, calculating centrality measures, and assessing graph components and their characteristics.

## Features
- **Minimum Spanning Tree (MST) Algorithms**: Compute MSTs using Kruskal's and Prim’s algorithms.
- **Neighbor Degree**: Calculate the average neighbor degree, including in-degree and out-degree for directed graphs.
- **Graph Operations**: Perform union, intersection, difference, and complement operations on graphs.
- **Rich-Club Coefficient**: Calculate the rich-club coefficient to assess the 'richness' of nodes.
- **S-Metric**: Compute the s-metric of a graph, a structural property.
- **Vitality Measures**: Calculate measures like closeness vitality.
- **Bipartite Graph Algorithms**: Functions to handle bipartite graphs including centrality, clustering, projection, and redundancy.

## Installation Instructions
To install Graph-Evolve, follow these steps:

1. Clone the repository:
    ```sh
    git clone https://github.com/Yuriy/Graph-Evolve.git
    ```

2. Change into the project directory:
    ```sh
    cd Graph-Evolve
    ```

3. Install the required packages (preferably within a virtual environment):
    ```sh
    pip install -r requirements.txt
    ```

## Usage Examples
Graph-Evolve provides a variety of algorithms and utility functions. Here are some examples of how to use a few key features.

1. **Calculating the Minimum Spanning Tree**:
    ```python
    import networkx as nx
    from networkx.algorithms.mst import minimum_spanning_tree

    G = nx.Graph()
    G.add_weighted_edges_from([(0, 1, 2), (1, 2, 3), (0, 2, 1)])
    T = minimum_spanning_tree(G)
    print(T.edges(data=True))
    ```

2. **Neighbor Degree**:
    ```python
    import networkx as nx
    from networkx.algorithms.neighbor_degree import average_neighbor_degree

    G = nx.Graph()
    G.add_edges_from([(0, 1), (1, 2), (2, 3)])
    avg_deg = average_neighbor_degree(G)
    print(avg_deg)
    ```

3. **Rich-Club Coefficient**:
    ```python
    import networkx as nx
    from networkx.algorithms.richclub import rich_club_coefficient

    G = nx.Graph()
    G.add_edges_from([(0, 1), (1, 2), (2, 3), (3, 4)])
    rc = rich_club_coefficient(G)
    print(rc)
    ```

4. **Bipartite Graph Projection**:
    ```python
    import networkx as nx
    from networkx.algorithms.bipartite import projected_graph

    B = nx.Graph()
    B.add_edges_from([(1, 'a'), (1, 'b'), (2, 'a'), (2, 'c')])
    G = projected_graph(B, [1, 2])
    print(G.edges())
    ```

## Code Summary
The project structure is organized as follows:

- **algorithms/**: Contains various submodules for different graph algorithms:
  - `mst.py`: Functions for minimum spanning tree algorithms.
  - `neighbor_degree.py`: Functions to calculate average neighbor degree.
  - `operators.py`: Graph operations like union, intersection.
  - `richclub.py`: Functions to calculate the rich-club coefficient.
  - `smetric.py`: Functions for s-metric calculation.
  - `vitality.py`: Functions to compute vitality measures.
- **algorithms/bipartite/**: Specialized algorithms for bipartite graphs:
  - `basic.py`: Basic bipartite graph functions.
  - `centrality.py`: Centrality measures for bipartite graphs.
  - `cluster.py`: Clustering coefficients for bipartite graphs.
  - `projection.py`: Create projections from bipartite graphs.
  - `redundancy.py`: Compute node redundancy in bipartite graphs.
  - `spectral.py`: Spectral measures for bipartite graphs.

## Contributing Guidelines
We welcome contributions from the community. To contribute, please follow these steps:
1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes.
4. Commit your changes (`git commit -am 'Add new feature'`).
5. Push to the branch (`git push origin feature-branch`).
6. Create a new Pull Request.

## License
This project is licensed under the BSD License. See the LICENSE file for more details.

```markdown
# Graph-Evolve

## Overview
Graph-Evolve is a Python library designed to analyze and manipulate complex networks. The library offers a rich set of algorithms for graph analysis, including components, flow, isomorphism, link analysis, and shortest paths. It is built on top of NetworkX and extends its functionality, providing efficient and scalable solutions for large-scale network data.

## Features
- Compute various types of connected components (weakly, strongly, etc.)
- Maximum flow and minimum cut algorithms
- Graph isomorphism and subgraph matching using VF2 algorithm
- PageRank and HITS link analysis
- Shortest path algorithms including A*, Floyd-Warshall, and Dijkstra's algorithm
- Extensive test suite for validation and reliability

## Installation Instructions
To install Graph-Evolve, follow these steps:

1. **Clone the repository:**
   ```bash
   git clone https://github.com/<your-github-username>/Graph-Evolve.git
   cd Graph-Evolve
   ```

2. **Create a virtual environment:**
   ```bash
   python -m venv venv
   source venv/bin/activate   # On Windows, use `venv\Scripts\activate`
   ```

3. **Install the dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up the library:**
   ```bash
   python setup.py install
   ```

## Usage Examples
Here are some examples of how to use Graph-Evolve in your project:

### Finding Connected Components
```python
import networkx as nx
from networkx.algorithms.components import connected_components

G = nx.erdos_renyi_graph(10, 0.5)
components = list(connected_components(G))
print(components)
```

### Maximum Flow Algorithm
```python
import networkx as nx
from networkx.algorithms.flow import ford_fulkerson

G = nx.DiGraph()
G.add_edge('A', 'B', capacity=4)
G.add_edge('A', 'C', capacity=2)
G.add_edge('B', 'D', capacity=3)
G.add_edge('C', 'D', capacity=4)
flow_value, flow_dict = ford_fulkerson(G, 'A', 'D')
print(flow_value, flow_dict)
```

### Graph Isomorphism
```python
import networkx as nx
from networkx.algorithms.isomorphism import GraphMatcher

G1 = nx.Graph([(1, 2), (1, 3), (2, 3), (3, 4)])
G2 = nx.Graph([(10, 20), (10, 30), (20, 30), (30, 40)])
GM = GraphMatcher(G1, G2)
print(GM.is_isomorphic())
```

### Calculating PageRank
```python
import networkx as nx
from networkx.algorithms.link_analysis.pagerank_alg import pagerank

G = nx.DiGraph() 
G.add_edges_from([(1, 2), (1, 3), (2, 3), (3, 1)])
pr = pagerank(G, alpha=0.85)
print(pr)
```

### Shortest Path using Dijkstra's Algorithm
```python
import networkx as nx
from networkx.algorithms.shortest_paths.weighted import dijkstra_path

G = nx.DiGraph()
G.add_weighted_edges_from([(1, 2, 4), (1, 3, 2), (2, 3, 5), (3, 4, 1)])
path = dijkstra_path(G, source=1, target=4)
print(path)
```

## Code Summary
The project is structured into several main modules:

1. **Components:**
   - `connected.py`, `strongly_connected.py`, `weakly_connected.py`: Handle different types of connected components.
   - `tests/test_connected.py`, `tests/test_strongly_connected.py`, `tests/test_weakly_connected.py`: Unit tests for the components algorithms.

2. **Flow:**
   - `maxflow.py`, `mincost.py`: Implement maximum flow and minimum cost flow algorithms.
   - `tests/test_maxflow.py`, `tests/test_mincost.py`: Unit tests for flow algorithms.

3. **Isomorphism:**
   - `isomorph.py`, `isomorphvf2.py`, `vf2weighted.py`: Provide graph isomorphism checking using the VF2 algorithm.
   - `tests/test_isomorphism.py`, `tests/test_isomorphvf2.py`: Unit tests for isomorphism algorithms.

4. **Link Analysis:**
   - `hits_alg.py`, `pagerank_alg.py`: Implement HITS and PageRank algorithms for link analysis.
   - `tests/test_hits.py`, `tests/test_pagerank.py`: Unit tests for link analysis algorithms.

5. **Shortest Paths:**
   - `astar.py`, `dense.py`, `generic.py`, `unweighted.py`, `weighted.py`: Various shortest path algorithms.
   - `tests/test_astar.py`, `tests/test_dense.py`, `tests/test_generic.py`, `tests/test_unweighted.py`, `tests/test_weighted.py`: Unit tests for shortest path algorithms.

6. **Additional Utility and Testing:**
   - All algorithms and data structures have utility functions and extensive testing to ensure reliability and performance.

## Contributing Guidelines
We welcome contributions from the community! To contribute:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Commit your changes with clear and concise commit messages.
4. Push your branch to your forked repository.
5. Open a pull request to the main repository.

Please ensure that your code follows the existing style and includes appropriate tests.

## License
Graph-Evolve is licensed under the BSD License. Please see the `LICENSE` file for more details.
```

This README.md file should give potential users and contributors a comprehensive understanding of the Graph-Evolve project, its capabilities, and how to get started with it.
```
```markdown
# Graph-Evolve

## Overview
Graph-Evolve is a comprehensive set of algorithms and utilities for handling, analyzing, and visualizing graphs using NetworkX. The project includes tools for graph traversal, matching, mixing, layout, and more, with a focus on testing various graph properties and functionalities.

## Features
- **Graph Traversal**: Implements both breadth-first search (BFS) and depth-first search (DFS) algorithms.
- **Distance Regularity Testing**: Tests for distance-regular graphs.
- **Eulerian Path and Circuit**: Determines whether a graph is Eulerian and finds Eulerian circuits.
- **Graph Matching**: Maximum weight matching for given graphs.
- **Mixing Attributes**: Algorithms for mixing node attributes.
- **Minimum Spanning Tree (MST)**: Finds MST for given graphs.
- **Neighbor Degree**: Calculates average neighbor degrees.
- **Graph Union**: Combines multiple graphs with attribute management.
- **Rich-club Coefficient**: Computes rich-club coefficients for graphs.
- **S-Metric**: Computes s-metric for given graphs.
- **Closeness Vitality**: Computes closeness vitality for nodes in a graph.
- **Graph Layouts**: Provides various node positioning algorithms for drawing graphs.
- **Graph Visualization**: Support for drawing graphs using Matplotlib and interfacing with Graphviz through Pygraphviz and Pydot.

## Installation Instructions
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Yuriy/Graph-Evolve.git
   ```

2. **Navigate to the Project Directory**:
   ```bash
   cd Graph-Evolve
   ```

3. **Install Dependencies**:
   Ensure you have `pip` installed, then run:
   ```bash
   pip install -r requirements.txt
   ```

4. **Run Tests** (Optional):
   To ensure everything is set up correctly, you may run the included tests.
   ```bash
   nosetests -v
   ```

## Usage Examples
Here are a few usage examples to illustrate the functionalities provided by Graph-Evolve:

- **Breadth-First Search (BFS)**:
  ```python
  import networkx as nx
  from networkx.algorithms.traversal.breadth_first_search import bfs_edges

  G = nx.path_graph(5)
  edges = bfs_edges(G, source=0)
  print(list(edges))
  ```

- **Depth-First Search (DFS)**:
  ```python
  import networkx as nx
  from networkx.algorithms.traversal.depth_first_search import dfs_edges

  G = nx.path_graph(5)
  edges = dfs_edges(G, source=0)
  print(list(edges))
  ```

- **Eulerian Circuit**:
  ```python
  import networkx as nx
  from networkx.algorithms.euler import eulerian_circuit, is_eulerian

  G = nx.complete_graph(5)
  if is_eulerian(G):
      circuit = eulerian_circuit(G)
      print(list(circuit))
  ```

## Code Summary
The project structure is as follows:

- `src/networkx/algorithms/tests/`: Contains various test scripts for algorithms like distance regularity, Eulerian paths, matching, mixing attributes, MSTs, neighbor degree, rich-club coefficient, and more.
- `src/networkx/algorithms/traversal/`: Implements basic graph traversal algorithms such as BFS and DFS.
- `src/networkx/classes/`: Defines different graph classes including directed graphs, undirected graphs, multigraphs, and multidigraphs.
- `src/networkx/drawing/`: Provides functionalities for graph visualization using Matplotlib and Graphviz.
- `src/networkx/generators/`: Contains generators for creating classic and bipartite graphs.
- `tests/`: Unit tests for different functionalities ensuring the robustness of the implementation.

## Contributing Guidelines
We welcome contributions to improve Graph-Evolve. Here are some guidelines to get started:

1. **Fork the Repository**: Use the GitHub interface to fork the repository to your own account.
2. **Create a Feature Branch**: Create a new branch corresponding to the feature or bug fix.
   ```bash
   git checkout -b feature/my-new-feature
   ```
3. **Make Your Changes**: Make your changes to the code, and ensure that they are well-documented.
4. **Add Tests**: Add tests to cover the new feature or bug fix.
5. **Run Tests**: Run the existing tests to make sure your changes do not break anything.
   ```bash
   nosetests -v
   ```
6. **Commit Your Changes**: Commit your changes with a descriptive commit message.
   ```bash
   git commit -am 'Add some feature'
   ```
7. **Push to the Branch**:
   ```bash
   git push origin feature/my-new-feature
   ```
8. **Create a Pull Request**: Open a pull request from your feature branch to the main repository.

## License
Graph-Evolve is licensed under the BSD License. See the LICENSE file for more details.

---

This README provides a comprehensive overview of the Graph-Evolve project, guiding users through installation, usage, code structure, contributing, and licensing.
```
# Graph-Evolve

## Overview
Graph-Evolve is a comprehensive library designed for generating and manipulating various types of graphs. The project leverages the power of the `networkx` library to provide robust and efficient methods for creating graphs with designated properties, including degree sequences, directed graphs, ego graphs, geometric graphs, and more.

## Features
- Generate graphs based on given degree sequences or expected sequences.
- Create specialized directed graphs, such as growing networks or scale-free networks.
- Generate ego graphs centered on a specific node.
- Produce random geometric graphs and intersection graphs.
- Construct hybrid graphs with various connectivity properties.
- Generate famous social networks, such as the Karate Club graph.
- Create stochastic graphs and threshold graphs.
- Handle line graphs, small and named graphs, among others.
- Read and write graphs in various formats, including adjacency lists, edge lists, GEXF, GML, and more.
  
## Installation Instructions
To install and set up the Graph-Evolve project, follow these steps:

1. Clone the repository:
    ```bash
    git clone https://github.com/username/Graph-Evolve.git
    cd Graph-Evolve
    ```

2. Set up a virtual environment (optional but recommended):
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows use `venv\Scripts\activate`
    ```

3. Install the necessary dependencies:
    ```bash
    pip install -r requirements.txt
    ```

## Usage Examples
Here's how to use the Graph-Evolve library to generate different types of graphs:

### Generate a Random Geometric Graph
```python
import networkx as nx
G = nx.random_geometric_graph(50, 0.25)
nx.draw(G, with_labels=True)
```

### Create a Scale-Free Directed Graph
```python
import networkx as nx
from networkx.generators.directed import scale_free_graph

G = scale_free_graph(100)
nx.draw(G, with_labels=False)
```

### Generate an Ego Graph
```python
import networkx as nx
from networkx.generators.ego import ego_graph

G = nx.star_graph(3)
H = ego_graph(G, 0)
nx.draw(H, with_labels=True)
```

### Create Zachary's Karate Club Graph
```python
import networkx as nx
from networkx.generators.social import karate_club_graph

G = karate_club_graph()
nx.draw(G, with_labels=True)
```

## Code Summary
The project is organized with various generators and utility functions in the following structure:

- `degree_seq.py`: Functions to generate graphs based on degree sequences.
- `directed.py`: Functions to create specialized directed graphs.
- `ego.py`: Functions to generate ego graphs.
- `geometric.py`: Functions to generate geometric graphs.
- `hybrid.py`: Functions to create hybrid graphs.
- `line.py`: Functions to generate line graphs.
- `random_graphs.py`: Functions to create random graphs.
- `small.py`: Functions for small and named graphs.
- `social.py`: Functions to generate famous social network graphs.
- `stochastic.py`: Functions to create stochastic graphs.
- `threshold.py`: Functions for threshold graphs.

The `tests` directory contains unit tests for each type of graph generator to ensure their correct functionality.

## Contributing Guidelines
We welcome contributions to the Graph-Evolve project! Here are the steps to contribute:

1. Fork the repository.
2. Create a new branch with a descriptive name: `git checkout -b feature-name`.
3. Make your changes and add tests to cover new functionalities.
4. Commit your changes: `git commit -m 'Add new feature'`.
5. Push to your branch: `git push origin feature-name`.
6. Open a pull request and provide a detailed description of your changes.

Please ensure that your code adheres to PEP 8 standards and passes all existing tests.

## License
This project is licensed under the BSD License. Redistributions and use in source and binary forms, with or without modification, are permitted provided that the conditions of the BSD License are met. See the `LICENSE` file for more details.

---

Thank you for using Graph-Evolve! We hope this library helps you effortlessly generate and manipulate various graph structures for your projects. If you encounter any issues or have suggestions for improvements, feel free to open an issue or submit a pull request. Happy coding!
```markdown
# Graph-Evolve

## Overview
Graph-Evolve is a comprehensive Python project focused on reading and writing graphs in various formats using the NetworkX library. This project supports a multitude of graph formats, providing tools for graph manipulation, serialization, and deserialization. It also includes machine learning simulators and state machine functionalities.

## Features
- **Multi-line Adjacency List**: Read and write NetworkX graphs as multi-line adjacency lists.
- **Shapefile**: Generate NetworkX directed graphs from point and line shapefiles.
- **YAML Support**: Read and write NetworkX graphs in YAML format.
- **P2G Format**: Support for the P2G format used in metabolic pathway studies.
- **Pajek Format**: Read graphs in Pajek format, supporting both directed and undirected graphs.
- **SparseGraph6**: Read graphs in graph6 and sparse6 formats, which are compact representations of undirected graphs.
- **Unit Testing**: Comprehensive unit tests for various read and write operations.
- **Simulators**: Machine learning simulators for grasping world scenarios.
- **State Machine (SMACH)**: Tools for creating state machines with concurrency, iterator, and sequence functionalities.

## Installation Instructions
1. **Clone the Repository**:
    ```bash
    git clone https://github.com/Yuriy/Graph-Evolve.git
    cd Graph-Evolve/src
    ```

2. **Set up a Virtual Environment** (Optional but recommended):
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install Dependencies**:
   Install dependencies using `pip`:
   ```bash
   pip install -r requirements.txt
   ```

4. **Run Tests**:
   Ensure that everything is set up correctly by running the tests:
   ```bash
   cd networkx/tests
   nosetests
   ```

## Usage Examples
### Reading and Writing Multi-line Adjacency Lists
```python
import networkx as nx
from networkx.readwrite.multiline_adjlist import read_multiline_adjlist, write_multiline_adjlist

# Create a sample graph
G = nx.Graph()
G.add_edges_from([(1, 2), (2, 3)])

# Write the graph to a file
write_multiline_adjlist(G, "graph.adjlist")

# Read the graph from a file
G_read = read_multiline_adjlist("graph.adjlist")
print(G_read.edges())
```

### Generating a DiGraph from Shapefile
```python
import networkx as nx
from networkx.readwrite.nx_shp import read_shp

# Read the shapefile
G = read_shp("path/to/shapefile.shp")

# Work with the generated directed graph
print(G.edges(data=True))
```

### Working with YAML Formats
```python
import networkx as nx
from networkx.readwrite.nx_yaml import write_yaml

G = nx.Graph()
G.add_edges_from([(1, 2), (2, 3)])

# Write the graph to a YAML file
write_yaml(G, "graph.yaml")

# Read the graph from a YAML file
G_read = nx.read_yaml("graph.yaml")
print(G_read.edges())
```

## Code Summary
### Structure
The project is organized into several modules, each focusing on different functionalities:
- **networkx**: Contains modules for reading and writing graphs in various formats:
  - `multiline_adjlist.py`: Multi-line adjacency list format.
  - `nx_shp.py`: Shapefile format.
  - `nx_yaml.py`: YAML format.
  - `p2g.py`: P2G format.
  - `pajek.py`: Pajek format.
  - `sparsegraph6.py`: Sparsegraph6 format.
- **simulators**: Contains machine learning simulators for grasping world and SVM models.
  - `grasping_world.py`: Defines objects and their manipulations in a grasping world scenario.
  - `multi_svm.py`: MultiSVR model training and prediction.
  - `svm_grasping_world.py`: SVM integration with grasping world objects.
- **smach**: State machine functionalities.
  - `concurrence.py`, `container.py`, `exceptions.py`, `iterator.py`, `log.py`, `sequence.py`, `state.py`: Various tools to create and manage states and transitions.

### Key Files
- `__init__.py`: Initialization of the modules.
- `tests/`: Contains unit tests for the project to ensure correctness and stability.

## Contributing Guidelines
We welcome contributions! To contribute:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/new-feature`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature/new-feature`).
5. Create a new Pull Request.

Please ensure your code follows the existing code style and include appropriate tests.

## License
This project is licensed under the BSD License. See the [LICENSE](LICENSE) file for more details.
```

Feel free to modify each section according to specific needs or additional information you wish to provide.
# Graph-Evolve

## Overview
Graph-Evolve is a project designed to implement and evaluate evolutionary algorithms applied to graph-based structures. It uses the Pyevolve framework and the SMACH library to create, manage, and evolve graphs that represent various processes or systems.

## Features
- **Finite State Machine (FSM)**: Implementation of hierarchical finite state machines using SMACH.
- **Evolutionary Algorithms**: Utilization of Pyevolve for evolving graph-based structures.
- **User Data Management**: Handling and updating user data within SMACH.
- **Graph Generation and Evaluation**: Functions for generating and evaluating graphs using evolutionary algorithms.
- **Plotting Tools**: Scripts for visualizing the performance and properties of evolved graphs.
- **MPI Support**: Tools for parallel computation using MPI.

## Installation Instructions

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Yuriy/Graph-Evolve.git
   cd Graph-Evolve
   ```

2. **Install Dependencies**:
   - Ensure you have Python installed.
   - Install required Python packages:
     ```bash
     pip install -r requirements.txt
     ```

3. **Setup Environment**:
   - Add the `src` and other directories to your Python path if necessary:
     ```python
     import sys
     sys.path.append('/path/to/Graph-Evolve/src')
     sys.path.append('/path/to/Graph-Evolve/src/pyevolve')
     ```

## Usage Examples

### Example 1: Running a Simple Evolutionary Algorithm
```python
from pyevolve import GSimpleGA, G1DList, Selectors, Initializators, Mutators
from graph_evolve import graph_genome

def eval_func(chromosome):
    return len(chromosome)

genome = G1DList.G1DList(10)
genome.evaluator.set(eval_func)
genome.mutator.set(Mutators.G1DListMutatorSwap)

ga = GSimpleGA.GSimpleGA(genome)
ga.selector.set(Selectors.GRouletteWheel)
ga.evolve(freq_stats=10)

print(ga.bestIndividual())
```

## Code Summary

### Key Files and Their Descriptions:

- **`src/smach/state_machine.py`**:
  - Implements a finite state machine container using SMACH.
  
- **`src/smach/user_data.py`**:
  - Defines the `UserData` class for handling state machine user data.
  
- **`src/smach/util.py`**:
  - Utility functions and classes for SMACH interfaces and shutdown checks.
  
- **`tests/evolutionary_length.py`**:
  - Script to test evolutionary algorithm focusing on chromosome length.
  
- **`tests/evolutionary_study.py`**:
  - Script to evaluate different aspects of evolutionary algorithms.
  
- **`tools/*`**:
  - Various tools for evaluating and visualizing evolutionary processes and data.

## Contributing Guidelines

1. **Fork the Repository**:
   - Create your own fork of the repository on GitHub.
   
2. **Clone Your Fork**:
   - Clone the forked repository to your local machine:
     ```bash
     git clone https://github.com/yourusername/Graph-Evolve.git
     cd Graph-Evolve
     ```

3. **Create a New Branch**:
   - Create and switch to a new branch for your feature or bugfix:
     ```bash
     git checkout -b feature-or-bugfix-name
     ```

4. **Make and Commit Changes**:
   - Make your changes, add them to the staging area, and commit with a descriptive message:
     ```bash
     git add .
     git commit -m "Description of changes"
     ```

5. **Push to GitHub and Create a Pull Request**:
   - Push your branch to your forked repository:
     ```bash
     git push origin feature-or-bugfix-name
     ```
   - Open a pull request on the original repository.
   
## License

This project is licensed under the [BSD License](LICENSE). The detailed terms and conditions are provided in the `LICENSE` file included in the repository.

---

Feel free to reach out with any questions or issues you encounter. Happy evolving!