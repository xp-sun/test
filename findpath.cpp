/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <iostream>

#include <assert.h>
#include <set>
#include <stack>
#include <vector>

using namespace std;

// A directed graph using adjacency list representation
class Graph
{
public:
    int V;    // No. of vertices in graph
    vector<int> *adj; // Pointer to an array containing adjacency lists

public:
    Graph(int V);   // Constructor
    ~Graph();
    
    void addEdge(int u, int v);
    void printAllPaths(int s, int d);
};

Graph::Graph(int V)
{
    this->V = V;
    adj = new vector<int>[V];
}

Graph::~Graph()
{
    delete [] adj;
}

void Graph::addEdge(int u, int v)
{
    adj[u].push_back(v); // Add v to u~Rs list.
}

typedef std::vector<int> IntVec;
typedef std::vector<IntVec> IntVecVec;
class PathVisitor
{
  private:
    std::set<int> m_visited;
    IntVec m_path;           // store the current searching path
    IntVecVec m_paths;       // store succeeded paths
    std::set<int> m_failedNodes;  // finished, and no path found

    size_t m_level;
    const Graph& m_graph;

  public:
    PathVisitor(const Graph& graph) : m_graph(graph) { }

   const IntVecVec& allPaths(int s, int d)
    {
        init();

        assert(m_paths.empty());
        assert(m_path.empty());
        findAllPaths(s, d);
        assert(m_path.empty());
        return m_paths;
    }

    bool findAllPaths(int u, int d)
    {
        
        // pre-visit
        m_level++;
        markVisited(u);
        m_path.push_back(u);
        
        // visit
        bool found = false;
        if(u == d)
        {
            m_paths.push_back(m_path);
            found = true;
        }
        else
        {
            for(std::vector<int>::const_iterator it = m_graph.adj[u].begin();
                it != m_graph.adj[u].end(); ++it)
            {
                int next = *it;
                if (unvisited(next) &&
                    m_failedNodes.find(next) == m_failedNodes.end())
                {
                    if(findAllPaths(next, d))
                        found = true;
                }
            }

        }

    
        // post-visit
        m_level--;
        unmarkVisited(u);
        m_path.pop_back();
        
        // we have finished on "u", and know there is no path between u->d
        if(!found)
            m_failedNodes.insert(u);

        return found;
    }
    
    // non-recursive
    const IntVecVec& findAllPath2(int u, int d)
    {
        init();
        
        std::stack<int> nodeStack;
        std::vector<bool> statusStack;
        std::vector<size_t> indexStack;
        size_t index = 0;
        
        markVisited(u);
        m_path.push_back(u);
        nodeStack.push(u);
        statusStack.push_back(false);
        indexStack.push_back(index);
        int nextNode = u;
        
        while(!nodeStack.empty())
        {
            if(nextNode == d)
            {
                if(m_path.size() >= 2)
                    m_paths.push_back(m_path);

                unmarkVisited(nextNode);
                nodeStack.pop();
                m_path.pop_back();
                statusStack.pop_back();
                indexStack.pop_back();
                if(!statusStack.empty())
                    statusStack.back() = true;
                if(indexStack.empty())
                    break;

                ++indexStack.back();
                index = indexStack.back();
                nextNode = nodeStack.top();
            }

            int parent = nextNode;
            bool keepDiving = false;
            const vector<int>& nodes = m_graph.adj[parent];
            index = indexStack.back();
            for(; index < nodes.size(); ++index)
            {
                nextNode = nodes[index];
                if (unvisited(nextNode) &&
                        m_failedNodes.find(nextNode) == m_failedNodes.end())
                {
                    nodeStack.push(nextNode);
                    m_path.push_back(nextNode);
                    markVisited(nextNode);
                    statusStack.push_back(false);
                    indexStack.back() = index;
                    indexStack.push_back(0); // start from 0 for the next level
                    keepDiving = true;
                    break;
                }
                else
                    nextNode = parent;
            }

            if(keepDiving)
            {
                assert(index < nodes.size());
                continue;
            }

            // we are done with all parent's children
            bool pathFound = statusStack.back();
            unmarkVisited(nodeStack.top());
            nodeStack.pop();
            m_path.pop_back();
            statusStack.pop_back();
            indexStack.pop_back();
            if(!indexStack.empty())
            {
                ++indexStack.back();
                index = indexStack.back();
                nextNode = nodeStack.top();
            }
            if(pathFound)
            {
                if(!statusStack.empty())
                    statusStack.back() = true;
            }
            else
                 m_failedNodes.insert(parent);
        }
        return m_paths;
    }

    void markVisited(int node) { m_visited.insert(node); }
    void unmarkVisited(int node) { m_visited.erase(node); }
    bool unvisited(int node) const
    {   return m_visited.find(node) == m_visited.end(); }

    void init()
    {
        m_visited.clear();
        m_paths.clear();
        m_failedNodes.clear();
        assert(m_path.empty());
        m_level = 0;
    }
};  // class PathVisitor

// Prints all paths from 's' to 'd'
void Graph::printAllPaths(int s, int d)
{
    PathVisitor visitor(*this);
    //const IntVecVec& paths = visitor.allPaths(s, d);
    const IntVecVec& paths = visitor.findAllPath2(s, d);
    for(const auto& fullPath : paths)
    {
        for(auto node : fullPath)
            std::cout << node << " ";
        std::cout << std::endl;
    }
}

int main()
{
    // Create a graph given in the above diagram
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(0, 3);
    g.addEdge(2, 0);
    g.addEdge(2, 1);
    g.addEdge(1, 3);

    int s = 2, d = 3;
    cout << "Following are all different paths from " << s
         << " to " << d << endl;
    g.printAllPaths(s, d);

    return 0;
}



