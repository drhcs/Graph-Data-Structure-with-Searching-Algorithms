#ifndef GRAPH
#define GRAPH

#include <unordered_map>
#include <list>
#include <queue>
#include <unordered_set>

template <class Key, class Val, class E>
class Graph {

        std::unordered_map<Key, Val> vertices_;
        std::unordered_map<Key, std::unordered_map<Key, E>> adjacencies_;

        void DFSTraversalRecurse(Key v, std::unordered_set<Key> & visited, std::list<Key> & traversal) {

            traversal.push_back(v);

            for (const auto & entry : adjacencies_[v])
            {
                Key neighbor = entry.first;
                if (visited.find(neighbor) == visited.end())
                {
                    visited.insert(neighbor);
                    DFSTraversalRecurse(neighbor, visited, traversal);
                }
            }
        }

        void DFSRecurse(std::list<Key> curr_path, Key u, std::unordered_set<Key> visited, std::list<std::list<Key>> & paths) {

            Key curr_vertex = curr_path.back();

            if (curr_vertex == u)
            {
                paths.push_back(curr_path);
                return;
            }

            for (const auto & entry : adjacencies_[curr_vertex])
            {
                Key neighbor = entry.first;
                if (visited.find(neighbor) == visited.end())
                {
                    visited.insert(neighbor);
                    curr_path.push_back(neighbor);
                    DFSRecurse(curr_path, u, visited, paths);
                    curr_path.pop_back();
                }
            }
        }

        std::list<Key> TracePathBackwards(Key v, Key u, std::unordered_map<Key, Key> prev) {
            std::list<Key> path;
            Key curr_vertex = u;
            while (curr_vertex != v)
            {
                path.push_front(curr_vertex);
                curr_vertex = prev[curr_vertex];
            }
            path.push_front(curr_vertex);
            return path;
        }

public:

        Graph() = default;

        ~Graph() = default;

        std::unordered_map<Key, Val> GetVertices() {
            return vertices_;
        }

        void SetVertices(std::unordered_map<Key, Val> vertices) {
            vertices_ = vertices;
        }

        std::unordered_map<Key, std::unordered_map<Key, E>> GetAdjacencies() {
            return adjacencies_;
        }

        void SetAdjacencies(std::unordered_map<Key, std::unordered_map<Key, E>> adjacencies) {
            adjacencies_ = adjacencies;
        }

        void AddVertex(Key k, Val v) {
            vertices_[k] = v;
        }

        Val GetVertex(Key k) {
            return vertices_[k];
        }

        Val RemoveVertex(Key k) {
            Val removed = vertices_[k];
            vertices_.erase(k);
            adjacencies_.erase(k);
            for (auto & entry : adjacencies_)
            {
                std::unordered_map<Key, E> & neighbors = entry.second;
                neighbors.erase(k);
            }
            return removed;
        }

        void AddDirectedEdge(Key v, Key u, E weight) {
            adjacencies_[v][u] = weight;
        }

        void AddUndirectedEdge(Key v, Key u, E weight) {
            adjacencies_[v][u] = weight;
            adjacencies_[u][v] = weight;
        }

        E GetEdge(Key v, Key u) {
            if (adjacencies_.find(v) != adjacencies_.end() || adjacencies_[v].find(u) != adjacencies_[v].end()) return 0;
            return adjacencies_[v][u];
        }

        E RemoveEdge(Key v, Key u) {
            E removed = adjacencies_[v][u];
            adjacencies_[v].erase(u);
            adjacencies_[u].erase(v);
            return removed;
        }

        void AddNeighbors(Key v, std::unordered_map<Key, E> neighbors) {
            adjacencies_[v] = neighbors;
        }

        std::unordered_map<Key, E> GetNeighbors(Key v) {
            return adjacencies_[v];
        }

        std::unordered_map<Key, E> RemoveNeighbors(Key v) {
            std::unordered_map<Key, E> removed = adjacencies_[v];
            adjacencies_.erase(v);
            for (auto & entry : adjacencies_)
            {
                std::unordered_map<Key, E> & neighbors = entry.second;
                neighbors.erase(v);
            }
            return removed;
        }

        std::list<Key> BFSTraversal(Key v) {
            std::list<Key> traversal;
            std::unordered_set<Key> visited;
            std::queue<Key> vertices;
            visited.insert(v);
            vertices.push(v);

            while (!vertices.empty())
            {
                Key curr_vertex = vertices.front();
                vertices.pop();
                traversal.push_back(curr_vertex);

                for (const auto & entry : adjacencies_[curr_vertex])
                {
                    Key neighbor = entry.first;
                    if (visited.find(neighbor) == visited.end())
                    {
                        visited.insert(neighbor);
                        vertices.push(neighbor);
                    }
                }
            }

            return traversal;
        }

        std::list<Key> DFSTraversal(Key v) {
            std::unordered_set<Key> visited;
            std::list<Key> traversal;
            visited.insert(v);
            DFSTraversalRecurse(v, visited, traversal);
            return traversal;
        }

        std::list<Key> BFS(Key v, Key u) {
            std::unordered_set<Key> visited;
            std::queue<std::list<Key>> paths;
            visited.insert(v);
            paths.push({v});

            while (!paths.empty())
            {
                std::list<Key> curr_path = paths.front();
                Key curr_vertex = curr_path.back();
                paths.pop();

                if (curr_vertex == u)
                {
                    return curr_path;
                }

                for (const auto & entry : adjacencies_[curr_vertex])
                {
                    Key neighbor = entry.first;
                    if (visited.find(neighbor) == visited.end())
                    {
                        visited.insert(neighbor);
                        curr_path.push_back(neighbor);
                        paths.push(curr_path);
                        curr_path.pop_back();
                    }
                }
            }

            return {};
        }

        std::list<std::list<Key>> DFS(Key v, Key u) {
            std::unordered_set<Key> visited;
            std::list<std::list<Key>> paths;
            DFSRecurse({v}, u, visited, paths);
            return paths;
        }

        std::list<std::list<Key>> GetConnectedComponents() {
            std::unordered_set<Key> visited;
            std::list<std::list<Key>> components;
            for (const auto & entry : vertices_)
            {
                Key curr_vertex = entry.first;
                if (visited.find(curr_vertex) == visited.end())
                {
                    visited.insert(curr_vertex);
                    std::list<Key> curr_component;
                    DFSTraversalRecurse(curr_vertex, visited, curr_component);
                    components.push_back(curr_component);
                }
            }
            return components;
        }

        std::list<Key> Dijkstras(Key v, Key u) {
            auto CompareWeights = [](std::pair<Key, E> & a, std::pair<Key, E> & b){ return a.second > b.second;};
            std::priority_queue<std::pair<Key, E>, std::vector<std::pair<Key, E>>, decltype(CompareWeights)> q(CompareWeights);
            std::unordered_map<Key, E> distances;
            std::unordered_map<Key, Key> prev;

            distances[v] = 0;
            q.push({v, distances[v]});

            for (const auto & entry : vertices_)
            {
                Key curr_vertex = entry.first;
                if (curr_vertex != v)
                {
                    distances[curr_vertex] = std::numeric_limits<E>::max();
                }
            }

            while (!q.empty())
            {
                auto curr_vertex = q.top();
                Key curr_vertex_key = curr_vertex.first;
                q.pop();

                if (curr_vertex_key == u)
                {
                    return TracePathBackwards(v, u, prev);
                }

                for (const auto & entry : adjacencies_[curr_vertex_key])
                {
                    Key neighbor = entry.first;
                    E distance_to_neighbor = entry.second;
                    E new_distance = distances[curr_vertex_key] + distance_to_neighbor;
                    if (new_distance < distances[neighbor])
                    {
                        distances[neighbor] = new_distance;
                        q.push({neighbor, new_distance});
                        prev[neighbor] = curr_vertex_key;
                    }
                }
            }

            return {};
        }

        template<typename F>
        std::list<Key> AStar(Key v, Key u, F && heuristic) {
            auto CompareWeights = [](std::pair<Key, E> & a, std::pair<Key, E> & b){ return a.second > b.second;};
            std::priority_queue<std::pair<Key, E>, std::vector<std::pair<Key, E>>, decltype(CompareWeights)> q(CompareWeights);
            std::unordered_map<Key, E> distances;
            std::unordered_map<Key, Key> prev;

            distances[v] = 0;
            q.push({v, distances[v]});

            for (const auto & entry : vertices_)
            {
                Key curr_vertex = entry.first;
                if (curr_vertex != v)
                {
                    distances[curr_vertex] = std::numeric_limits<E>::max();
                }
            }

            while (!q.empty())
            {
                auto curr_vertex = q.top();
                Key curr_vertex_key = curr_vertex.first;
                q.pop();

                if (curr_vertex_key == u)
                {
                    return TracePathBackwards(v, u, prev);
                }

                for (const auto & entry : adjacencies_[curr_vertex_key])
                {
                    Key neighbor = entry.first;
                    E distance_to_neighbor = entry.second;
                    E new_distance = distances[curr_vertex_key] + distance_to_neighbor;
                    if (new_distance < distances[neighbor])
                    {
                        distances[neighbor] = new_distance;
                        q.push({neighbor, new_distance + heuristic(neighbor)});
                        prev[neighbor] = curr_vertex_key;
                    }
                }
            }

            return {};
        }

        Graph<Key, Val, E> Prims() {
            auto CompareWeights = [](std::tuple<Key, Key, E> & a, std::tuple<Key, Key, E> & b){ return std::get<2>(a) > std::get<2>(b);};
            std::priority_queue<std::tuple<Key, Key, E>, std::vector<std::tuple<Key, Key, E>>, decltype(CompareWeights)> q(CompareWeights);
            std::unordered_set<Key> visited;
            Graph<Key, Val, E> min_spanning_tree;
            min_spanning_tree.SetVertices(vertices_);

            Key arbitrary_vertex = vertices_.begin()->first;
            visited.insert(arbitrary_vertex);
            for (const auto & entry : adjacencies_[arbitrary_vertex])
            {
                Key neighbor = entry.first;
                Key weight = entry.second;
                q.push({arbitrary_vertex, neighbor, weight});
            }

            while (visited.size() != vertices_.size())
            {
                auto min_edge = q.top();
                Key vertex_from = std::get<0>(min_edge);
                Key vertex_to = std::get<1>(min_edge);
                E weight = std::get<2>(min_edge);
                q.pop();

                visited.insert(vertex_to);
                min_spanning_tree.AddUndirectedEdge(vertex_from, vertex_to, weight);

                for (const auto & entry : adjacencies_[vertex_to])
                {
                    Key neighbor = entry.first;
                    Key weight = entry.second;
                    if (visited.find(neighbor) == visited.end())
                    {
                        q.push({vertex_to, neighbor, weight});
                    }
                }
            }

            return min_spanning_tree;
        }
};

#endif
