#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  // TODO_STUDENT
	unordered_map<VertexT,unordered_map<VertexT, WeightT>> adjList;
	size_t total;

 public:
  /// Default constructor
  graph() {
    // TODO_STUDENT
		total = 0;
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    // TODO_STUDENT
		unordered_map<VertexT, WeightT> empty;
    auto success = adjList.emplace(v, empty);
		return success.second;
		
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    // TODO_STUDENT
		if(adjList.find(from) == adjList.end() || adjList.find(to) == adjList.end()) {
			return false;
		}
		if((adjList.find(from)->second.emplace(to,weight).second)) {
			total++;
		}
		adjList[from][to] = weight;
    return true;
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    // TODO_STUDENT
		if(adjList.at(from).find(to) == adjList.at(from).end()) {
			return false;
		}
		weight = (adjList.at(from).find(to)->second);
    return true;
  }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    // TODO_STUDENT
		for (const auto& element : adjList.at(v)) {
        S.emplace(element.first);
    }
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    // TODO_STUDENT
		vector<VertexT> allV;
		for(const auto& elements: adjList) {
			allV.push_back(elements.first);
		}
    return allV;
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    // TODO_STUDENT
    return adjList.size();
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    // TODO_STUDENT
    return total;
  }
};
