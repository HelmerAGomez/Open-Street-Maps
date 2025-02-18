#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue>  // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "json.hpp"

#include "dist.h"
#include "graph.h"

using namespace std;

double INF = numeric_limits<double>::max();

class prioritize {
   public:
    bool operator()(const pair<long long, double>& p1,
                    const pair<long long, double>& p2) const {
        return p1.second > p2.second;
    }
};

void buildGraph(istream& input, graph<long long, double>& g,
                vector<BuildingInfo>& buildings) {
  // TODO_STUDENT
	map<long long, Coordinates> waypointInfo;
	nlohmann::json j;
	input >> j;
	if (j.contains("buildings")) {
		BuildingInfo tempBuild;
    for (const auto& building : j["buildings"]) {
      tempBuild.id = building["id"];
			Coordinates setCords(building["lat"],building["lon"]);
			tempBuild.location = setCords;
			tempBuild.abbr = building["abbr"];
      tempBuild.name = building["name"];
			buildings.push_back(tempBuild);
			g.addVertex(building["id"]);
    } 
	}
	if (j.contains("waypoints")) {
    for (const auto& waypoints : j["waypoints"]) {
			Coordinates setCords(waypoints["lat"],waypoints["lon"]);
			waypointInfo.emplace(waypoints["id"], setCords);
			g.addVertex(waypoints["id"]);
    } 
	}
	vector<long long> path;
	if (j.contains("footways")) {
    for (const auto& footway : j["footways"]) { 
			path.clear();
      for (const auto& node : footway) {
        path.push_back(node); 
      }
			for(size_t i = 1; i < path.size();i++) {
				Coordinates firstCoords = waypointInfo.at(path.at(i-1));
				Coordinates secondCoords = waypointInfo.at(path.at(i));
				double edgeWeight = distBetween2Points(firstCoords, secondCoords);
				g.addEdge(path.at(i-1), path.at(i), edgeWeight);
				g.addEdge(path.at(i), path.at(i-1),edgeWeight);
			}
    }
  }
	for(size_t i = 0; i < buildings.size(); i++) {
		for (auto j = waypointInfo.begin(); j != waypointInfo.end(); j++) {
			double distanceFromBuild = distBetween2Points(buildings.at(i).location, j->second);
			if(distanceFromBuild <= 0.036){
				g.addEdge(buildings.at(i).id, j->first, distanceFromBuild);
				g.addEdge(j->first, buildings.at(i).id, distanceFromBuild);
			}
		}
	}
}

BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) {
  for (const BuildingInfo& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target,
                           const set<long long>& ignoreNodes) {

	unordered_map<long long, double> distance;
	unordered_map<long long, long long> predecessor;
	vector<long long> shortestPath;
	priority_queue<pair<long long, double>,
               vector<pair<long long, double>>,
               prioritize>
    worklist;
		bool reachedEnd = false;
	if(start == target) {
		shortestPath.push_back(start);
		return shortestPath;
	}
	for(auto &vertex: G.getVertices()) {
		distance.emplace(vertex, INF);
	}
	worklist.push({start,0});
	distance[start] = 0;
	predecessor[start] = -1;
	while(!(worklist.empty()) && !reachedEnd) {
		auto currVertex = worklist.top();
		worklist.pop();
		if(currVertex.first == target) {
			reachedEnd = true;
		}
		else {
			for(auto &neighbor: G.neighbors(currVertex.first)) {
				if(neighbor == target  || ignoreNodes.find(neighbor) == ignoreNodes.end()) {
					double edgeWeight;
					G.getWeight(currVertex.first, neighbor,edgeWeight);
					double alternativePath = edgeWeight + distance[currVertex.first];
					if(alternativePath < distance[neighbor]) {
						distance[neighbor] = alternativePath;
						predecessor[neighbor] = currVertex.first;
						worklist.push({neighbor, distance[neighbor]});
					}
				}
			}
		}
	}

	if(distance[target] == INF) {
  	return vector<long long>{};
	}
	long long currVertex = target;
  while (currVertex != -1) {
    shortestPath.insert(shortestPath.begin(), currVertex);
    currVertex = predecessor[currVertex];
  }
	return shortestPath;
}

double pathLength(const graph<long long, double>& G,
                  const vector<long long>& path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

void application(const vector<BuildingInfo>& buildings,
                 const graph<long long, double>& G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto& building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
