#include <iostream>
#include <vector>
#include <climits>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <algorithm>

using namespace std;

const int INF = 1e9;

int N, T, M, K, F;

vector<int> hubs, houses, fuel_stations;
vector<vector<int>> dist, next_node;

unordered_map<int, unordered_map<int, int>> edge_map;
unordered_set<int> fuel_station_set;


// Floyd-Warshall Algorithm

void floyd_warshall(const vector<vector<pair<int, int>>>& graph) {

    dist.assign(T, vector<int>(T, INF));
    next_node.assign(T, vector<int>(T, -1));

    // Distance from a node to itself
    for (int u = 0; u < T; ++u) {
        dist[u][u] = 0;
        next_node[u][u] = u;
    }

    // Direct edges
    for (int u = 0; u < T; ++u) {
        for (int i = 0; i < graph[u].size(); ++i) {

            int v = graph[u][i].first;
            int cost = graph[u][i].second;

            dist[u][v] = cost;
            next_node[u][v] = v;
        }
    }

    // Floyd-Warshall
    for (int k = 0; k < T; ++k) {

        for (int i = 0; i < T; ++i) {

            for (int j = 0; j < T; ++j) {

                if (dist[i][k] < INF &&
                    dist[k][j] < INF &&
                    dist[i][j] > dist[i][k] + dist[k][j]) {

                    dist[i][j] = dist[i][k] + dist[k][j];

                    // Store first node on the shortest path
                    next_node[i][j] = next_node[i][k];
                }
            }
        }
    }
}

// Reconstruct shortest path

vector<int> get_path(int u, int v) {

    vector<int> path;

    if (next_node[u][v] == -1)
        return path;

    path.push_back(u);

    while (u != v) {

        u = next_node[u][v];

        path.push_back(u);
    }

    return path;
}


// Find the point farthest from its nearest fuel station

int farthest_from_fuel(const vector<int>& points) {

    int max_dist = -1;
    int selected = -1;

    for (int p : points) {

        int min_to_station = INF;

        for (int s : fuel_stations) {

            if (dist[p][s] < min_to_station)
                min_to_station = dist[p][s];
        }

        if (min_to_station > max_dist) {

            max_dist = min_to_station;
            selected = p;
        }
    }

    return selected;
}



// Travel along a shortest path while checking fuel

bool append_path_with_fuel(
    vector<int>& route,
    int from,
    int to,
    int& fuel
) {

    vector<int> path = get_path(from, to);

    if (path.empty())
        return false;

    for (int i = 1; i < path.size(); ++i) {

        int u = path[i - 1];
        int v = path[i];

        if (edge_map[u].count(v) == 0)
            return false;

        int cost = edge_map[u][v];

        // Not enough fuel for this road
        if (cost > fuel)
            return false;

        fuel -= cost;

        route.push_back(v);

        // Refuel automatically
        if (fuel_station_set.count(v))
            fuel = F;
    }

    return true;
}



// Find a reachable fuel station

int find_reachable_fuel_station(
    int current,
    int fuel
) {

    int best_station = -1;
    int best_distance = INF;

    for (int station : fuel_stations) {

        // Already at this station
        if (station == current) {
            return station;
        }

        // Check if the shortest distance can be reached
        // with the currently available fuel
        if (dist[current][station] <= fuel) {

            if (dist[current][station] < best_distance) {

                best_distance = dist[current][station];
                best_station = station;
            }
        }
    }

    return best_station;
}


// --------------------------------------------------
// Travel to a destination using fuel stations if needed
// --------------------------------------------------
bool travel_with_fuel_support(
    vector<int>& route,
    int& current,
    int destination,
    int& fuel
) {

    while (current != destination) {

        // First try to directly reach the destination
        if (append_path_with_fuel(
                route,
                current,
                destination,
                fuel)) {

            current = destination;
            return true;
        }

        // If direct travel is not possible,
        // find a reachable fuel station
        int station = find_reachable_fuel_station(
            current,
            fuel
        );

        // No fuel station can be reached
        if (station == -1)
            return false;

        // Avoid infinite loop
        if (station == current)
            return false;

        // Travel to the fuel station
        if (!append_path_with_fuel(
                route,
                current,
                station,
                fuel)) {

            return false;
        }

        current = station;

        // Refuel
        fuel = F;
    }

    return true;
}



// Main

int main() {

    cin >> N >> T >> M >> K >> F;

    hubs.resize(N);
    houses.resize(N);
    fuel_stations.resize(K);


    
    // Input hubs
    
    for (int i = 0; i < N; ++i)
        cin >> hubs[i];


    
    // Input houses
    
    for (int i = 0; i < N; ++i)
        cin >> houses[i];


   
    // Input fuel stations
    
    for (int i = 0; i < K; ++i) {

        cin >> fuel_stations[i];

        fuel_station_set.insert(
            fuel_stations[i]
        );
    }


    // Build graph
    vector<vector<pair<int, int>>> graph(T);

    for (int i = 0; i < M; ++i) {

        int u, v, c;

        cin >> u >> v >> c;

        // Undirected graph
        graph[u].push_back({v, c});
        graph[v].push_back({u, c});

        // Direct edge lookup
        edge_map[u][v] = c;
        edge_map[v][u] = c;
    }


    // -----------------------------
    // Calculate all-pairs shortest paths
    // -----------------------------
    floyd_warshall(graph);

    // Select starting hub
   
    int start_hub = farthest_from_fuel(hubs);


   
    // Select final house
  
    int end_house = farthest_from_fuel(houses);


    unordered_set<int> visited_hubs;
    unordered_set<int> visited_houses;


    vector<int> route;

    int current = start_hub;

    int fuel = F;

    route.push_back(current);

    visited_hubs.insert(current);


    // STEP 1: Visit all hubs using greedy approach
  
    while (visited_hubs.size() < hubs.size()) {

        int next_hub = -1;
        int best_distance = INF;


        // Find nearest unvisited hub
        for (int hub : hubs) {

            if (visited_hubs.count(hub))
                continue;

            if (dist[current][hub] < best_distance) {

                best_distance = dist[current][hub];

                next_hub = hub;
            }
        }


        if (next_hub == -1)
            break;


        // Travel toward the selected hub.
        // If fuel is insufficient, the function
        // will use a reachable fuel station.
        if (!travel_with_fuel_support(
                route,
                current,
                next_hub,
                fuel)) {

            cout << "Unable to reach next hub due to fuel constraints.\n";

            return 0;
        }


        visited_hubs.insert(next_hub);
    }


    
    // STEP 2: Visit houses

    for (int house : houses) {

        // Keep the selected final house for last
        if (house == end_house)
            continue;


        if (!travel_with_fuel_support(
                route,
                current,
                house,
                fuel)) {

            cout << "Unable to reach house due to fuel constraints.\n";

            return 0;
        }


        visited_houses.insert(house);
    }


   
    // STEP 3: Visit final house
  
    if (!travel_with_fuel_support(
            route,
            current,
            end_house,
            fuel)) {

        cout << "Unable to reach final house due to fuel constraints.\n";

        return 0;
    }

    visited_houses.insert(end_house);
    
    // Output route
    
    cout << route.size() << "\n";

    for (int node : route)
        cout << node << " ";

    cout << "\n";


    return 0;
}
