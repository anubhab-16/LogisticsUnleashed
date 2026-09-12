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

vector<vector<int>> dist;
vector<vector<int>> next_node;

unordered_map<int, unordered_map<int, int>> edge_map;
unordered_set<int> fuel_station_set;

// Floyd-Warshall

void floyd_warshall(const vector<vector<pair<int, int>>>& graph) {

    dist.assign(T, vector<int>(T, INF));
    next_node.assign(T, vector<int>(T, -1));

    for (int i = 0; i < T; ++i) {
        dist[i][i] = 0;
        next_node[i][i] = i;
    }

    // Direct edges
    for (int u = 0; u < T; ++u) {

        for (auto edge : graph[u]) {

            int v = edge.first;
            int cost = edge.second;

            if (cost < dist[u][v]) {
                dist[u][v] = cost;
                next_node[u][v] = v;
            }
        }
    }

    // Floyd-Warshall
    for (int k = 0; k < T; ++k) {

        for (int i = 0; i < T; ++i) {

            for (int j = 0; j < T; ++j) {

                if (dist[i][k] == INF ||
                    dist[k][j] == INF)
                    continue;

                if (dist[i][j] >
                    dist[i][k] + dist[k][j]) {

                    dist[i][j] =
                        dist[i][k] + dist[k][j];

                    next_node[i][j] =
                        next_node[i][k];
                }
            }
        }
    }
}



// Reconstruct shortest path

vector<int> get_path(int u, int v) {

    vector<int> path;

    if (u < 0 || v < 0 ||
        u >= T || v >= T)
        return path;

    if (next_node[u][v] == -1)
        return path;

    path.push_back(u);

    while (u != v) {

        u = next_node[u][v];

        if (u == -1)
            return {};

        path.push_back(u);
    }

    return path;
}



// Travel through a shortest path
// Does NOT modify original route/fuel until successful

bool try_append_path(
    vector<int>& route,
    int from,
    int to,
    int& fuel,
    int& added_distance
) {

    vector<int> path = get_path(from, to);

    if (path.empty())
        return false;

    vector<int> temp_route = route;
    int temp_fuel = fuel;
    int temp_distance = 0;

    for (int i = 1; i < path.size(); ++i) {

        int u = path[i - 1];
        int v = path[i];

        if (edge_map[u].count(v) == 0)
            return false;

        int cost = edge_map[u][v];

        if (cost > temp_fuel)
            return false;

        temp_fuel -= cost;
        temp_distance += cost;

        temp_route.push_back(v);

        // Automatically refuel at fuel station
        if (fuel_station_set.count(v)) {
            temp_fuel = F;
        }
    }

    // Commit only after complete path succeeds
    route = temp_route;
    fuel = temp_fuel;
    added_distance = temp_distance;

    return true;
}



// Find a reachable fuel station

int find_best_reachable_station(
    int current,
    int fuel,
    vector<int>& route,
    int& station_distance
) {

    int best_station = -1;
    int best_distance = INF;

    vector<int> best_route;
    int best_fuel = fuel;

    for (int station : fuel_stations) {

        if (station == current)
            continue;

        vector<int> temp_route = route;
        int temp_fuel = fuel;
        int temp_distance = 0;

        if (!try_append_path(
                temp_route,
                current,
                station,
                temp_fuel,
                temp_distance))
            continue;

        if (temp_distance < best_distance) {

            best_distance = temp_distance;
            best_station = station;
            best_route = temp_route;
            best_fuel = temp_fuel;
        }
    }

    if (best_station != -1) {

        route = best_route;
        fuel = best_fuel;
        station_distance = best_distance;
    }

    return best_station;
}



// Travel from current location to destination
// using fuel stations if required

bool travel_with_fuel_support(
    vector<int>& route,
    int& current,
    int destination,
    int& fuel,
    int& total_distance
) {

    while (current != destination) {

        // Try directly reaching destination
        vector<int> temp_route = route;
        int temp_fuel = fuel;
        int direct_distance = 0;

        if (try_append_path(
                temp_route,
                current,
                destination,
                temp_fuel,
                direct_distance)) {

            route = temp_route;
            fuel = temp_fuel;

            total_distance += direct_distance;
            current = destination;

            return true;
        }

        // Direct route failed.
        // Find a reachable fuel station.
        int station_distance = 0;

        int station = find_best_reachable_station(
            current,
            fuel,
            route,
            station_distance
        );

        if (station == -1)
            return false;

        total_distance += station_distance;

        current = station;

        // Refuel
        fuel = F;
    }

    return true;
}



// Check whether a location is already used


bool contains(
    const vector<int>& locations,
    int value
) {

    for (int x : locations) {

        if (x == value)
            return true;
    }

    return false;
}

// best route

vector<int> best_route;
int best_distance = INF;


void find_best_route(
    int current,
    vector<int>& remaining,
    vector<int>& route,
    int fuel,
    int current_distance
) {

    // If every required location has been visited
    if (remaining.empty()) {

        if (current_distance < best_distance) {

            best_distance = current_distance;
            best_route = route;
        }

        return;
    }


    // Try every possible next location
    for (int i = 0; i < remaining.size(); ++i) {

        int destination = remaining[i];

        // Pruning
        if (current_distance >= best_distance)
            continue;

        vector<int> temp_route = route;

        int temp_fuel = fuel;
        int temp_current = current;

        int added_distance = 0;

        // Try travelling to this destination
        if (!travel_with_fuel_support(
                temp_route,
                temp_current,
                destination,
                temp_fuel,
                added_distance))
            continue;

        int new_distance =
            current_distance + added_distance;

        if (new_distance >= best_distance)
            continue;


        // Remove selected destination
        vector<int> new_remaining;

        for (int j = 0;
             j < remaining.size();
             ++j) {

            if (j != i)
                new_remaining.push_back(
                    remaining[j]
                );
        }

        find_best_route(
            destination,
            new_remaining,
            temp_route,
            temp_fuel,
            new_distance
        );
    }
}


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

        graph[u].push_back(
            {v, c}
        );

        graph[v].push_back(
            {u, c}
        );

        edge_map[u][v] = c;
        edge_map[v][u] = c;
    }


    // Calculate all-pairs shortest paths

    floyd_warshall(graph);


    
    // Select starting hub

    int start_hub = -1;
    int maximum_distance = -1;

    for (int hub : hubs) {

        int nearest_station = INF;

        for (int station : fuel_stations) {

            if (dist[hub][station] <
                nearest_station) {

                nearest_station =
                    dist[hub][station];
            }
        }

        if (nearest_station >
            maximum_distance) {

            maximum_distance =
                nearest_station;

            start_hub = hub;
        }
    }

    // Select final house

    int end_house = -1;
    maximum_distance = -1;

    for (int house : houses) {

        int nearest_station = INF;

        for (int station : fuel_stations) {

            if (dist[house][station] <
                nearest_station) {

                nearest_station =
                    dist[house][station];
            }
        }

        if (nearest_station >
            maximum_distance) {

            maximum_distance =
                nearest_station;

            end_house = house;
        }
    }


    // Create list of locations to visit

    vector<int> locations;

    // Add all hubs except starting hub

    for (int hub : hubs) {

        if (hub != start_hub)
            locations.push_back(hub);
    }


    // Add houses except final house

    for (int house : houses) {

        if (house != end_house)
            locations.push_back(house);
    }

    vector<int> route;

    route.push_back(start_hub);

    int initial_fuel = F;


    find_best_route(
        start_hub,
        locations,
        route,
        initial_fuel,
        0
    );


    if (best_route.empty()) {

        cout << "No feasible route found.\n";

        return 0;
    }


    int final_current =
        best_route.back();

    int final_fuel = F;

    int final_distance = best_distance;

    vector<int> final_route =
        best_route;


    // Recalculate fuel along best route
    // so we know the actual remaining fuel.

    final_fuel = F;

    for (int i = 1;
         i < final_route.size();
         ++i) {

        int u = final_route[i - 1];
        int v = final_route[i];

        int cost = edge_map[u][v];

        final_fuel -= cost;

        if (fuel_station_set.count(v))
            final_fuel = F;
    }


    int added_distance = 0;

    if (!travel_with_fuel_support(
            final_route,
            final_current,
            end_house,
            final_fuel,
            added_distance)) {

        cout << "Unable to reach final house due to fuel constraints.\n";

        return 0;
    }


    final_distance += added_distance;


    // --------------------------------------------------
    // Output
    // --------------------------------------------------

    cout << "Minimum route distance: "
         << final_distance << "\n";

    cout << "Number of nodes in route: "
         << final_route.size() << "\n";

    cout << "Route:\n";

    for (int node : final_route)
        cout << node << " ";

    cout << "\n";


    return 0;
}
