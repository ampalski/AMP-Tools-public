#include "PRM.h"
#include <map>
#include <list> 
#include <iterator>

double PrmHeuristic::operator()(amp::Node node) const {
    return (q_goal - nodes.at(node)).norm();
}

amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    /*
    1) Initialize graph with nodes q_init and q_goal
    2) until _numNodes, randomly samplea  new point, adding it to the vertex map if not in obstacle
    3) connect: For each pair of nodes, if distance < _connectRadius and collision free, add edge to edge set
    4) Perform A* on the resulting graph
    5) Perform check that q_init actually connected to q_goal
    */
    
    // Init
    amp::Node curNode = 0;
    _samples.clear();


    _samples[curNode++] = problem.q_init;
    _samples[curNode++] = problem.q_goal;

    // Sample
    while (curNode < _numNodes) {
        _samples[curNode++] = Utils::generateCollisionFreeSample(problem);
    }

    // Connect: 
    std::shared_ptr<amp::Graph<double>> graph = std::make_shared<amp::Graph<double>>();
    double dist;
    for (amp::Node i = 0; i < curNode; i++) {
        for (amp::Node j = i + 1; j < curNode; j++) {
            dist = (_samples[j]-_samples[i]).norm();
            if (dist < _connectRadius) {
                if (Utils::checkStep(_samples[i], _samples[j], problem)) {
                    continue;
                }
                //DEBUG("Connecting "<<i<<" and "<<j);
                graph->connect(i, j, dist);
                graph->connect(j, i, dist);
            }
        }
    }

    // Graph Search:
    amp::ShortestPathProblem graphProblem;
    graphProblem.graph = graph;
    graphProblem.init_node = 0;
    graphProblem.goal_node = 1;

    PrmHeuristic heuristic;
    heuristic.q_goal = problem.q_goal;
    heuristic.nodes = _samples;

    MyAStar algo;
    amp::AStar::GraphSearchResult result = algo.search(graphProblem, heuristic);

    if (!result.success) {
        amp::Path2D path;
        path.valid = false;
        //DEBUG("No path found");
        return path;
    }
    // Smoothing, if applicable
    if (_smoothing) {
        //p = 10% of total nodes, somewhat arbitrary
        for (int i = 0; i < (int)floor(_numNodes * .1); i++) {
            SmoothPath1(result, problem);
        }
    }

    // Convert back to Path2D
    amp::Path2D path;
    path.valid = true;
    for (amp::Node temp : result.node_path) {
        path.waypoints.push_back(_samples[temp]);
        //PRINT_VEC2("Node: ", path.waypoints.back());
    }

    _graph = *graph;
    return path;
}

amp::Graph<double> MyPRM::getGraph(){
    return _graph;
}
std::map<amp::Node, Eigen::Vector2d> MyPRM::getMap() {
    return _samples;
}

void MyPRM::SmoothPath1(amp::AStar::GraphSearchResult& path, const amp::Problem2D& problem) {
    // Define Range
    int n = path.node_path.size();
    if (n <= 2) return;

    //DEBUG("Attempting to smooth, n="<<n);
    int i = amp::RNG::randi(0, n);
    int j = amp::RNG::randi(0, n);
    //DEBUG("i = "<<i<<", j = "<<j);
    if (abs(i-j) <= 1) return;

    if (i > j) {
        int temp = j;
        j = i;
        i = temp;
    }

    std::list<amp::Node>::iterator i_it = path.node_path.begin();
    std::list<amp::Node>::iterator j_it = path.node_path.begin();
    std::advance(i_it, i);
    std::advance(j_it, j);

    // Collision checking
    if (Utils::checkStep(_samples[*i_it], _samples[*j_it], problem)) {
        return;
    }

    //Remove in-between nodes
    std::advance(i_it, 1);
    path.node_path.erase(i_it,j_it);
    //DEBUG("Smoothed, new n="<<path.node_path.size());
}