#include "AStar.h"
#include <set>
#include <list>
#include <limits>

/*
GraphSearchResult is a struct with bool success if a path was found, 
std::list<amp::Node> node_path for the sequence of nodes in the path.
Note that amp::Node is just a uint32_t
`node_path.front()` must contain init node, and `node_path.back()` must contain the goal node
double path cost = sum of edge weights along node_path

ShortestPathProblem comes from Graph.h. struct with
shared_ptr to a Graph<double>
amp::Node init_node
amp::Node goal_node

SearchHeuristic also comes from Graph.h, struct with
virtual double operator()(amp::Node node) const that must be overriden

have to implement:
virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) 
SearchHeuristic::operator()
*/

struct AStarNode {

    amp::Node prevNode;
    double pathLength;
    double heuristic;
    bool closed;
    bool open;
    double priority;

    AStarNode() {
        prevNode = -1;
        pathLength = 0;
        heuristic = 0;
        closed = false;
        open = false;
        priority = pathLength + heuristic;
    }

    AStarNode(amp::Node b, double c, double d) {
        prevNode = b;
        pathLength = c;
        heuristic = d;
        closed = false;
        open = false;
        priority = pathLength + heuristic;
    }
};

amp::Node findBest(std::map<amp::Node, AStarNode>& allNodes, const std::set<amp::Node>& open) {
    double min = std::numeric_limits<double>::max();
    amp::Node best = -1;
    for (amp::Node i : open) {
        if (allNodes[i].priority < min) {
            min = allNodes[i].priority;
            best = i;
        }
    }

    return best;
}

MyAStar::GraphSearchResult MyAStar::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::set<amp::Node> open;
    std::map<amp::Node, AStarNode> allNodes;
    open.emplace(problem.init_node);

    for(amp::Node i : problem.graph->nodes()) {
        AStarNode temp(0, 0.0, heuristic(i));
        allNodes[i] = temp;
    }
    allNodes[problem.init_node].open = true;

    int ktr = 0;
    bool goalFound = false;
    //DEBUG("# of nodes: " << problem.graph->nodes().size());
    while (!open.empty()) {
        ktr++;
        //if (ktr % 100 == 0) DEBUG("Iteration: " << ktr);
        // Pick nbest from O such that f (nbest) ≤ f (n), ∀n ∈ O
        //for (amp::Node i : open) {
            //DEBUG(i <<": " << allNodes[i].priority);
        //}
        amp::Node nBest = findBest(allNodes, open);
        //DEBUG("nBest = " << nBest);
        // Remove nbest from O and add to C.
        open.erase(nBest);
        allNodes[nBest].open = false;

        if (!allNodes[nBest].closed) {
            //DEBUG("Inserting " << nBest << " to closed");
            allNodes[nBest].closed = true;
        }
        // If nbest = qgoal , EXIT.
        if (nBest == problem.goal_node){
            goalFound = true;
            break;
        }
        // Expand nbest : for all x ∈ Star(nbest) that are not in C.
        std::vector<amp::Node> children = problem.graph->children(nBest);
        std::vector<double> edges = problem.graph->outgoingEdges(nBest);
        //for (AStarNode i : closed) {
            //DEBUG(i.node <<": " << i.prevNode);
        //}
        for (int i = 0; i < children.size(); i++) {
            //DEBUG("Checking child " << children[i]);
            
            if (allNodes[children[i]].closed) {
                //DEBUG("Already closed");
                continue;
            }

            // if x ∈/ O then add x to O.
            if (!allNodes[children[i]].open) {
                //DEBUG("Adding to open");
                allNodes[children[i]].prevNode = nBest;
                allNodes[children[i]].open = true;
                allNodes[children[i]].pathLength = allNodes[nBest].pathLength + edges[i];
                allNodes[children[i]].priority = allNodes[children[i]].pathLength + allNodes[children[i]].heuristic;
                open.emplace(children[i]);
            } else {
                // else if g(nbest) + c(nbest , x) < g(x) then update x’s backpointer to point to nbest
                if ((allNodes[nBest].pathLength + edges[i]) < allNodes[children[i]].pathLength) {
                    //DEBUG("Resetting prevNode");
                    allNodes[children[i]].prevNode = nBest;
                    allNodes[children[i]].pathLength = allNodes[nBest].pathLength + edges[i];
                    allNodes[children[i]].priority = allNodes[children[i]].pathLength + allNodes[children[i]].heuristic;
                }
            }
        }

    }
    //for (AStarNode i : closed) {
        //DEBUG(i.node <<": " << i.prevNode);
    //}
    MyAStar::GraphSearchResult result;
    if (goalFound) {
        result.success = true;
        result.path_cost = allNodes[problem.goal_node].pathLength;
        result.node_path.insert(result.node_path.begin(), problem.goal_node);
        //DEBUG("Inserted " << problem.goal_node);
        ktr = 0;
        amp::Node temp = problem.goal_node;
        while (temp != problem.init_node) {
            ktr++;
            temp = allNodes[temp].prevNode;
            result.node_path.insert(result.node_path.begin(), temp);
            //DEBUG("inserted " << temp);
            if (ktr > 10000) {
                break;
            }
        }
        
    } else {
        result.success = false;
    }

    return result;
    
    //MyAStar::GraphSearchResult result;
    //result.success = false;
    //return result;
}


/*
Thoughts:

remove node from the struct above, add bool closed as the way to track which nodes are closed

use a std::map to store all the nodes ... initialize by adding all of them 
with filled out heuristic value, closed = false, prevNode = -1, pathLength = 0

use priority queue with either new struct or pair, sorted on the length+heuristic
that also stores the node associated with that value
*/