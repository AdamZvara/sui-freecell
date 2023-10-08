#include <set>
#include <algorithm>

#include "search-strategies.h"

class PathMapItem {
public:
	const SearchState parent;
	const SearchAction action;

	PathMapItem(const SearchState &p, const SearchAction& a) : parent(p), action(a) {}
};

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

bool operator==(const SearchState &a, const SearchState &b) {
    return !(a < b) && !(b < a);
}

class AStarFrontierItem {
public:
	const SearchState state;
	int g_cost;
	double f_cost;
	
	AStarFrontierItem(const SearchState &state, int g_cost, double f_cost):
		state(state), g_cost(g_cost), f_cost(f_cost) {}

	// Constructor used for checking if state is in frontier
	AStarFrontierItem(const SearchState &state): state(state) {}

	bool operator<(const AStarFrontierItem &other) const {
		if (state == other.state) 
			return false;
		return f_cost < other.f_cost;
	}
};

std::vector<SearchAction> getPath(std::map<SearchState, PathMapItem> &paths, const SearchState &final, const SearchState &init) {
	std::vector<SearchAction> path;
	auto current = paths.find(final);

	while (!(current->first == init)) {
		path.push_back(current->second.action);
		current = paths.find(current->second.parent);
	}

	std::reverse(path.begin(), path.end());
	return path;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {	
	std::set<AStarFrontierItem> frontier;
	std::set<SearchState> closed;
	std::map<SearchState, PathMapItem> paths;

	// Add initial state
	frontier.insert(AStarFrontierItem(init_state, 0, compute_heuristic(init_state, *heuristic_)));
	paths.insert(std::make_pair(init_state, PathMapItem(init_state, init_state.actions()[0])));

	while (!frontier.empty()) {
		AStarFrontierItem current = *frontier.begin();
		frontier.erase(frontier.begin());

		closed.insert(current.state);

		if (current.state.isFinal())
			return getPath(paths, current.state, init_state);

		for (const SearchAction &action : current.state.actions()) {
			SearchState new_state = action.execute(current.state);

			// Don't add to frontier if already in closed
			if (closed.find(new_state) != closed.end())
				continue;

			auto stored_state = frontier.find(AStarFrontierItem(new_state));
			double f_updated = current.g_cost + 1 + compute_heuristic(new_state, *heuristic_);

			if (stored_state == frontier.end() || stored_state->f_cost > f_updated) {
				if (stored_state != frontier.end())
					frontier.erase(stored_state); // Only if item was already in map
				frontier.insert(AStarFrontierItem(new_state, current.g_cost + 1, f_updated));
				paths.insert(std::make_pair(new_state, PathMapItem(current.state, action)));
			}
		}
	}

	return {};
}