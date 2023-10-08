#include <set>
#include <algorithm>

#include "search-strategies.h"
#include "memusage.h"

bool operator==(const SearchState &a, const SearchState &b) {
    return !(a < b) && !(b < a);
}

inline bool memUsageSucceeded(size_t limit) {
	return getCurrentRSS() > limit * 0.98; // Leave 2% as a reserve
}

class PathItem {
public:
	SearchState parent;
	SearchAction action;

	PathItem(const SearchState &p, const SearchAction& a) : parent(p), action(a) {}
};

std::vector<SearchAction> getPath(std::map<SearchState, PathItem> &paths, const SearchState &final, const SearchState &init) {
	std::vector<SearchAction> path;
	auto current = paths.find(final);

	while (!(current->second.parent == init)) {
		path.push_back(current->second.action);
		current = paths.find(current->second.parent);
	}

	// Add first action
	path.push_back(current->second.action);

	std::reverse(path.begin(), path.end());
	return path;
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

class AStarFrontierItem {
public:
	SearchState state;
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

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {	
	std::set<AStarFrontierItem> frontier;
	std::set<SearchState> closed;
	std::map<SearchState, PathItem> paths;

	// Add initial state
	frontier.insert(AStarFrontierItem(init_state, 0, compute_heuristic(init_state, *heuristic_)));

	while (!frontier.empty()) {
		AStarFrontierItem current = *frontier.begin();
		frontier.erase(frontier.begin());

		closed.insert(current.state);

		if (current.state.isFinal())
			return getPath(paths, current.state, init_state);

		for (SearchAction &action : current.state.actions()) {
			// Stop if memory usage is too high
			if (memUsageSucceeded(mem_limit_)) {
				std::cout << "Memory limit exceeded, returning empty path\n";
				return {};
			}

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
				paths.insert_or_assign(new_state, PathItem(current.state, action));
			}
		}
	}

	return {};
}