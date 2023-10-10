#include <set>
#include <algorithm>

#include "search-strategies.h"
#include "memusage.h"

#define RESERVE 50000 // 50 MB memory reserve

bool operator==(const SearchState &a, const SearchState &b) {
    return !(a < b) && !(b < a);
}

inline bool memUsageSucceeded(size_t limit) {
	return getCurrentRSS() > limit - RESERVE;
}

class PathItem {
public:
	std::shared_ptr<SearchState> parent;
	SearchAction action;

	PathItem(std::shared_ptr<SearchState> p, const SearchAction& a) : parent(move(p)), action(a) {}
};

std::vector<SearchAction> getPath(
	std::map<std::shared_ptr<SearchState>, PathItem> &paths, 
	std::shared_ptr<SearchState> final, 
	std::shared_ptr<SearchState> init
) {
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
	std::shared_ptr<SearchState> state;
	int g_cost;
	double f_cost;
	
	AStarFrontierItem(std::shared_ptr<SearchState> state, int g_cost, double f_cost):
		state(move(state)), g_cost(g_cost), f_cost(f_cost) {}

	// Constructor used for checking if state is in open
	AStarFrontierItem(std::shared_ptr<SearchState> state): state(move(state)), g_cost(0), f_cost(0) {}

	bool operator<(const AStarFrontierItem &other) const {
		// std::cout << "AStarFrontierItem comparison" << std::endl;
		if (*state.get() == *other.state.get()) 
			return false;
		return f_cost < other.f_cost;
	}
};

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {	
	std::multiset<AStarFrontierItem> open;
	std::set<std::shared_ptr<SearchState>> closed;
	std::map<std::shared_ptr<SearchState>, PathItem> paths;

	auto init_state_ptr = std::make_shared<SearchState>(init_state);

	// Add initial state
	open.insert(AStarFrontierItem(init_state_ptr, 0, compute_heuristic(init_state, *heuristic_)));

	while (!open.empty()) {
		AStarFrontierItem current = *open.begin();
		open.erase(open.begin());
		closed.insert(current.state);

		auto current_state = current.state.get();
		auto new_g_cost = current.g_cost + 1;

		if (current_state->isFinal())
			return getPath(paths, current.state, init_state_ptr);

		for (SearchAction &action : current_state->actions()) {
			// Stop if memory usage is too high
			if (memUsageSucceeded(mem_limit_)) {
				std::cout << "Memory limit exceeded (" << getCurrentRSS();
				std::cout << " out of " << mem_limit_ << "), returning empty path" << std::endl;
				return {};
			}

			std::shared_ptr<SearchState> new_state = std::make_shared<SearchState>(action.execute(*current_state));

			// Don't add to open if already in closed
			if (closed.find(new_state) != closed.end())
				continue;

			auto item = AStarFrontierItem(new_state);
			auto stored_state = open.find(item);
			double f_updated = new_g_cost + compute_heuristic(*new_state, *heuristic_);

			if (stored_state == open.end() || stored_state->f_cost > f_updated) {
				item.f_cost = f_updated;
				item.g_cost = new_g_cost;
				if (stored_state != open.end())
					open.erase(stored_state); // Only if item was already in open
				open.insert(item);
				paths.insert_or_assign(new_state, PathItem(current.state, action));
			}
		}
	}

	return {};
}