#include <set>
#include <algorithm>
#include <queue>
#include <stack>

#include "card-storage.h"
#include "search-strategies.h"
#include "memusage.h"

#include <malloc.h>
#include <utility>

#define RESERVE 50000 // 50 MB memory reserve

bool operator==(const SearchState &a, const SearchState &b) {
    return a.state_ == b.state_;
}

bool searchStatePtrLesser(const std::shared_ptr<SearchState> a, const std::shared_ptr<SearchState> b) {
    return *a < *b;
}

inline bool memUsageSucceeded(size_t limit) {
	return getCurrentRSS() > limit - RESERVE;
}

class PathItem {
public:
	std::shared_ptr<SearchState> parent;
	SearchAction action;

	PathItem(std::shared_ptr<SearchState> p, const SearchAction& a) : parent(std::move(p)), action(a) {}
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
    malloc_trim(0);
	std::map<std::shared_ptr<SearchState>, PathItem> paths;
    std::queue<std::shared_ptr<SearchState>> open;
    std::set<std::shared_ptr<SearchState>, decltype(searchStatePtrLesser)*> explored(searchStatePtrLesser);

    std::shared_ptr<SearchState> initStatePtr = std::make_shared<SearchState>(init_state);
    open.push(initStatePtr);

    if (init_state.isFinal()) {
        // found final state
        return {};
    }

    while (!open.empty()) {
        std::shared_ptr<SearchState> currState = open.front();
        open.pop();
        // skip duplicates in OPEN
        if (explored.find(currState) != explored.end()) {
            continue;
        }

        explored.insert(currState);

        std::vector<SearchAction> availActions = currState->actions();
        for (SearchAction action : availActions) {
			if (memUsageSucceeded(mem_limit_)) {
				std::cout << "Memory limit exceeded (" << getCurrentRSS();
				std::cout << " out of " << mem_limit_ << "), returning empty path" << std::endl;
				return {};
			}

            std::shared_ptr<SearchState> newState = std::make_shared<SearchState>(action.execute(*currState));

            if (newState->isFinal()) {
                // found final state
                std::cout << "Found Final State" << std::endl;
                // return path
                paths.insert_or_assign(newState, PathItem(currState, action));
                return getPath(paths, newState, initStatePtr);
            }

            if (explored.find(newState) == explored.end()) {
                open.push(newState);
                paths.insert_or_assign(newState, PathItem(currState, action));
            }
        }
    }

	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
    malloc_trim(0);
    std::map<std::shared_ptr<SearchState>, PathItem> paths;
    std::stack<std::pair<std::shared_ptr<SearchState>, int>> open;
    std::set<std::shared_ptr<SearchState>, decltype(searchStatePtrLesser)*> openSet(searchStatePtrLesser);

    int currDepth = 0;
    std::shared_ptr<SearchState> initStatePtr = std::make_shared<SearchState>(init_state);
    open.push(std::make_pair(initStatePtr, currDepth));

    if (init_state.isFinal()) {
        // found final state
        return {};
    }

    while (!open.empty()) {
        std::pair<std::shared_ptr<SearchState>, int> currStatePair = open.top();
        std::shared_ptr<SearchState> currState = currStatePair.first;
        currDepth = currStatePair.second;
        open.pop();

        if (currDepth >= depth_limit_) {
            continue;
        }

        std::vector<SearchAction> availActions = currState->actions();

        for (SearchAction action : availActions) {
			if (memUsageSucceeded(mem_limit_)) {
				std::cout << "Memory limit exceeded (" << getCurrentRSS();
				std::cout << " out of " << mem_limit_ << "), returning empty path" << std::endl;
				return {};
			}

            std::shared_ptr<SearchState> newState = std::make_shared<SearchState>(action.execute(*currState));
            if (newState->isFinal()) {
                // found final state
                std::cout << "Found Final State\n";
                // return path
                paths.insert_or_assign(newState, PathItem(currState, action));
                return getPath(paths, newState, initStatePtr);
            }

            if (openSet.find(newState) == openSet.end()) {
                open.push(std::make_pair(newState, currDepth + 1));
                openSet.insert(newState);
                paths.insert_or_assign(newState, PathItem(currStatePair.first, action));
            }
        }
    }

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
		state(std::move(state)), g_cost(g_cost), f_cost(f_cost) {}

	// Constructor used for checking if state is in open
	AStarFrontierItem(std::shared_ptr<SearchState> state): state(std::move(state)), g_cost(0), f_cost(0) {}

	bool operator<(const AStarFrontierItem &other) const {
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
				std::cout << "Memory limit exceeded" << std::endl;
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
					open.erase(stored_state);
				open.insert(stored_state, item);
				paths.insert_or_assign(new_state, PathItem(current.state, action));
			}
		}
	}

	return {};
}
