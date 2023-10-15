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

typedef std::shared_ptr<SearchState> SearchSharedPtr; 

/* Since this is the only operator that isn't defined and can access
 private members, it is used to compare states based on their stacks 
 (e.g. 2 states are considered equal, if their stacks are equal, so
 we don't take their freecells into consideration, because many expanded states
 only differ with card position in free cells or home cells ...) */
bool operator==(const SearchState &s1, const SearchState &s2) {
	return s1.state_.stacks < s2.state_.stacks;
}

bool searchStatePtrLesser(const std::shared_ptr<SearchState> a, const std::shared_ptr<SearchState> b) {
    return *a < *b;
}

inline bool memUsageSucceeded(size_t limit) {
	return getCurrentRSS() > limit - RESERVE;
}

class PathItem {
public:
	double f_cost;
	SearchSharedPtr parent;
	SearchAction action;

	PathItem(double f, SearchSharedPtr p, const SearchAction& a) : 
		f_cost(f), parent(std::move(p)), action(a) {}

	PathItem(SearchSharedPtr p, const SearchAction& a) : 
		f_cost(0), parent(std::move(p)), action(a) {}
};

// Since we use shared pointers to states, we need to define a custom comparator
struct cmpStateSharedPtr {
	bool operator()(const SearchSharedPtr &lhs, const SearchSharedPtr &rhs) const {
		return *lhs == *rhs;
	}
};

template <typename T>
std::vector<SearchAction> getPath(
	std::map<SearchSharedPtr, PathItem, T> &paths, 
	SearchSharedPtr final, 
	SearchSharedPtr init
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

class AStarOpenItem {
public:
	SearchSharedPtr state;
	int g_cost;
	double f_cost;
	
	AStarOpenItem(SearchSharedPtr state, int g_cost, double f_cost):
		state(std::move(state)), g_cost(g_cost), f_cost(f_cost) {}

	bool operator<(const AStarOpenItem &other) const {
		if (f_cost == other.f_cost) {
			return *state.get() == *other.state.get();
		}
		return f_cost < other.f_cost;
	}
};

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {	
	std::multiset<AStarOpenItem> open;
	std::set<SearchSharedPtr, cmpStateSharedPtr> closed;
	std::map<SearchSharedPtr, PathItem, cmpStateSharedPtr> paths;

	auto init_state_ptr = std::make_shared<SearchState>(init_state);

	// Add initial state
	open.insert(AStarOpenItem(init_state_ptr, 0, 0));
	// Initial state can have random action because it is not used
	paths.insert(std::make_pair(init_state_ptr, PathItem(0, init_state_ptr, init_state.actions()[0])));

	while (!open.empty()) {
		AStarOpenItem current = *open.begin();
		open.erase(open.begin());
		closed.insert(current.state);

		auto current_state = current.state.get();
		auto new_g_cost = current.g_cost + 1;

		if (current_state->isFinal()) {
			return getPath(paths, current.state, init_state_ptr);
		}

		for (SearchAction &action : current_state->actions()) {
			if (memUsageSucceeded(mem_limit_)) {
				// Stop if memory usage is too high
				std::cerr << "Memory limit exceeded" << std::endl;
				return {};
			}

			auto new_state = std::make_shared<SearchState>(action.execute(*current_state));

			// Don't add to open if already in closed
			if (closed.find(new_state) != closed.end()) {
				continue;
			}

			double f_updated = new_g_cost + compute_heuristic(*new_state, *heuristic_);
			auto new_item = AStarOpenItem(new_state, new_g_cost, f_updated);
			auto in_map = paths.find(new_state);
			// If item is in paths, get its f_cost and search for it in open
			auto in_open = (in_map != paths.end()) ? open.find(AStarOpenItem(new_state, 0, in_map->second.f_cost)) : open.end();

			if (in_open == open.end() || in_open->f_cost > f_updated) {
				if (in_open != open.end()) { // Only if state is in open
					open.erase(in_open);
				}
				open.insert(new_item);
				paths.insert_or_assign(new_state, PathItem(f_updated, current.state, action));
			}
		}
	}

	return {};
}
