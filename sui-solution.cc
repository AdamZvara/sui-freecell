#include <set>
#include <algorithm>
#include <queue>
#include <stack>

#include "card-storage.h"
#include "search-strategies.h"
#include "memusage.h"

#include <utility>

#define RESERVE 50000 // 50 MB memory reserve

typedef std::shared_ptr<SearchState> SearchSharedPtr; 

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

template <typename MapType>
std::vector<SearchAction> getPath(
	MapType &paths, 
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

bool searchStatePtrLesser(const SearchSharedPtr a, const SearchSharedPtr b) {
    return *a < *b;
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
		std::map<SearchSharedPtr, PathItem> paths;
    std::queue<SearchSharedPtr> open;
    std::set<SearchSharedPtr, decltype(searchStatePtrLesser)*> explored(searchStatePtrLesser);

    SearchSharedPtr initStatePtr = std::make_shared<SearchState>(init_state);
    open.push(initStatePtr);

    if (init_state.isFinal()) {
        // found final state
        return {};
    }

    while (!open.empty()) {
        SearchSharedPtr currState = open.front();
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

            SearchSharedPtr newState = std::make_shared<SearchState>(action.execute(*currState));

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
    std::map<SearchSharedPtr, PathItem> paths;
    std::stack<std::pair<SearchSharedPtr, int>> open;
    std::set<SearchSharedPtr, decltype(searchStatePtrLesser)*> openSet(searchStatePtrLesser);

    int currDepth = 0;
    SearchSharedPtr initStatePtr = std::make_shared<SearchState>(init_state);
    open.push(std::make_pair(initStatePtr, currDepth));

    if (init_state.isFinal()) {
        // found final state
        return {};
    }

    while (!open.empty()) {
        std::pair<SearchSharedPtr, int> currStatePair = open.top();
        SearchSharedPtr currState = currStatePair.first;
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

            SearchSharedPtr newState = std::make_shared<SearchState>(action.execute(*currState));
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

double nbOfCardsOnTop(std::array<WorkStack, nb_stacks> stacks, Card card_to_find) {
	// Try to find card with value one more (of the same color) in stacks
	for (const auto &stack : stacks) {
		auto storage = stack.storage();
		// Skip empty stack
		if (storage.size() == 0) {
			continue;
		}
		// For non-empty stack find card to be placed at top of the home pile
		auto card = std::find(storage.begin(), storage.end(), card_to_find);
		// Count the number of cards on top of this stack
		if (card != storage.end()) {
			return std::distance(card, storage.end()) - 1; // -1 because we don't count past the end
		}
	}

	// Card is at free cell 
	return 0;
}

inline bool hasEmptySpot(const GameState &state)  {
	auto free_cells_single_empty = std::find_if(state.free_cells.begin(), state.free_cells.end(), [](const FreeCell &cell) {
		return cell.canAccept(Card(Color::Heart, 1));
	});
	auto stacks_single_empty = std::find_if(state.stacks.begin(), state.stacks.end(), [](const WorkStack &stack) {
		return stack.canAccept(Card(Color::Heart, 1));
	});
	return free_cells_single_empty != state.free_cells.end() || stacks_single_empty != state.stacks.end();
} 

inline int CardOutOfHome(const GameState &state) {
	int cards_out_of_home = king_value * colors_list.size();
    for (const auto &home : state.homes) {
        auto opt_top = home.topCard();
        if (opt_top.has_value())
            cards_out_of_home -= opt_top->value;
    }
	return cards_out_of_home;
}

// Using Heineman's staged deepening heuristic with comination of CardsOutOfHome heuristic
// Source: https://www.human-competitive.org/sites/default/files/elyasaf-hauptmann-sipper-paper.pdf
double StudentHeuristic::distanceLowerBound(const GameState &state) const {
	int result = 0;
	int colors_idx = 0;
	std::array<std::optional<Color>, 4> used_colors;

	// For each top card of home pile
   	for (const auto &home : state.homes) {
		auto opt_top = home.topCard();
		// If there is any card placed in that home pile and it's not the king
		if (opt_top.has_value() && opt_top->value != king_value) {
			// calculate number cards on top for this card in working piles
			result += nbOfCardsOnTop(state.stacks, Card(opt_top->color, opt_top->value + 1));
			used_colors[colors_idx++] = opt_top->color;
		}
	}
    	
	// Run through home piles for the remaining colors
	for (auto color: colors_list) {
		auto a = std::find(used_colors.begin(), used_colors.end(), color);
		if (a == used_colors.end()) {
			result += nbOfCardsOnTop(state.stacks, Card(color, 1));
		}
	}

	// Multiply result by 2 if there are no empty free cells or stacks
	if (!hasEmptySpot(state)) {
		result *= 2;
	}

	// Combine CardsOutOfHome heuristic and result
	return CardOutOfHome(state) + result;
}

/* Since this is the only operator that isn't defined and can access
 private members, it is used to compare states based on their stacks 
 (e.g. 2 states are considered equal, if their stacks are equal, so
 we don't take their freecells into consideration, because many expanded states
 only differ with card position in free cells or home cells ...) */
bool operator==(const SearchState &s1, const SearchState &s2) {
		return s1.state_.stacks < s2.state_.stacks;
}

bool searchStatePtrEqAstar(const SearchSharedPtr lhs, const SearchSharedPtr rhs) {
		return *lhs == *rhs;
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
		std::set<SearchSharedPtr, decltype(searchStatePtrEqAstar)*> closed(searchStatePtrEqAstar);
		std::map<SearchSharedPtr, PathItem, decltype(searchStatePtrEqAstar)*> paths(searchStatePtrEqAstar);

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
