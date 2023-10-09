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

	std::cout << state << std::endl;
	std::cout << result << std::endl;

	return result;
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
	std::multiset<AStarFrontierItem> frontier;
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
				std::cout << "Memory limit exceeded (" << getCurrentRSS();
				std::cout << " out of " << mem_limit_ << "), returning empty path" << std::endl;
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