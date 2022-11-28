#include "planner.hpp"

using namespace std;

bool print_status = true;
bool debug = false;
int which_heuristic = 1;

// backrack from goal to start
list<GroundedAction> SymbolicPlanner::backtrack()
{
    list<GroundedAction> plan;
    auto goal_state = this->env->get_goal_conditions();
    string goal_str = condition_to_string(goal_state);
    auto start_state = this->env->get_initial_conditions();
    string start_str = condition_to_string(start_state);

    string current_state = goal_str;
    while (current_state != start_str)
    {
        int action_index = node_info[current_state].parent;
        vector<GroundedAction> all_gacs = this->get_grounded_actions();
        GroundedAction action = all_gacs.at(action_index);
        plan.push_front(action);
        current_state = node_info[current_state].parent_node_str;
    }
    return plan;
}

// Compute all possible grounded actions from a state
void SymbolicPlanner::compute_all_grounded_actions()
{
    // All actions (ungrounded)
    auto actions = this->env->get_actions();

    // All symbols as unordered_set of strings
    auto symbols = this->env->get_symbols();

    // Convert symbols to vector of strings
    vector<string> symbols_vec;
    for (string s : symbols)
        symbols_vec.push_back(s);

    int num_args = 0;

    vector<vector<string>> symbol_combinations;
    vector<vector<string>> symbol_permutations;
    
    // For each action (ungrounded), compute all possible grounded actions
    for (Action a : actions)
    {
        // Find number of arguments for action a
        num_args = a.get_args().size();

        // Find all combinations of symbols for each action
        symbol_combinations = combinations(symbols_vec, num_args);

        // Find all possible permutations of symbols for the arguments
        for(int i=0; i<symbol_combinations.size(); i++)
        {
            permute(symbol_combinations[i], symbol_permutations, 0, num_args - 1);
        }
        
        // Get all possible grounded actions for symbol permutations and add to list of grounded actions
        for (vector<string> symbol_permutation : symbol_permutations)
        {
            list<string> args = a.get_args();
            list<string> grounded_args;
            copy(symbol_permutation.begin(), symbol_permutation.end(), back_inserter(grounded_args));

            // build map from action arguments to grounded action arguments
            unordered_map<string, string> arg_map;
            auto arg_it = args.begin();
            auto grounded_arg_it = grounded_args.begin();
            while (arg_it != args.end() && grounded_arg_it != grounded_args.end())
            {
                arg_map[*arg_it] = *grounded_arg_it;
                ++arg_it;
                ++grounded_arg_it;
            }

            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_precons;
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_effects;

            // make grounded preconditions using action preconditions and arg_map
            for (Condition precon : a.get_preconditions())
            {
                list<string> grounded_precon_args;
                for (string arg : precon.get_args())
                {
                    if(arg_map[arg] == "")
                        grounded_precon_args.push_back(arg);
                    else
                        grounded_precon_args.push_back(arg_map[arg]);
                }
                GroundedCondition grounded_precon(precon.get_predicate(), grounded_precon_args, precon.get_truth());
                grounded_precons.insert(grounded_precon);
            }

            // make grounded effects using action effects and arg_map
            for (Condition effect : a.get_effects())
            {
                list<string> grounded_effect_args;
                for (string arg : effect.get_args())
                {
                    if(arg_map[arg] == "")
                        grounded_effect_args.push_back(arg);
                    else
                        grounded_effect_args.push_back(arg_map[arg]);
                }
                GroundedCondition grounded_effect(effect.get_predicate(), grounded_effect_args, effect.get_truth());
                grounded_effects.insert(grounded_effect);
            }

            GroundedAction ga(a.get_name(), grounded_args, grounded_precons, grounded_effects);
            this->grounded_actions.push_back(ga);
        }

        // Clear symbol combinations and permutations for next action
        symbol_combinations.clear();
        symbol_permutations.clear();
    }
}

// Calculate heuristic value for a given node
int SymbolicPlanner::heuristic(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state)
{
    int heauristic_value = 0;
    auto goal = this->env->get_goal_conditions();
    switch (which_heuristic)
    {
        // h(s) = 0
        case 0:
            heauristic_value = 0;
            break;

        // h(s) = No of unsatisfied literals
        case 1:
            heauristic_value = simple_heur(state);
            break;

        // h(s) = empty-delete-list
        case 2:
            heauristic_value = empty_delete_list_heur(state);
            break;
    }
    
    return heauristic_value;
}

// h(s) = No of unsatisfied literals
int SymbolicPlanner::simple_heur(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state)
{
    int heauristic_value = 0;
    auto goal = this->env->get_goal_conditions();
    for(GroundedCondition condition : goal)
    {
        if(state.find(condition) == state.end())
            heauristic_value++;
    }
    return heauristic_value;
}

// Compute empty-delete-list heuristic
int SymbolicPlanner::empty_delete_list_heur(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state)
{
    unordered_set<string> closed_list_; // idx of expanded nodes
    unordered_map<string, node> node_info_; // idx, node
    unordered_map<string, int> state_map_; // string_state, idx
    // f value, string state: sorted according to f value
    priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> open_list_;

    int num_actions = 0;

    // Add start state to open list
    string start_state_str = condition_to_string(state);
    node_info_[start_state_str].state = state;
    node_info_[start_state_str].g = 0;
    node_info_[start_state_str].h = simple_heur(state);
    int f = 0 + node_info_[start_state_str].h;
    open_list_.push(make_pair(f, start_state_str));

    auto goal_state = this->env->get_goal_conditions();
    string goal_str = condition_to_string(goal_state);
    while(!open_list_.empty())
    {
        // cout<<"Open list size: "<<open_list_.size()<<endl;
        // cout<<"Closed list size: "<<closed_list_.size()<<endl;
        pair<int, string> current_node_idx = open_list_.top();   //f-value, cell state
        open_list_.pop();
        string current_node_str = current_node_idx.second;
        if(in_closed_list(closed_list_, current_node_str))
            continue;
        closed_list_.insert(current_node_str);

        node current_node = node_info_[current_node_str];

        int action_count = -1;

        for(GroundedAction ga : this->grounded_actions)
        {
            ++action_count;
            if(this->is_action_valid(current_node.state, ga))
            {
                node next_node = this->take_action_relaxed(current_node, ga);
                string next_node_str = condition_to_string(next_node.state);

                if(in_closed_list(closed_list_, next_node_str))
                    continue;

                // check if new node g-value is greater than current g-value + cost
                if(node_info_[next_node_str].g > current_node.g + 1)
                {
                    // break if goal reached
                    if(goal_reached(next_node.state))
                    {
                        node_info_[goal_str].g = current_node.g + 1;
                        node_info_[goal_str].parent = action_count;
                        node_info_[goal_str].state = next_node.state;
                        node_info_[goal_str].parent_node_str = current_node_str;

                        string ittr = goal_str;
                        while(ittr != start_state_str)
                        {
                            num_actions++;
                            ittr = node_info_[ittr].parent_node_str;
                        }
                        return num_actions;
                    }
                    node_info_[next_node_str].g = current_node.g + 1;
                    node_info_[next_node_str].h = simple_heur(next_node.state);
                    node_info_[next_node_str].parent = action_count;
                    node_info_[next_node_str].state = next_node.state;
                    node_info_[next_node_str].parent_node_str = current_node_str;
                    // cout<<node_info_[next_node_str].h<<endl;
                    int f = node_info_[next_node_str].g + node_info_[next_node_str].h;
                    open_list_.push(make_pair(f, next_node_str));
                }
            }
        }
    }
    return num_actions;
}


// Initialize start node for A* search
void SymbolicPlanner::init_start_node()
{
    // Get initial state
    auto init_state = this->env->get_initial_conditions();
    string initial_state = condition_to_string(init_state);
    node_info[initial_state].state = init_state;
    node_info[initial_state].g = 0;
    node_info[initial_state].h = heuristic(node_info[initial_state].state);
    int f = node_info[initial_state].g + node_info[initial_state].h;
    open_list.push(make_pair(f, initial_state));
}

bool SymbolicPlanner::in_closed_list(unordered_set<string> &closed_list, string &idx)
{
    if (closed_list.find(idx) == closed_list.end())
        return false;
    else
        return true;
}

// Check if action can be taken in given state
bool SymbolicPlanner::is_action_valid(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state, GroundedAction &a)
{
    // Check if all preconditions are satisfied
    for (GroundedCondition precon : a.get_preconditions())
    {
        if (state.find(precon) == state.end())
            return false;
    }
    return true;
}

// Take action in given state
SymbolicPlanner::node SymbolicPlanner::take_action(node &n, GroundedAction &a)
{
    node new_node;
    new_node.state = n.state;

    // Add effects of action to new state
    for (GroundedCondition effect : a.get_effects())
    {
        if(effect.get_truth())
        {
            new_node.state.insert(effect);
        }
        else
        {
            effect.flip_truth();
            new_node.state.erase(new_node.state.find(effect));
        }
    }

    return new_node;
}

// Take relaxed action in given state
SymbolicPlanner::node SymbolicPlanner::take_action_relaxed(node &n, GroundedAction &a)
{
    node new_node;
    new_node.state = n.state;

    // Add effects of action to new state
    for (GroundedCondition effect : a.get_effects())
    {
        if(effect.get_truth())
        {
            new_node.state.insert(effect);
        }
    }

    return new_node;
}

// Check if goal reached
bool SymbolicPlanner::goal_reached(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &state)
{
    for (GroundedCondition goal : this->env->get_goal_conditions())
    {
        if (state.find(goal) == state.end())
            return false;
    }
    return true;
}

// A* search
void SymbolicPlanner::a_star_search()
{
    auto goal_state = this->env->get_goal_conditions();
    string goal_str = condition_to_string(goal_state);
    while(!open_list.empty())
    {
        // cout<<"Open list size: "<<open_list.size()<<endl;
        // cout<<"Closed list size: "<<closed_list.size()<<endl;
        pair<int, string> current_node_idx = open_list.top();   //f-value, cell state
        open_list.pop();
        string current_node_str = current_node_idx.second;
        if(in_closed_list(closed_list, current_node_str))
            continue;
        closed_list.insert(current_node_str);

        node current_node = this->node_info[current_node_str];

        int action_count = -1;

        for(GroundedAction ga : this->grounded_actions)
        {
            ++action_count;
            if(this->is_action_valid(current_node.state, ga))
            {
                node next_node = this->take_action(current_node, ga);
                string next_node_str = condition_to_string(next_node.state);

                if(in_closed_list(closed_list, next_node_str))
                    continue;

                // check if new node g-value is greater than current g-value + cost
                if(node_info[next_node_str].g > current_node.g + 1)
                {
                    // break if goal reached
                    if(goal_reached(next_node.state))
                    {
                        node_info[goal_str].g = current_node.g + 1;
                        node_info[goal_str].h = heuristic(next_node.state);
                        node_info[goal_str].parent = action_count;
                        node_info[goal_str].state = next_node.state;
                        node_info[goal_str].parent_node_str = current_node_str;
                        break;
                    }
                    node_info[next_node_str].g = current_node.g + 1;
                    node_info[next_node_str].h = heuristic(next_node.state);
                    node_info[next_node_str].parent = action_count;
                    node_info[next_node_str].state = next_node.state;
                    node_info[next_node_str].parent_node_str = current_node_str;
                    int f = node_info[next_node_str].g + node_info[next_node_str].h;
                    open_list.push(make_pair(f, next_node_str));
                }
            }
        }
    }
}

list<GroundedAction> planner(Env* env)
{
    SymbolicPlanner planner = SymbolicPlanner(env);
    cout << endl;

    clock_t t;
    t = clock();

    // Compute all possible grounded actions
    planner.compute_all_grounded_actions();

    // print all grounded actions
    if(debug)
    {
        cout << "ALL Grounded Actions:" << endl;
        for (GroundedAction ga : planner.get_grounded_actions())
        {
            cout << ga;
        }
        cout << endl;
    }

    cout<<"Number of possible actions: "<<planner.get_grounded_actions().size()<<endl;
    
    planner.init_start_node();

    // Perform A* search
    planner.a_star_search();
    cout<<"Number of states expanded: "<<planner.closed_list.size()<<endl;

    list<GroundedAction> actions;
    // Backtrack to get the plan
    actions = planner.backtrack();

    t = clock() - t;
    cout<<"Time Taken: "<<((float)t)/CLOCKS_PER_SEC<<" seconds\n";

    // blocks world example
    // list<GroundedAction> actions;
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}