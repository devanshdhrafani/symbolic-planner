#include "planner.hpp"

using namespace std;

bool print_status = true;
bool debug = true;

// Check if given grounded action can be taken in given state

// Get successors of a state (list of grounded actions)


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

// A* search

list<GroundedAction> planner(Env* env)
{
    SymbolicPlanner planner = SymbolicPlanner(env);

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

    // Perform A* search
    // planner.a_star_search();

    // Backtrack to get the plan
    // list<GroundedAction> plan = planner.backtrack();

    // blocks world example
    list<GroundedAction> actions;
    actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

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