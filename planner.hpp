#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <set>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <time.h>
#include "env.hpp"

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

// For our vector subset, at every step we have A.size()-i-1
// choices to include. A loop helps us to choose each element
// and then move to the indices further present in the array.
// At the end we backtrack to choose a different index.
void comb_util(vector<string>& A, vector<vector<string> >& res,
                 vector<string>& subset, int index)
{
    res.push_back(subset);
      // Loop to choose from different elements present
      // after the current index 'index'
    for (int i = index; i < A.size(); i++) 
    {
        // include the A[i] in subset.
        subset.push_back(A[i]);
 
        // move onto the next element.
        comb_util(A, res, subset, i + 1);
 
        // exclude the A[i] from subset and triggers
        // backtracking.
        subset.pop_back();
    }
    return;
}
 
// Combination code modified from GeeksforGeeks example
// Find and return all combinations of symbols that can be picked. 
vector<vector<string>> combinations(vector<string>& A, int n)
{
    vector<string> subset;
    vector<vector<string> > res;
 
    // keeps track of current element in vector A
    // and the number of elements present in the array subset
    int index = 0;
    comb_util(A, res, subset, index);
    
    vector<vector<string>> ret;
    
    // Only return combinations whose size is n
    for(int i=0; i<res.size(); i++)
    {
        if(res[i].size()==n)
        {
            ret.push_back(res[i]);
        }
    }
    return ret;
}

// Generates permutations of a string vector using Heap's algorithm
void permute(vector<string> a, vector<vector<string>> &all_permutations, int l, int r)
{
	if (l == r)
		all_permutations.push_back(a);
	else {
		for (int i = l; i <= r; i++) {
			swap(a[l], a[i]);
			permute(a, all_permutations, l + 1, r);
			swap(a[l], a[i]);
		}
	}
}

string condition_to_string(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& stateset)
{
    set<string> state;
    string string_return = "";

    for(GroundedCondition gc : stateset)
        state.insert(gc.toString());
    for (auto it = state.begin(); it != state.end(); it++) 
        string_return += *it; 
    return string_return;
}

class SymbolicPlanner
{
    private:
        vector<GroundedAction> grounded_actions;
        Env* env;

    public:
        SymbolicPlanner(Env* env)
        {
            this->env = env;
        }
        struct node
        {
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;
            double g = std::numeric_limits<int>::max();
            double h = std::numeric_limits<int>::max();
        
            int parent = -1; // state of previous(parent) node
            string parent_node_str = "";
        };

        unordered_set<string> closed_list; // idx of expanded nodes
        unordered_map<string, node> node_info; // idx, node
        unordered_map<string, int> state_map; // string_state, idx

        // f value, string state: sorted according to f value
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> open_list;
        
        vector<GroundedAction> get_grounded_actions() const
        {
            return this->grounded_actions;
        }

        list<GroundedAction> backtrack();
        void compute_all_grounded_actions();
        int heuristic(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state);
        void init_start_node();
        bool in_closed_list(string idx);
        bool is_action_valid(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state, GroundedAction action);
        node take_action(node n, GroundedAction a);
        bool goal_reached(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state);
        void a_star_search();

        // list<GroundedAction> backtrack();
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}
