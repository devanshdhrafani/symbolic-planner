#include <iostream>
#include <fstream>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include "conditions.hpp"

using namespace std;

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};


class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_preconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_effects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    GroundedAction(string name, list<string> arg_values,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> precon,
                   unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effect)
    {
        this->name = name;
        for (string ar : arg_values)
            this->arg_values.push_back(ar);
        for (GroundedCondition gc : precon)
            this->grounded_preconditions.insert(gc);
        for (GroundedCondition gc : effect)
            this->grounded_effects.insert(gc);
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions() const
    {
        return this->grounded_preconditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects() const
    {
        return this->grounded_effects;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString();
        // os << "Grounded precondition: ";
        // for (GroundedCondition precond : gac.get_preconditions())
        //     os << precond;
        // os << endl;
        // os << "Grounded effect: ";
        // for (GroundedCondition effect : gac.get_effects())
        //     os << effect;
        // os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};
