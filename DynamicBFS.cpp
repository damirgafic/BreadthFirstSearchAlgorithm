/*
	This solution is a more accurate translation of the text book's algorithm
	for breadth-first search.

	What's different about this implementation is that it dynamically
	generates a search tree and it encodes the states at the bit level
	instead of using strings.

	Both of these facts may prove useful to you in future projects.  For one,
	the text mentions using bits to encode genetic algorithms.  For another,
	the search spaces you may deal with will be too big to manually code.
	Finally, the sample code for the text is pretty sparse when it comes to
	C++.

	The search space in this implemenation (the tree) is procedurally
	generated.

	I may refactor this later as some of the code here is pretty smelly, but
	it works and is mostly clear.  One big improvement would be to use smart
	pointers.

	Some methods listed in the text were ommitted or not implemented
	because they were not needed to solve this problem.

*/

#include <algorithm>
#include <deque>
#include <iostream>
using namespace std;

// R=River, C=Cabbage, G=Goat, W=Wolf
// PCGW PCGW : state
// PCGW pcgw : PCGW cross left, pcgw cross right
// Encoding scheme:  1100 0011 = 195 = PC|GW
#define RW 		0x01	// wolf on right
#define RG 		0x02	// goat on right
#define RC		0x04 	// cabbage on right
#define RP		0x08 	// peasant on right
#define LW		0x10	// wolf on left
#define LG		0x20	// goat on left
#define LC		0x40	// cabbage on left
#define LP		0x80	// peasant on left
#define RPCGW   0x0F
#define PCGWR   0xF0
#define PGRCW   0xA5
#define CWRPG   0x5A
#define PCGRW	0xE1
#define WRPCG	0x1E
#define CRPGW	0x4B
#define PGWRC	0xB4
#define GRPCW	0x2D
#define PCWRG	0xD2


class Problem
{
	protected:
    short initial,        	// initial state of the problem
           goal;        	// goal state of the problem
    public:

    // specifies initial state, and optionally a goal state.
	// child class can specify additional goal state(s)
    Problem(short initial, short goal = 0)
	{
		this->initial = initial;
		this->goal = goal;
	}

    // returns a list of actions that can be executed by a specified state.
    virtual deque<short> actions(short) = 0;

    // returns the state that results from a given state and given action
    virtual short result(short, short) = 0;

    // returns true if given state is the goal state.
    bool goal_test(short g) const
	{
		return goal == g;	}
};


class Node
{
	private:
    short state;       // the state represented by this node
    short action;      // the action taken by the parent to get here
    Node* parent;       // a pointer to the node that generated this one
    deque<short> soln;
    public:

	// initializes the Node with a given state, the action that got us to this
	// state, and the pointer to a parent Node.
	Node(short, short = 0, Node* = nullptr);
    // returns a list of dynamically created nodes that are children of
    // a node with the given state.
    deque<Node*> expand(Problem*, short);
    // generates a child node with a state given an action.
    Node* childNode(Problem*, short);
    // returns a list of the actions taken to get from root to here
    deque<short> solution();
    // returns a list of the nodes in the path from root to here
    deque<Node*> path();

	short getState() const { return state; }
};

Node::Node(short s, short a, Node* p)
{
	state = s;
	action = a;
	if(p)
		soln = p->solution();
	soln.push_back(a);
}

deque<short> Node::solution()
{
	return soln;
}

class BFSProblem : public Problem
{
	public:
	BFSProblem(short initial, short goal = 0) : Problem(initial, goal) {}

	virtual deque<short> actions(short);
	virtual short result(short, short);

	short getInitial() const;
	short getGoal() const { return goal; }
};

// action encoding:
// 1100 0000 = 192 = PC cross to the left
// 0000 1001 = 9 = PW cross to the right.
deque<short> BFSProblem::actions(short state)
{
	deque<short> acts;

	if(state == RPCGW)
		acts.push_back(LP|LG);
	else if(state == PGRCW)
	{
		acts.push_back(RP);
		acts.push_back(RP|RG);
	}
	else if(state == PCGRW)
	{
		acts.push_back(RP|RC);
		acts.push_back(RP|RG);
	}
	else if(state == CRPGW)
	{
		acts.push_back(LP|LG);
		acts.push_back(LP|LW);
	}
	else if(state == PCWRG)
	{
		acts.push_back(RP|RC);
		acts.push_back(RP|RW);
		acts.push_back(RP);
	}
	else if(state == WRPCG)
	{
		acts.push_back(LP|LC);
		acts.push_back(LP|LG);
	}
	else if(state == CWRPG)
	{
		acts.push_back(LP);
		acts.push_back(LP|LG);
	}
	else if(state == GRPCW)
	{
		acts.push_back(LP);
		acts.push_back(LP|LC);
		acts.push_back(LP|LW);
	}
	else if(state == PGWRC)
	{
		acts.push_back(RP|RG);
		acts.push_back(RP|RW);
	}
	return acts;
}

/*
	Turns on the action bits and turns off their corresponding bits on
	the other side of the river.  Then, returns the new encoded state.
	So, if a state is 1100 0011 = PC | GW and the action is
	0000 1100, then the new state returned will be 0000 1111 = |PCGW
*/
short BFSProblem::result(short state, short action)
{
	if(action == LP)
		state = state & ~RP | LP;
	else if(action == (LP | LC))
		state = state & ~RP & ~RC | LP | LC;
	else if(action == (LP | LG))
		state = state & ~RP & ~RG | LP | LG;
	else if(action == (LP | LW))
		state = state & ~RP & ~RW | LP | LW;
	else if(action == RP)
		state = state & ~LP | RP;
	else if(action == (RP | RC))
		state = state & ~LP & ~LC | RP | RC;
	else if(action == (RP | RG))
		state = state & ~LP & ~LG | RP | RG;
	else if(action == (RP | RW))
		state = state & ~LP & ~LW | RP | RW;

	return state;
}

// just a helper
short BFSProblem::getInitial() const
{ return initial; }

// the function for expanding a child node
Node* childNode(Problem* prob, Node* parent, short action)
{
	return new Node(prob->result(parent->getState(), action), action, parent);
}

// the BFS implementation, returns a list of actions as the solution
deque<short> BFS(BFSProblem* p)
{
	deque<Node*> frontier,  // all child nodes of a node
		expanded;			// tracks all the nodes that got created.
	deque<short> solution, 	// the sequence of actions to get to goal state
				explored;	// the list of explored states

	Node* node = new Node(p->getInitial());
	frontier.push_back( node );
	expanded.push_back( node );

	if(p->goal_test(node->getState()))
		solution = node->solution();

	while(!frontier.empty())
	{
		node = frontier.front();
		frontier.pop_front();

		explored.push_back(node->getState());

		for(short action : p->actions(node->getState()))
		{
			Node* child = childNode(p, node, action);
			expanded.push_back(child);

			if(find(frontier.begin(), frontier.end(), child) == frontier.end()
				&& find(explored.begin(), explored.end(), child->getState()) == explored.end())
			{
				if(p->goal_test(child->getState()))
				{
					// free memory
					for(Node* n : expanded)
						delete n;

					return child->solution();
				}

				frontier.push_back(child);
			}
		}
	}

	return solution;
}

int main()
{							      //start, goal
	BFSProblem* b = new BFSProblem(RPCGW, PCGWR);

	deque<short> solution = BFS(b);

	// translate the actions
	for(short action : solution)
	{
		switch(action)
		{
			case RP:
				cout << "Peasant crosses right." << endl;
			break;
			case RP|RC:
				cout << "Peasant and cabbage crosses right." << endl;
			break;
			case RP|RG:
				cout << "Peasant and goat crosses right." << endl;
			break;
			case RP|RW:
				cout << "Peasant and wolf crosses right." << endl;
			break;
			case LP:
				cout << "Peasant crosses left." << endl;
			break;
			case LP|LC:
				cout << "Peasant and cabbage crosses left." << endl;
			break;
			case LP|LG:
				cout << "Peasant and goat crosses left." << endl;
			break;
			case LP|LW:
				cout << "Peasant and wolf crosses left." << endl;
			break;
		}
	}

	delete b;
}
