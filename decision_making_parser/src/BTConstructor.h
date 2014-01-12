/*
 * BTConstructor.h
 *
 *  Created on: Nov 27, 2013
 *      Author: dan
 */

#ifndef BTCONSTRUCTOR_H_
#define BTCONSTRUCTOR_H_

#include <deque>
#include <map>
#include <sstream>
#include <vector>
#include <iostream>
#include "Container.h"

namespace bt_constructor{
using namespace std;

#define DEF_STACK_MAP(TYPE, MNAME, NAME)\
	std::map<std::string, TYPE> MNAME;\
	std::deque<TYPE> stack_##NAME;\
	void create_##NAME(){\
		TYPE t; t.lib=lib;\
		stack_##NAME.push_back(t);\
	}\
	void drop_##NAME(){\
		stack_##NAME.pop_back();\
	}\
	void add_##NAME(){\
		MNAME[NAME().name] = NAME();\
		drop_##NAME();\
	}\
	TYPE& NAME(){ return stack_##NAME.back(); }

#define DEF_STACK_VECTOR(TYPE, VNAME, NAME)\
	std::vector<TYPE> VNAME;\
	std::deque<TYPE> stack_##NAME;\
	void create_##NAME(){\
		TYPE t; t.lib=lib;\
		stack_##NAME.push_back(t);\
	}\
	void drop_##NAME(){\
		stack_##NAME.pop_back();\
	}\
	void add_##NAME(){\
		VNAME.push_back( NAME() );\
		drop_##NAME();\
	}\
	TYPE& NAME(){ return stack_##NAME.back(); }

class BTConstructor;
class Element{
public:
	mutable BTConstructor const* lib;
	mutable string tab;
};

class Node;
std::deque<Node>& stack_node(Node* _this);

class Node:public Element{
protected:
	Node(string t):type(t),line(0),pos(0){}
public:
	mutable string id;
	string type;
	string name;
	string decorator_name;
	string task_result;
	int line, pos;
	string file;
	Node():line(0),pos(0){}

	std::vector<Node> nodes;

	void create_node(){
		Node t; t.lib=lib;
		stack_node(this).push_back(t);
	}
	void drop_node(){
		stack_node(this).pop_back();
	}
	void add_node(){
		Node t = node();
		drop_node();
		if(stack_node(this).empty()){
			t.id = getId();
			nodes.push_back( t );
		}else{
			Node &n = node();
			t.id = n.getId();
			n.nodes.push_back( t );
		}
	}
	Node& node(){
		if(stack_node(this).empty()){
			return *this;
		}else{
			return stack_node(this).back();
		}
	}

	void create_call(){
		Node t; t.lib=lib;
		t.id = getId();
		nodes.push_back(t);
	}
	void add_call(){
	}
	Node& call(){
		return nodes.back();
	}

	std::string getId()const{ return id+"/"+name; }
};


class Tree:public Node{
public:
	Tree():Node("tree"){}
};

class BTConstructor:public Element, public Container{
public:
	BTConstructor(std::stringstream& errors, std::string filename):errors(errors), filename(filename){ lib=this; }
	DEF_STACK_MAP(Tree, trees, tree)
	mutable std::deque<Node> stack_node;
	Container* fsms;
	std::stringstream& errors;
	std::string filename;

	bool contains_tree(string name)const{
		return contains(name);
	}
	std::string copy_tree(std::string name)const{
		return copy(name);
	}

	bool contains_fsm(string name)const{
		return fsms->contains(name);
	}
	std::string copy_fsm(std::string name)const{
		return fsms->copy(name);
	}

	bool contains(std::string name)const{ return trees.find(name)!=trees.end(); }
	std::string copy(std::string name)const;


	void saveXml_fsm(std::ostream& out, std::string tab, std::string name, std::string id)const{
		fsms->saveXml(out, tab, name, id);
	}
	void saveXml_tree(std::ostream& out, std::string tab, std::string name, std::string id)const{
		saveXml(out, tab, name, id);
	}

	void saveXml(std::ostream& out, std::string tab, std::string name, std::string id)const;


	void saveDot_fsm(std::ostream& out, std::string tab, std::string name, std::string id)const{
		fsms->saveDot(out, tab, name, id);
	}
	void saveDot_tree(std::ostream& out, std::string tab, std::string name, std::string id)const{
		saveDot(out, tab, name, id);
	}

	void saveDot(std::ostream& out, std::string tab, std::string name, std::string id)const;


	void map_ids_tree(std::string name, std::string id)const{map_ids(name,id);}
	void map_ids_fsm(std::string name, std::string id)const{fsms->map_ids(name,id);}
	void map_ids(std::string name, std::string id)const;

	std::string search_simple_node(std::string name)const;

};

inline std::deque<Node>& stack_node(Node * _this){
	return _this->lib->stack_node;
}

#define FOREACH_VEC(TYPE, VEC) for(std::vector<TYPE>::const_iterator i=o.VEC.begin();i!=o.VEC.end();i++)
#define FOREACH_MAP(TYPE, VEC) for(std::map<std::string, TYPE>::const_iterator i=o.VEC.begin();i!=o.VEC.end();i++)

bool print_call_node(std::ostream& out, const Node& o);
std::ostream& operator<<(std::ostream& out, const Node& o);
std::ostream& operator<<(std::ostream& out, const Tree& o);
std::ostream& operator<<(std::ostream& out, const BTConstructor& o);

//-------------------

bool saveXml_call_node(std::ostream& out, const Node& o);
std::ostream& saveXml(std::ostream& out, const Node& o);
std::ostream& saveXml(std::ostream& out, const Tree& o);
std::ostream& saveXml(std::ostream& out, const BTConstructor& o);
void saveXml(std::string prefix, const BTConstructor& o);


//-------------------

bool saveDot_call_node(std::ostream& out, const Node& o);
std::ostream& saveDot(std::ostream& out, const Node& o);
std::ostream& saveDot(std::ostream& out, const Tree& o);
std::ostream& saveDot(std::ostream& out, const BTConstructor& o);
void saveDot(std::string prefix, const BTConstructor& o);

void map_ids(const Node& o);
void map_ids(const Tree& o);
void map_ids(const BTConstructor& o);

}


#undef FOREACH_MAP
#undef FOREACH_VEC
#undef DEF_STACK_MAP
#undef DEF_STACK_VECTOR_

#endif /* BTCONSTRUCTOR_H_ */
