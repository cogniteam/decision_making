/*
 * BTConstructor.cpp
 *
 *  Created on: Nov 29, 2013
 *      Author: dan
 */

#include "BTConstructor.h"
#include <fstream>
#include "ParserExceptions.h"

namespace bt_constructor{
using namespace std;


#define FOREACH_VEC(TYPE, VEC) for(std::vector<TYPE>::const_iterator i=o.VEC.begin();i!=o.VEC.end();i++)
#define FOREACH_MAP(TYPE, VEC) for(std::map<std::string, TYPE>::const_iterator i=o.VEC.begin();i!=o.VEC.end();i++)

void bt_NODE_bgn(std::ostream& out, std::string tab, std::string node, std::string name, string id){
	string _name = " name=\""+ name +"\"";
	string _id = " id=\""+ id +"\"";
	out<<tab<<"<"<<node<<"" <<_name <<_id <<">";
}

void bt_NODE_bgn(std::ostream& out, std::string tab, std::string node, std::string name, string id, string attr, string attr_value){
    string _name = " name=\""+ name +"\"";
    string _id = " id=\""+ id +"\"";
    string _attr = " " + attr + "=\""+ attr_value +"\"";
    out<<tab<<"<"<<node<<"" <<_name <<_id << _attr <<">";
}

void bt_tree_bgn(std::ostream& out, std::string tab, std::string name, string id){
	bt_NODE_bgn(out, tab, "plan", name , id);
}
void bt_tree_end(std::ostream& out, std::string tab){
	out<<tab<<"</plan>";
}
void bt_par_bgn(std::ostream& out, std::string tab, std::string name, string id){
	bt_NODE_bgn(out, tab, "par", name , id);
}
void bt_par_end(std::ostream& out, std::string tab){
	out<<tab<<"</par>";
}
void bt_seq_bgn(std::ostream& out, std::string tab, std::string name, string id){
	bt_NODE_bgn(out, tab, "seq", name , id);
}
void bt_seq_end(std::ostream& out, std::string tab){
	out<<tab<<"</seq>";
}
void bt_sel_bgn(std::ostream& out, std::string tab, std::string name, string id){
	bt_NODE_bgn(out, tab, "sel", name , id);
}
void bt_sel_end(std::ostream& out, std::string tab){
	out<<tab<<"</sel>";
}
void bt_task_bgn(std::ostream& out, std::string tab, std::string name, string id){
	bt_NODE_bgn(out, tab, "task", name , id);
}
void bt_task_end(std::ostream& out, std::string tab){
	out<<tab<<"</task>";
}
void bt_dec_bgn(std::ostream& out, std::string tab, std::string name, string id, string dec_type){
	bt_NODE_bgn(out, tab, "dec", name , id, "type", dec_type);
}
void bt_dec_end(std::ostream& out, std::string tab){
	out<<tab<<"</dec>";
}
void bt_call(std::ostream& out, std::string tab, std::string name, std::string type, string id){
	if(type=="rtask"){
		string _name = " name=\"TASK["+ name +"]\"";
		string _id = " id=\""+ id +"\"";
		out<<tab<<"<task"<< _name << _id <<" />";
	}
}
void bt_task_result(std::ostream& out, std::string tab, std::string name, string id, string task_result){
    bt_NODE_bgn(out, tab, "task_result", name , id, "result", task_result);
}
void bt_task_result_after(std::ostream& out, std::string tab, std::string name, string id, string task_result){
    bt_NODE_bgn(out, tab, "task_result_after", name , id, "result", task_result);
}
bool saveXml_call_node(std::ostream& out, const Node& o){
	string resolved = "";
	if(o.type == "bt"){
		bool isResolved = o.lib->contains_tree(o.name);
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   BT("<<o.name<<") definition does not found." <<endl;
			out<<o.tab<<"<error>"<<endl;
			out<<o.tab<<"   "<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			out<<o.tab<<"   "<<"   BT("<<o.name<<") definition does not found." <<endl;
			out<<o.tab<<"</error>";
		}else{
			bt_task_bgn(out, o.tab, o.name, o.getId()); out<<endl;
			o.lib->saveXml_tree(out, o.tab+"    ", o.name, o.getId()); out<<endl;
			bt_task_end(out, o.tab);
		}
		return true;
	}
	if(o.type == "fsm"){
		bool isResolved = o.lib->contains_fsm(o.name);
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   FSM("<<o.name<<") definition does not found." <<endl;
			out<<o.tab<<"<error>"<<endl;
			out<<o.tab<<"   "<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			out<<o.tab<<"   "<<"   FSM("<<o.name<<") definition does not found." <<endl;
			out<<o.tab<<"</error>";
		}else{
			string tab = o.tab+"   ";
			bt_task_bgn(out, o.tab, o.name, o.getId()); out<<endl;
			out<<tab<<"<scxml>"<<endl;
			o.lib->saveXml_fsm(out, tab+"    ", o.name, o.getId());	out<<endl;
			out<<tab<<"</scxml>"<<endl;
			bt_task_end(out, o.tab);
		}
		return true;
	}
	if(o.type == "rtask"){
		bt_call(out, o.tab, o.name, o.type, o.getId());
		return true;
	}
	return false;
}

std::ostream& saveXml(std::ostream& out, const Node& o){
	if(saveXml_call_node(out, o)) return out;
	if(o.type == "par")	bt_par_bgn(out, o.tab, o.name, o.getId());
	if(o.type == "seq")	bt_seq_bgn(out, o.tab, o.name, o.getId());
	if(o.type == "sel")	bt_sel_bgn(out, o.tab, o.name, o.getId());

	/**
	 * TODO Unique id
	 */
	if(o.type == "dec")	bt_dec_bgn(out, o.tab, o.name, o.getId(), o.decorator_name);

	if(o.type == "task_result") bt_task_result(out, o.tab, o.name, o.getId(), o.task_result);
	if(o.type == "task_result_after") bt_task_result_after(out, o.tab, o.name, o.getId(), o.task_result);

	if(o.type == "task")bt_task_bgn(out, o.tab, o.name, o.getId());
	FOREACH_VEC(Node, nodes){
		out<<endl;
		i->lib = o.lib;
		i->tab = o.tab+"      ";
		i->id = o.getId();
		saveXml(out, *i);
	}
	out<<endl;
	if(o.type == "par")	bt_par_end(out, o.tab);
	if(o.type == "seq")	bt_seq_end(out, o.tab);
	if(o.type == "sel")	bt_sel_end(out, o.tab);
	if(o.type == "dec")	bt_dec_end(out, o.tab);
	if(o.type == "task")bt_task_end(out, o.tab);
	return out;
}


std::ostream& saveXml(std::ostream& out, const Tree& o){
	bt_tree_bgn(out, o.tab, o.name, o.getId());
	FOREACH_VEC(Node, nodes){
		out<<endl;
		i->lib = o.lib;
		i->tab = o.tab+"      ";
		i->id = o.getId();
		saveXml(out, *i);
	}
	out<<endl;
	bt_tree_end(out, o.tab);
	return out;
}

std::ostream& saveXml(std::ostream& out, const BTConstructor& o){
	FOREACH_MAP(Tree, trees){
		i->second.lib = &o;
		i->second.tab = "";
		saveXml(out, i->second)<<endl;
	}
	return out;
}


void saveXml(std::string path_prefix, const BTConstructor& o){
	FOREACH_MAP(Tree, trees){
		std::stringstream filename; filename << path_prefix << i->second.name <<".btxml";
		std::ofstream out(filename.str().c_str());
		if(out.is_open()){
			out<<"<?xml version=\"1.0\"?>"; out<<endl;
			out<<"<!-- from file: "<<o.filename<<" -->"; out<<endl;
			i->second.lib = &o;
			i->second.tab = "";
			saveXml(out, i->second)<<endl;
			out.close();
		}else{
			throw PEFileNotCreated(filename.str());
		}
	}
}

//=========================================================================================


bool print_call_node(std::ostream& out, const Node& o){
	string resolved = "";
	if(o.type == "bt"){
		bool isResolved = o.lib->contains_tree(o.name);
		resolved = isResolved?"resolved":"not found";
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   BT("<<o.name<<") definition does not found." <<endl;
		}
			out<<o.tab<<"call "<<o.type<<" : "<<o.name << " : " << resolved;
		return true;
	}
	if(o.type == "fsm"){
		bool isResolved = o.lib->contains_fsm(o.name);
		resolved = isResolved?"resolved":"not found";
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   FSM("<<o.name<<") definition does not found." <<endl;
		}
			out<<o.tab<<"call "<<o.type<<" : "<<o.name << " : " << resolved;
		return true;
	}
	if(o.type == "rtask"){
			out<<o.tab<<"call "<<"task"<<" : "<<o.name;
		return true;
	}
	return false;
}

std::ostream& operator<<(std::ostream& out, const Node& o){
	if(print_call_node(out, o)) return out;
	out<<o.tab<<"Node{"<<endl;
	out<<o.tab<<"   name="<<o.name<<endl;
	out<<o.tab<<"   type="<<o.type<<endl;
	out<<o.tab<<"   Nodes:"<<endl;
	FOREACH_VEC(Node, nodes){
		i->lib = o.lib;
		i->tab = o.tab+"      ";
		out<<(*i)<<endl;
	}
	out<<o.tab<<"}";
	return out;
}


std::ostream& operator<<(std::ostream& out, const Tree& o){
	out<<"BT{"<<endl;
	out<<"   name="<<o.name<<endl;
	out<<"   Nodes:"<<endl;
	FOREACH_VEC(Node, nodes){
		i->lib = o.lib;
		i->tab = "      ";
		out<<(*i)<<endl;
	}
	out<<"}";
	return out;
}

std::ostream& operator<<(std::ostream& out, const BTConstructor& o){
	FOREACH_MAP(Tree, trees){
		i->second.lib = &o;
		out<<i->second<<endl;
	}
	return out;
}


}


