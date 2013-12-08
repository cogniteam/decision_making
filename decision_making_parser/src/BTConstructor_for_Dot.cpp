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


bool saveDot_call_node(std::ostream& out, const Node& o){
	string resolved = "";
	if(o.type == "bt"){
		bool isResolved = o.lib->contains_tree(o.name);
		if(isResolved){
			o.lib->saveDot_tree(out, o.tab+"    ", o.name, o.getId()); out<<endl;
			Container::postData()<<o.tab
					<<Container::map_id_to_number()[o.getId()]
					<<" -> "
					<<Container::map_id_to_number()[o.getId()+"/"+o.name]
					<<" // bt_call from "<<o.getId()<<" to "<<(o.getId()+"/"+o.name)<<endl;
		}
		return true;
	}
	if(o.type == "fsm"){
		bool isResolved = o.lib->contains_fsm(o.name);
		if(isResolved){
			string tab = o.tab+"   ";
			o.lib->saveDot_fsm(out, tab+"    ", o.name, o.getId());	out<<endl;
		}
		return true;
	}
	if(o.type == "rtask"){
		out<<o.tab<<Container::map_id_to_number()[o.getId()]<<" [label=\""<<o.name<<"\"]";
		return true;
	}
	return false;
}

std::ostream& saveDot(std::ostream& out, const Node& o){
	if(o.type!="fsm"){
		if(o.type == "bt")
			out<<o.tab<<Container::map_id_to_number()[o.getId()]<<" [label=\"BT["<<o.name<<"]\"]"<<" //node:"<<o.type<<":"<<o.getId()<<endl;
		else
			out<<o.tab<<Container::map_id_to_number()[o.getId()]<<" [label=\""<<o.name<<"\"]"<<" //node:"<<o.type<<":"<<o.getId()<<endl;
	}
	if(saveDot_call_node(out, o)) return out;
	if(o.type == "par")	 ;
	if(o.type == "seq")	 ;
	if(o.type == "sel")	 ;
	if(o.type == "dec")	 ;
	if(o.type == "task") ;

	FOREACH_VEC(Node, nodes){
		out<<endl;
		i->lib = o.lib;
		i->tab = o.tab+"      ";
		i->id = o.getId();
		saveDot(out, *i);
		out<<endl;
		if(i->type!="fsm"){
			Container::postData()<<o.tab<<Container::map_id_to_number()[o.getId()]<<" -> "<<Container::map_id_to_number()[i->getId()]<<" // from "<<o.getId()<<" to "<<i->getId()<<endl;
		}else{
			bool isResolved = o.lib->contains_fsm(i->name);
			string dst = o.lib->search_simple_node(i->name);
			Container::postData()<<o.tab<<Container::map_id_to_number()[o.getId()]<<" -> "<<Container::map_id_to_number()[dst]
			          <<"[ ltail=\""<<Container::map_id_to_number()[o.getId()]<<"\", lhead=\"cluster_"<< Container::map_id_to_number()[i->getId()+"/"+i->name]<<"\" ]"
			          <<" // fsm_call from "<<o.getId()<<" to "<<i->getId()<<endl;
		}
	}
	out<<endl;
	if(o.type == "par") ;
	if(o.type == "seq")	 ;
	if(o.type == "sel")	 ;
	if(o.type == "dec")	 ;
	if(o.type == "task") ;
	return out;
}


std::ostream& saveDot(std::ostream& out, const Tree& o){
	out<<o.tab<<Container::map_id_to_number()[o.getId()]<<" [label=\""<<o.name<<"\"]"<<" //root "<<o.getId()<<endl;
	FOREACH_VEC(Node, nodes){
		out<<endl;
		i->lib = o.lib;
		i->tab = o.tab+"      ";
		i->id = o.getId();
		saveDot(out, *i);
		out<<endl;
		//cout<<"// Edge " <<o.tab<<Container::map_id_to_number()[o.getId()]<<" -> "<<Container::map_id_to_number()[i->getId()]<<" // from "<<o.getId()<<" to "<<i->getId()<<endl;
		Container::postData()<<o.tab<<Container::map_id_to_number()[o.getId()]<<" -> "<<Container::map_id_to_number()[i->getId()]<<" // from "<<o.getId()<<" to "<<i->getId()<<endl;
	}
	out<<endl;
	return out;
}

void printIds(){
//	std::cout<<"\nPrint all ids map (BT): {"<<endl;
//	for(std::map<std::string, std::string>::const_iterator i=Container::map_id_to_number().begin();i!=Container::map_id_to_number().end();i++){
//		std::cout<<"    "<< i->first <<" :=  "<< i->second <<std::endl;
//	}
//	std::cout<<"}"<<std::endl;
}

std::ostream& saveDot(std::ostream& out, const BTConstructor& o){

	FOREACH_MAP(Tree, trees){
		i->second.lib = &o;
		i->second.tab = "      ";
		map_ids(i->second); printIds();
		Container::clear_postData();
		out<<o.tab<<"digraph "<<i->second.name<<" { //bt"<<endl;
		out<<o.tab<<"      compound=\"true\""<<endl;
		saveDot(out, i->second)<<endl;
		out<<o.tab<<"     "<<Container::postData().str()<<endl;
		out<<o.tab<<"}";
	}
	return out;
}


void saveDot(std::string path_prefix, const BTConstructor& o){

	FOREACH_MAP(Tree, trees){
		std::stringstream filename; filename << path_prefix << i->second.name <<DOT_FILE_EXT;
		std::ofstream out(filename.str().c_str());
		if(out.is_open()){
			i->second.lib = &o;
			i->second.tab = "      ";
			map_ids(i->second); printIds();
			Container::clear_postData();
			out<<o.tab<<"digraph "<<i->second.name<<" { //bt"<<endl;
			out<<o.tab<<"      compound=\"true\""<<endl;
			out<<o.tab<<"//from file: "<<o.filename<<" -->"; out<<endl;
			saveDot(out, i->second)<<endl;
			out<<o.tab<<"     "<<Container::postData().str()<<endl;
			out<<o.tab<<"}";
			out.close();
		}else{
			throw PEFileNotCreated(filename.str());
		}
	}
}

void map_ids(const Node& o){
	Container::map_id_to_number()[o.getId()] = Container::get_id_counter();
	FOREACH_VEC(Node, nodes){
		i->lib = o.lib;
		i->id = o.getId();
		if(i->nodes.empty()){
			if(i->type == "bt"){
				bool isResolved = o.lib->contains_tree(i->name);
				if(isResolved){
					o.lib->map_ids_tree(i->name, i->getId());
				}
			}else
			if(i->type == "fsm"){
				bool isResolved = o.lib->contains_fsm(i->name);
				if(isResolved){
					o.lib->map_ids_fsm(i->name, i->getId());
				}
			}//else
			//map_ids(*i);
		}//else
		{
			map_ids(*i);
		}
	}
}
void map_ids(const Tree& o){
	Container::map_id_to_number()[o.getId()] = Container::get_id_counter();
	FOREACH_VEC(Node, nodes){
		i->lib = o.lib;
		i->id = o.getId();
		map_ids(*i);
	}
}
void map_ids(const BTConstructor& o){
	Container::reset_id_counter();
	Container::map_id_to_number().clear();
	FOREACH_MAP(Tree, trees){
		i->second.lib = &o;
		map_ids(i->second);
	}
}

}


