/*
 * Constructors.cpp
 *
 *  Created on: Nov 28, 2013
 *      Author: dan
 */


#include "FSMConstructor.h"
#include "BTConstructor.h"


namespace fsm_constructor{

std::string FSMConstructor::copy(std::string name)const{
	std::stringstream s; s<<fsms.at(name); return s.str();
}

void FSMConstructor::saveXml(std::ostream& out, std::string tab, std::string name, std::string id)const{
	const Fsm& fsm = fsms.at(name);
	fsm.lib = this;
	fsm.tab = tab;
	fsm.id = id;
	fsm_constructor::saveXml(out, fsm);
}

void FSMConstructor::saveDot(std::ostream& out, std::string tab, std::string name, std::string id)const{
	const Fsm& fsm = fsms.at(name);
	fsm.lib = this;
	fsm.tab = tab;
	fsm.id = id;
	fsm_constructor::saveDot(out, fsm);
}

void FSMConstructor::map_ids(std::string name, std::string id)const{
	const Fsm& fsm = fsms.at(name);
	fsm.lib = this;
	fsm.id = id;
	fsm_constructor::map_ids(fsm);
}

}


namespace bt_constructor{

std::string BTConstructor::copy(std::string name)const{
	std::stringstream s; s<<trees.at(name); return s.str();
}

void BTConstructor::saveXml(std::ostream& out, std::string tab, std::string name, std::string id)const{
	const Tree& node = trees.at(name);
	node.lib = this;
	node.tab = tab;
	node.id =id;
	bt_constructor::saveXml(out, node);
}

void BTConstructor::saveDot(std::ostream& out, std::string tab, std::string name, std::string id)const{
	const Tree& node = trees.at(name);
	node.lib = this;
	node.tab = tab;
	node.id =id;
	bt_constructor::saveDot(out, node);
}

void BTConstructor::map_ids(std::string name, std::string id)const{
	const Tree& node = trees.at(name);
	node.lib = this;
	node.id=id;
	bt_constructor::map_ids(node);
}

std::string BTConstructor::search_simple_node(std::string name)const{
	return fsm_constructor::searchSimpleState(this->fsms, name);
}

}
