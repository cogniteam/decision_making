/*
 * TAOConstructor.h
 *
 *  Created on: Dec 18, 2013
 *      Author: blackpc
 */

#ifndef TAOCONSTRUCTOR_H_
#define TAOCONSTRUCTOR_H_

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <map>
#include <set>
#include <deque>

#include "Container.h"

using namespace std;


/*************************************************************************************************
*** XML utilities
**************************************************************************************************/

namespace {

    string xmlTag(string tagName) { return "<" + tagName + " />"; }
    string xmlTagOpen(string tagName) { return "<" + tagName + ">"; }
    string xmlTagClose(string tagName) { return "</" + tagName + ">"; }

    string xmlTag(string tagName, string attrName, string attrValue) {
        return "<" + tagName + " " + attrName + "=\"" + attrValue + "\" />";
    }

    string xmlTag(string tagName, string attrName1, string attrValue1, string attrName2, string attrValue2) {
        return "<" + tagName + " " + attrName1 + "=\"" + attrValue1 + "\" " + attrName2 + "=\"" + attrValue2 + "\" />";
    }

    string xmlTag(string tagName, string attrName1, string attrValue1, string attrName2, string attrValue2, string attrName3, string attrValue3) {
        return "<" + tagName + " " + attrName1 + "=\"" + attrValue1 + "\" " +  attrName2 + "=\"" + attrValue2 + "\" " + attrName3 + "=\"" + attrValue3 + "\" />";
    }

    string xmlTagOpen(string tagName, string attrName, string attrValue) {
        return "<" + tagName + " " + attrName + "=\"" + attrValue + "\">";
    }

    string xmlTagOpen(string tagName, string attrName1, string attrValue1, string attrName2, string attrValue2) {
        return "<" + tagName + " " + attrName1 + "=\"" + attrValue1 + "\" " + attrName2 + "=\"" + attrValue2 + "\">";
    }

    string xmlTagOpen(string tagName, string attrName1, string attrValue1, string attrName2, string attrValue2, string attrName3, string attrValue3) {
        return "<" + tagName + " " + attrName1 + "=\"" + attrValue1 + "\" " + attrName2 + "=\"" + attrValue2 + "\"" + " " + attrName3 + "=\"" + attrValue3 + "\">";
    }

    string xmlCData(string data) {
        return "<![CDATA[" + data + "]]>";
    }

    ostream& tabbedWrite(ostream& stream, int depth, string text) {
        stream << string(depth * 4, ' ') << text << endl;
        return stream;
    }

    class XmlTagScopped
    {
    public:
        XmlTagScopped(ostream& stream, int depth, string tagName,
                      string attrName1 = "", string attrValue1 = "",
                      string attrName2 = "", string attrValue2 = "",
                      string attrName3 = "", string attrValue3 = "")
            : _stream(stream), _depth(depth), _tagName(tagName)
        {
            if (attrName1 == "" && attrName2 == "" && attrName3 == "")
                tabbedWrite(_stream, _depth, xmlTagOpen(_tagName));
            else if (attrName2 == "" && attrName3 == "")
                tabbedWrite(_stream, _depth, xmlTagOpen(_tagName, attrName1, attrValue1));
            else if (attrName3 == "")
                tabbedWrite(_stream, _depth, xmlTagOpen(_tagName, attrName1, attrValue1, attrName2, attrValue2));
            else
                tabbedWrite(_stream, _depth, xmlTagOpen(_tagName, attrName1, attrValue1, attrName2, attrValue2, attrName3, attrValue3));
        }

        ~XmlTagScopped() {
            tabbedWrite(_stream, _depth, xmlTagClose(_tagName));
        }

    private:
        ostream&    _stream;
        int         _depth;
        string      _tagName;
    };

}


namespace tao_constructor {


/*************************************************************************************************
*** Interfaces
**************************************************************************************************/

class IXmlWritable {
public:
    virtual ~IXmlWritable() { }
    virtual void writeXml(ostream& stream, int depth) = 0;
};


class ITAOConstructor {
public:
    ITAOConstructor() { }
    virtual ~ITAOConstructor() { }
    virtual void writeXmlTao(string sourceTao, string targetTao, string parentId, ostream& stream, int depth) = 0;
    virtual string getBehId(string taoName, string behName) = 0;
};


class Element {
public:
    mutable ITAOConstructor* lib;
    mutable string tab;
    mutable string id;
    Element(ITAOConstructor* constructor = 0) : lib(constructor) { }
};


/*************************************************************************************************
*** TAO Elements
**************************************************************************************************/


class Next : public Element, public IXmlWritable {
public:
    string protocol;
    vector<string> next_ops;
    string taoName;

    Next(string parent, string parentTao, ITAOConstructor * constructor = 0)
        : Element(constructor), taoName(parentTao)
    {
        id = parent;
    }

    void writeXml(ostream& stream, int depth) {
        if (protocol == "")
            return;

        XmlTagScopped xml(stream, depth, "tao_next", "id", getId(), "protocol", protocol);

        for (int i = 0; i < next_ops.size(); ++i) {
            tabbedWrite(stream, depth + 1, xmlTag("tao_next_op", "id", getId() + "/" + next_ops[i], "name", lib->getBehId(taoName, next_ops[i])));
        }
    }

    string getId() const {
//        return id + "/next";
        return id + "";
    }
};

class Alloc : public Element, public IXmlWritable {
public:
    string protocol;
    vector<string> roles;
    string taoName;

    Alloc(string parent, string parentTao, ITAOConstructor* constructor = 0)
        : Element(constructor), taoName(parentTao)
    {
        id = parent;
    }

    void writeXml(ostream& stream, int depth) {
        if (protocol == "" && roles.size() == 0)
            return;

        XmlTagScopped xml(stream, depth, "tao_allocate", "id", getId(), "protocol", protocol);

        for (int i = 0; i < roles.size(); ++i) {
            string targetTaoName = roles[i];

            // TODO Add tao id
            lib->writeXmlTao(taoName, targetTaoName, getId(), stream, depth + 1);
        }
    }

    string getId() const {
//        return id + "/alloc";
        return id + "";
    }
};

class Beh : public Element, public IXmlWritable {
public:
    string name;

    string start;
    string stop;

    vector<string> task_calls;

    Alloc alloc;
    Next next;

    string taoName;

    Beh(string parent, string behName, string parentTao, ITAOConstructor* constructor)
        : name(behName),
          Element(constructor), alloc(getId(parent, name), parentTao, constructor),
          next(getId(parent, name), parentTao, constructor), taoName(parentTao) {

        id = parent;
    }

    void writeXml(ostream& stream, int depth) {
        XmlTagScopped xml(stream, depth, "tao_plan", "id", getId(), "name", name);

        {
            XmlTagScopped xml(stream, depth + 1, "tao_start_condition", "id", getId() + "/start" );
            tabbedWrite(stream, depth + 2, xmlCData(start));
        }

        {
            XmlTagScopped xml(stream, depth + 1, "tao_stop_condition", "id", getId() + "/stop");
            tabbedWrite(stream, depth + 2, xmlCData(stop));
        }

        for (size_t i = 0; i < task_calls.size(); i++)
            tabbedWrite(stream, depth + 1, xmlTag("tao_call_task", "id", getId() + "/call_task_" + task_calls[i] , "task", task_calls[i]));

        alloc.writeXml(stream, depth + 1);

        next.writeXml(stream, depth + 1);
    }

    string getId(string parent, string name) {
//        return parent + "/behs/" + name;
        return parent + "/" + name;
    }

    string getId() const {
//        return id + "/behs/" + name;
        return id + "/" + name;
    }
};

class TAO : public Element, public IXmlWritable {
public:

    string name;
    std::string start;
    std::vector<Beh> behs;
    std::deque<Beh> stack;

    TAO(ITAOConstructor* constructor = 0) : Element(constructor) {

    }

    void createBeh(string name = "") {
        stack.push_back(Beh(getId(), name, getName(), lib));
    }

    void add() {
        behs.push_back(currentBeh());
        drop();
    }

    void drop() {
        stack.pop_back();
    }

    Beh& currentBeh() {
        return stack.back();
    }

    virtual string getName() {
        return name;
    }

    virtual void writeXml(ostream& stream, int depth) {
        XmlTagScopped xml(stream, depth, "tao", "id", getId(), "name", name, "start", lib->getBehId( id != "" ? getId() : name, start));
        {
            {
                XmlTagScopped xml(stream, depth + 1, "tao_plans", "id", getId() + "/plans");

                for (int i = 0; i < behs.size(); i++)
                    behs[i].writeXml(stream, depth + 2);
            }
        }
    }

    void updateId(string newId, string newName) {
//        cerr << "Updating name to " << newId << endl;
        name = newName;
        id = newId;

        for (int i = 0; i < this->behs.size(); ++i) {
            this->behs[i].id = getId();
            this->behs[i].taoName = getId();
            this->behs[i].next.id = this->behs[i].getId();
            this->behs[i].next.taoName = getId();
            this->behs[i].alloc.id = this->behs[i].getId();
            this->behs[i].alloc.taoName = getId();
        }

    }

    std::string getId() const {
        return id + "/" + name;
    }
};


class TAOConstructor : public ITAOConstructor {
public:

    typedef map<std::string, TAO> TaosMap;

    TaosMap taos;
    deque<TAO> stack;
    Container* trees;
    stringstream& errors;
    string filename;

    TAOConstructor(std::stringstream& errors, std::string filename)
        : trees(0), errors(errors), filename(filename)
    { }

    void create() {
        stack.push_back(TAO(this));
    }

    void add() {
        taos[currentTao().name] = currentTao();
        drop();
    }

    void drop() {
        stack.pop_back();
    }

    TAO& currentTao() {
        return stack.back();
    }

    virtual string getBehId(string taoName, string behName) {
        if (taos.count(taoName) > 0) {
            for (int i = 0; i < taos[taoName].behs.size(); ++i) {
                if (taos[taoName].behs[i].name == behName)
                    return taos[taoName].behs[i].getId();
            }
        }

        if (subTaos.count(taoName) > 0) {
            for (int i = 0; i < subTaos[taoName].behs.size(); ++i) {
                if (subTaos[taoName].behs[i].name == behName)
                    return subTaos[taoName].behs[i].getId();
            }
        }

        return behName;
    }

    string getOnlyTaoName(string fullName) {
        if (fullName[0] == '/')
            for (int i = fullName.size() - 1; i >= 0; i--)
                if (fullName[i] == '/')
                    return string(&fullName.c_str()[i + 1]);

        return fullName;
    }

    void writeXmlTao(string sourceTao, string targetTao, string parentId, ostream& stream, int depth) {

        if (isLoopReference(getOnlyTaoName(sourceTao), getOnlyTaoName(targetTao))) {
            tabbedWrite(stream, depth, xmlTag("tao_ref", "name", targetTao));
            return;
        }

        if (taos.count(targetTao) > 0) {
            referencedTaos[sourceTao].insert(targetTao);

            TAO tao = taos[targetTao];
            tao.updateId(parentId, targetTao);
            subTaos[tao.getId()] = tao;
            subTaos[tao.getId()].writeXml(stream, depth);
        }
        else
            tabbedWrite(stream, depth, xmlTag("tao_not_found", "name", targetTao));
    }

    void writeXml(ostream& stream) {
        TaosMap::iterator it;

        for(it = taos.begin(); it != taos.end(); ++it) {
            it->second.writeXml(stream, 0);
        }
    }

private:

    map<string, set<string> > referencedTaos;
    TaosMap subTaos;
    bool isLoopReference(string sourceTao, string targetTao) {
        bool result = referencedTaos[targetTao].count(sourceTao) > 0;
        return result;
    }
};

ostream& saveXml(ostream& stream, TAOConstructor& constructor);
void saveXml(string directory, TAOConstructor& constructor);

}  // namespace tao_constructor

#endif /* TAOCONSTRUCTOR_H_ */
