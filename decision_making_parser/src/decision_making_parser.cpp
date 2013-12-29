/*
 * main.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: dan
 */



#include "Parsers.h"
#include "ParserExceptions.h"
#include <sstream>

using namespace std;

#define CREATE_PARSERS \
	BTParser* btparser = 0;\
	FSMParser* fsmparser = 0;\
	TAOParser* taoparser = 0;\
	struct GC{\
		BTParser*& btparser;\
		FSMParser*& fsmparser;\
		TAOParser*& taoparser;\
		std::string file;\
		GC(BTParser*& btparser, FSMParser*& fsmparser, TAOParser*& taoparser, std::string file):\
			btparser(btparser), fsmparser(fsmparser), taoparser(taoparser), file(file)\
		{\
			btparser = createBT(file);\
			fsmparser = createFSM(file);\
			taoparser = createTAO(file);\
		}\
		~GC(){\
			del(btparser);\
			del(fsmparser);\
			del(taoparser);\
		}\
	} gc(btparser, fsmparser, taoparser, file);

bool parseToXml(std::ostream& parsing_result, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);
		bt_constructor::BTConstructor& bt = parseBT(btparser);
		tao_constructor::TAOConstructor& tao = parseTAO(taoparser);

		//set cross links from different types of parses
		fsm.trees = &bt;
		bt.fsms = &fsm;

		xml_version(parsing_result, ""); parsing_result<<std::endl;
		fsm_constructor::saveXml(parsing_result, fsm)<<std::endl;
		bt_constructor::saveXml(parsing_result, bt)<<std::endl;
		tao_constructor::saveXml(parsing_result, tao)<<std::endl;

		errors << fsm.errors.str();
		errors << bt.errors.str();
		errors << tao.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}
bool parseToXml(std::string result_prefix, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);
		bt_constructor::BTConstructor& bt = parseBT(btparser);
		tao_constructor::TAOConstructor& tao = parseTAO(taoparser);

		//set cross links from different types of parses
		fsm.trees = &bt;
		bt.fsms = &fsm;

		fsm_constructor::saveXml(result_prefix, fsm);
		bt_constructor::saveXml(result_prefix, bt);
		tao_constructor::saveXml(result_prefix, tao);

		errors << fsm.errors.str();
		errors << bt.errors.str();
		errors << tao.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}

bool parseToDot(std::ostream& parsing_result, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);
		bt_constructor::BTConstructor& bt = parseBT(btparser);

		//set cross links from different types of parses
		fsm.trees = &bt;
		bt.fsms = &fsm;

		fsm_constructor::saveDot(parsing_result, fsm)<<std::endl;
		bt_constructor::saveDot(parsing_result, bt)<<std::endl;

		errors << fsm.errors.str();
		errors << bt.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}
bool parseToDot(std::string result_prefix, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);
		bt_constructor::BTConstructor& bt = parseBT(btparser);

		//set cross links from different types of parses
		fsm.trees = &bt;
		bt.fsms = &fsm;

		fsm_constructor::saveDot(result_prefix, fsm);
		bt_constructor::saveDot(result_prefix, bt);

		errors << fsm.errors.str();
		errors << bt.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}


vector<string> split(string text, char del=':'){
	stringstream in(text);
	vector<string> res;
	char ch;
	while(in.eof()==false){
		in >> ch;
		stringstream word;
		while(in.eof()==false and ch!=del){
			word << ch;
			in >> ch;
		}
		if(word.str().size()>0)
			res.push_back(word.str());
	}
	return res;
}

#define SearchFlag(NAME) searchFlag(argc, argv, NAME)
int searchFlag(int argc, char** argv, string name){
	for(int i=0;i<argc;i++){
		string v(argv[i]);
		if( v == name ) return i;
	}
	return -1;
}
#define SearchValue(NAME) searchValue(argc, argv, NAME)
int searchValue(int argc, char** argv, string name){
	int i = searchFlag(argc, argv, name);
	if(i<0 or i==argc-1) return -1;
	return i+1;
}

int main(int argc, char** argv){

	cout<<"-- Start decision making parsing"<<endl;
	string exe ( argv[0] );
	string print_mode = "PRINT_ERRORS";
	if(SearchFlag("-pe")>0) print_mode = "PRINT_ERRORS";
	if(SearchFlag("-pa")>0) print_mode = "PRINT_ALL";
	bool isParseDots = false;
	bool isParseXmls = false;
	if(SearchFlag("-dot")>0) isParseDots=true;
	if(SearchFlag("-xml")>0) isParseXmls=true;
	string project="";
	if(SearchValue("-src")>0){
		project = string( argv[SearchValue("-src")] );
	}
	string shared_folder="";
	if(SearchValue("-dst")>0){
		shared_folder = string( argv[SearchValue("-dst")] );
		if(shared_folder[shared_folder.size()-1]!='/'){
			shared_folder+="/";
		}
	}else{
		std::cerr << "Error: destination folder does not found in parameters. (use -dst flag)" << endl;
		return 1;
	}
	vector<string> files;
	if(SearchValue("-f")>0){
		files = split(string(argv[SearchValue("-f")]));
	}else{
		std::cerr << "Error: list of files does not found in parameters. (use -f flag and : as delimiter)" << endl;
		return 2;
	}


	cout<<"   -- parser path: "<<exe<<endl;
	cout<<"   -- project folder: "<<project<<endl;
	cout<<"   -- share folder: "<<shared_folder<<endl;
	if(print_mode == "PRINT_ALL")
		cout<<"files"<<endl;
	if(isParseXmls)
		cout<<"    -- parse XMLs"<<endl;
	if(isParseDots)
		cout<<"    -- parse DOTs"<<endl;

	for(size_t i=0;i<files.size();i++){
		std::string input_file = files[i];
		if(print_mode == "PRINT_ALL"){
			cout<<"    "<<i<<") parse "<<input_file<<endl;
			cout<<"       save to "<<shared_folder<<endl;
		}
		if(isParseXmls){
			//cout<<"    -- parse XMLs"<<endl;
			std::stringstream errors;
			parseToXml(shared_folder, errors, input_file);
			if(errors.str().size()>0){
				std::cerr << errors.str() <<std::endl;
			}
		}
		if(isParseDots){
			//cout<<"    -- parse DOTs"<<endl;
			std::stringstream errors;
			parseToDot(shared_folder, errors, input_file);
			if(not isParseXmls){
				if(errors.str().size()>0){
					std::cerr << errors.str() <<std::endl;
				}
			}
		}
	}

	cout<<"-- End decision making parsing"<<endl;
	return 0;
}
