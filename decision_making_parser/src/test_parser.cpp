/*
 * test_parser.cpp
 *
 *  Created on: Nov 24, 2013
 *      Author: dan
 */


#include <iostream>
#include <sstream>
#include <vector>
using namespace std;

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

int main(int argc, char** argv){

	cout<<"Start Parser"<<endl;
	string exe ( argv[0] );
	string project( argv[1] );
	vector<string> files = split(string(argv[2]));

	cout<<"parser path: "<<exe<<endl;
	cout<<"project folder: "<<project<<endl;
	cout<<"files"<<endl;
	for(size_t i=0;i<files.size();i++){
		cout<<"    "<<i<<") "<<files[i]<<endl;
	}

	cout<<"End parser"<<endl;
	return 0;
}




