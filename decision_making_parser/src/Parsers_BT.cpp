// http://www.compileonline.com/compile_cpp0x_online.php
#include <iostream>
#include <fstream>
#include <deque>

#include "Tokenizer.h"
#include "BTConstructor.h"
#include "ParserExceptions.h"

using bt_constructor::BTConstructor;

template<class A, class B>
std::string operator+(const A& s, const B& t){
	std::stringstream ss; ss<<s<<t;
	return ss.str();
}

#define PRINT(X) //cout<<"parser: "<<X<<endl

class BTParser {
public:
	enum TokenType{
		tkn_null=0,
		tkn_text=1,

		tkn_bopen,
		tkn_bclose,
		tkn_fopen,
		tkn_fclose,
		tkn_semicol,
		tkn_col,

		tkn_bt_bgn,
		tkn_bt_root_bgn,
		tkn_par_bgn,
		tkn_seq_bgn,
		tkn_sel_bgn,
		tkn_task_bgn,

		tkn_dec_not_bgn,
		tkn_dec_success_bgn,
		tkn_dec_fail_bgn,

		tkn_bt_end,
		tkn_par_end,
		tkn_seq_end,
		tkn_sel_end,
		tkn_task_end,

		tkn_dec_not_end,
        tkn_dec_success_end,
        tkn_dec_fail_end,

        tkn_set_task_result,
        tkn_set_task_result_after,

		tkn_call_bt,
		tkn_call_task,
		tkn_call_fsm,
	};


	//============= TOKENAZER ===============================

	struct TokenizerInit{
		void init(Parser::TokenizerData<TokenType>& tkn){
			tkn.string_token["BT_BGN"]=tkn_bt_bgn;
			tkn.string_token["BT_PAR_BGN"]=tkn_par_bgn;
			tkn.string_token["BT_SEQ_BGN"]=tkn_seq_bgn;
			tkn.string_token["BT_SEL_BGN"]=tkn_sel_bgn;
			tkn.string_token["BT_ROOT_BGN"]=tkn_bt_root_bgn;
			tkn.string_token["BT_TASK_BGN"]=tkn_task_bgn;

			tkn.string_token["BT_DEC_NOT_BGN"]=tkn_dec_not_bgn;
			tkn.string_token["BT_DEC_SUCCESS_BGN"]=tkn_dec_success_bgn;
			tkn.string_token["BT_DEC_FAIL_BGN"]=tkn_dec_fail_bgn;

			tkn.string_token["BT_END"]=tkn_bt_end;
			tkn.string_token["BT_PAR_END"]=tkn_par_end;
			tkn.string_token["BT_SEQ_END"]=tkn_seq_end;
			tkn.string_token["BT_SEL_END"]=tkn_sel_end;
			tkn.string_token["BT_TASK_END"]=tkn_task_end;

			tkn.string_token["BT_DEC_NOT_END"]=tkn_dec_not_end;
            tkn.string_token["BT_DEC_SUCCESS_END"]=tkn_dec_success_end;
            tkn.string_token["BT_DEC_FAIL_END"]=tkn_dec_fail_end;

            tkn.string_token["BT_SET_TASK_RESULT"]=tkn_set_task_result;
            tkn.string_token["BT_SET_TASK_RESULT_AFTER"]=tkn_set_task_result_after;

			tkn.string_token["BT_CALL_FSM"]=tkn_call_fsm;
			tkn.string_token["BT_CALL_BT"]=tkn_call_bt;
			tkn.string_token["BT_CALL_TASK"]=tkn_call_task;

			tkn.spec_token['(']=tkn_bopen;
			tkn.spec_token[')']=tkn_bclose;
			tkn.spec_token['{']=tkn_fopen;
			tkn.spec_token['}']=tkn_fclose;
			tkn.spec_token[',']=tkn_col;
			//tkn.spec_token[';']=tkn_semicol;
		}
		bool isTokenSumbol(char c){
			return
				('0'<=c and c<='9') or
				('a'<=c and c<='z') or
				('A'<=c and c<='Z') or
				(c=='_')
			;
		}
		bool isDelimiter(char c){
			return not isTokenSumbol(c);
		}
		bool isFlowControl(char c){
			return c==10 or c==13;
		}
	};

	//============= LEXEM PARSER =================================

	typedef Parser::TokenStream<TokenType> tstream;
	typedef Parser::Token<TokenType> Token;

	bool name(tstream& stream, string txt, string& output, int& line, int& pos){
		Token tkn;
		TKN_NEXT( tkn_bopen )
		TKN_NEXT( tkn_text )
		PRINT( txt<<" name is "<<tkn.text );
		output=tkn.text;
		searchLineInfo(tkn.start, line, pos);
		TKN_NEXT( tkn_bclose )
		return true;
	}

	size_t findClosingBracket(tstream& stream, TokenType openToken, TokenType closeToken) {
        size_t foundIndex = tkn_search_close_parent(stream, openToken, closeToken);

//        if (foundIndex == size_t(-1))
//            throw ClosingBracketNotFound();

        return foundIndex;
    }

	bool parse_bracket_content(tstream& stream, string& outputText) {
        Token tkn;
        TokenType openingBracket;
        TokenType closingBracket;
        stream >> tkn;

        if (tkn.type == tkn_bopen) {
            openingBracket = tkn.type;
            closingBracket = tkn_bclose;
        } else if (tkn.type == tkn_fopen) {
            openingBracket = tkn.type;
            closingBracket = tkn_fclose;
        } else
            return false;


        size_t foundIndex = findClosingBracket(stream, openingBracket, closingBracket);

        if (foundIndex == size_t(-1))
            return false;

        string resultString;
        long startPosition = -1;
        long endPosition = -1;

        while (foundIndex > stream.i + 1) {
            stream >> tkn;
            resultString += tkn.text;

            if (startPosition < 0)
                startPosition = tkn.start;
        }

        endPosition = tkn.end;

        // Closing bracket
        stream >> tkn;

        if (startPosition >= 0 and endPosition > startPosition)
            resultString = fullText.substr(startPosition, endPosition - startPosition);

        outputText = resultString;
        return true;
    }

	bool name(tstream& stream, string txt, string& output){
		int line, pos;
		return name(stream, txt, output, line, pos);
	}

	bool root_name(tstream& stream, string txt, string& output){
		Token tkn;
		TKN_NEXT( tkn_bopen )
		TKN_NEXT( tkn_text )
		PRINT( txt<<" name is "<<tkn.text );
		output=tkn.text;
		while(stream.eof()==false and tkn.type!=tkn_bclose ){
			stream >> tkn;
		}
		return true;
	}

	std::deque<Token> tknStack;
	TokenType endOfTask(TokenType bgn){
		switch(bgn){
		case tkn_par_bgn: return tkn_par_end;
		case tkn_seq_bgn: return tkn_seq_end;
		case tkn_sel_bgn: return tkn_sel_end;
		case tkn_task_bgn: return tkn_task_end;
		case tkn_bt_bgn: return tkn_bt_end;
		case tkn_bt_root_bgn: return tkn_bt_end;

		/**
		 * Decorators
		 */
		case tkn_dec_not_bgn: return tkn_dec_not_end;
		case tkn_dec_success_bgn: return tkn_dec_success_end;
		case tkn_dec_fail_bgn: return tkn_dec_fail_end;

		default: return tkn_null;
		}
	}

	bool bt_node_body(tstream& stream, Token& tkn){
		tknStack.push_back(tkn);
		TokenType endTkn = endOfTask(tkn.type);
		size_t end = TKN_SEARCH_CLOSE_PARENT(end, stream, tkn.type, endTkn, "task def");
		size_t cend = stream.getEnd();
		stream.setEnd(end-1);
		TKN_SEARCH( bt_node(stream) );
		stream.setEnd(cend);
		return true;
	}

	string node_type_str(TokenType t){
		switch(t){
		case tkn_par_bgn: return "par";
		case tkn_sel_bgn: return "sel";
		case tkn_seq_bgn: return "seq";
		case tkn_task_bgn: return "task";

		case tkn_dec_not_bgn: return "dec";
		case tkn_dec_success_bgn: return "dec";
		case tkn_dec_fail_bgn: return "dec";

		default: return "?";
		}
	}

	bool bt_node(tstream& stream){
		Token tkn;
		while(not stream.eof()){
			stream >> tkn;

			/**
			 *
			 */
			if(
				tkn.type == tkn_par_bgn or
				tkn.type == tkn_seq_bgn or
				tkn.type == tkn_sel_bgn or
				tkn.type == tkn_task_bgn or
				tkn.type == tkn_dec_fail_bgn
			){
				constructor.tree().create_node();
				constructor.tree().node().type = node_type_str(tkn.type);
				TKN_SEARCH( name(stream, "node "+tkn.type+" start", constructor.tree().node().name) )

				if (tkn.type == tkn_dec_fail_bgn) {
				    constructor.tree().node().name = "Fail";
				    constructor.tree().node().decorator_name = "Fail [" + constructor.tree().node().name + "]";
				}

				if( bt_node_body(stream, tkn) ){
					constructor.tree().add_node();
				}else{
					constructor.tree().drop_node();
					return false;
				}
			}

			if(
                tkn.type == tkn_dec_not_bgn or
                tkn.type == tkn_dec_success_bgn
            ){

			    constructor.tree().create_node();
                constructor.tree().node().type = node_type_str(tkn.type);
                constructor.tree().node().name = tkn.type == tkn_dec_not_bgn ? "Not" : "Success";

                if( bt_node_body(stream, tkn) ){
                    constructor.tree().add_node();
                }else{
                    constructor.tree().drop_node();
                    return false;
                }
            }

			if(
				tkn.type == tkn_par_end or
				tkn.type == tkn_seq_end or
				tkn.type == tkn_sel_end or
				tkn.type == tkn_task_end
			){
				string node_name;

				TKN_SEARCH( name(stream, "node "+tkn+" end", node_name) )
				tknStack.pop_back();
			}

			/**
			 * Decorators
			 */
            if(
                tkn.type == tkn_dec_not_end or
                tkn.type == tkn_dec_success_end or
                tkn.type == tkn_dec_fail_end
            ){
                tknStack.pop_back();
            }

			if( tkn.type == tkn_call_fsm ){
				string node_name; int line, pos;
				TKN_SEARCH( name(stream, "node "+tkn+"", node_name, line, pos) )
				constructor.tree().node().create_call();
				constructor.tree().node().call().type = "fsm";
				constructor.tree().node().call().name = node_name;
				constructor.tree().node().call().file = filename;
				constructor.tree().node().call().line = line;
				constructor.tree().node().call().pos = pos;
				constructor.tree().node().add_call();
			}
			if( tkn.type == tkn_call_bt ){
				string node_name; int line, pos;
				TKN_SEARCH( name(stream, "node "+tkn+"", node_name, line, pos) )
				constructor.tree().node().create_call();
				constructor.tree().node().call().type = "bt";
				constructor.tree().node().call().name = node_name;
				constructor.tree().node().call().file = filename;
				constructor.tree().node().call().line = line;
				constructor.tree().node().call().pos = pos;
				constructor.tree().node().add_call();
			}
			if( tkn.type == tkn_call_task ){
				string node_name; int line, pos;
				TKN_SEARCH( name(stream, "node "+tkn+"", node_name, line, pos) )
				constructor.tree().node().create_call();
				constructor.tree().node().call().type = "rtask";
				constructor.tree().node().call().name = node_name;
				constructor.tree().node().call().file = filename;
				constructor.tree().node().call().line = line;
				constructor.tree().node().call().pos = pos;
				constructor.tree().node().add_call();
			}

			if( tkn.type == tkn_set_task_result ){
                string node_name; int line, pos;
                TKN_SEARCH( parse_bracket_content(stream, node_name) )
                constructor.tree().node().create_call();
                constructor.tree().node().call().type = "task_result";
                constructor.tree().node().call().name = "TaskResult";
                constructor.tree().node().call().task_result = node_name;
                constructor.tree().node().call().file = filename;
                constructor.tree().node().call().line = line;
                constructor.tree().node().call().pos = pos;
                constructor.tree().node().add_call();
            }
			if( tkn.type == tkn_set_task_result_after ){
                string node_name; int line, pos;
                TKN_SEARCH( parse_bracket_content(stream, node_name) )
                constructor.tree().node().create_call();
                constructor.tree().node().call().type = "task_result_after";
                constructor.tree().node().call().name = "TaskResultAfter";
                constructor.tree().node().call().task_result = node_name;
                constructor.tree().node().call().file = filename;
                constructor.tree().node().call().line = line;
                constructor.tree().node().call().pos = pos;
                constructor.tree().node().add_call();
            }

        }

		return true;
	}


	bool bt_body(tstream& stream, Token& tkn){
		TKN_SEARCH( root_name(stream, "BT "+tkn+" start", constructor.tree().name) )
		TokenType endTkn = endOfTask(tkn.type);
		size_t end = TKN_SEARCH_CLOSE_PARENT(end, stream, tkn.type, endTkn, "task def");
		size_t cend = stream.getEnd();
		stream.setEnd(end-1);
		TKN_SEARCH( bt_node(stream) );
		stream.setEnd(cend);
		return true;
	}
	bool bt(tstream& stream){
		Token tkn;
		PRINT("start search from "<<stream.first());
		while(not stream.eof()){
			stream >> tkn;
			if( tkn.type == tkn_bt_bgn or tkn.type == tkn_bt_root_bgn ){
				constructor.create_tree();
				if( bt_body(stream, tkn) ){
					constructor.add_tree();
				}else{
					constructor.drop_tree();
				}
			}
			if(
				tkn.type == tkn_bt_end
			){
				string bt_name;
				TKN_SEARCH( name(stream, "BT "+tkn+" end", bt_name) )
			}
		}
		return false;
	}

	struct TokenizerContext{
		Parser::Tokenizer<TokenType, TokenizerInit> tokenizer;
		std::stringstream ss_fullText;
		stringstream buffer;
		size_t start_index, index;
		ifstream& file;
		char c;
		TokenizerContext(ifstream& file):
		start_index(0),index(0),
		file(file), c(0)
		{}
	};

	void searchLineInfo(int i, int& line, int& pos){
		line=0;
		for(size_t n=0;n<lines.size();n++){
			if(i<lines[n]){
				pos = i-lines[line-2];
				return;
			}
			line = n+2;
		}
	}

	void saveReadedChar(TokenizerContext& ctx , char c){
		ctx.ss_fullText<<c;
		if(c=='\n') lines.push_back(ctx.index);
	}

	bool skipComments( TokenizerContext& ctx )
	{
		if(ctx.c=='/' and ctx.file.eof()==false){
			char cc;
			ctx.file.read(&cc, 1); saveReadedChar(ctx, cc);
			if(cc=='/'){
				ctx.index++;
				while(cc!='\n' and ctx.file.eof()==false){
					ctx.file.read(&cc, 1); saveReadedChar(ctx, cc); ctx.index++;
				}
				ctx.index++;
				return true;
			}
			else{
				ctx.tokenizer.searchToken(ctx.index++, ctx.buffer, ctx.start_index, ctx.c, tokens);
				ctx.c=cc;
			}
		}
		return false;
	}

	BTParser(string file):filename(file), constructor(errors, file){}

	string filename;
	std::string fullText;
	tstream tokens;
	BTConstructor constructor;
	std::stringstream errors;
	std::vector<int> lines;

	BTConstructor& main()
	{
		PRINT(__TIME__);
		if(true){
			PRINT("Read input file");
			ifstream file(filename.c_str());
			TokenizerContext ctx(file);
			if(ctx.file.is_open()){
				PRINT("File ("<<filename<<") is open");
				while(ctx.file.eof()==false){
					ctx.file.read(&ctx.c, 1); saveReadedChar(ctx, ctx.c);
					if(skipComments(ctx)) continue;
					ctx.tokenizer.searchToken(ctx.index++, ctx.buffer, ctx.start_index, ctx.c, tokens);
				}
				PRINT("End of file");
				fullText = ctx.ss_fullText.str();
				file.close();
			}else{
				throw PEFileNotFound(filename);
			}
		}

		if(false){
			//cout<<"Result: ";
			while(not tokens.eof()){
				Token t; tokens>>t;
				PRINT(t);
			}
		}
		if(true){
			while( bt(tokens) );
			//PRINT( constructor );
		}

		return constructor;
	}

};

ostream& operator<<(ostream& out, BTParser::TokenType t){
	#define PRINTTKN(x) case BTParser::tkn_##x: return out<<#x;
	switch(t){
		PRINTTKN(null)
		PRINTTKN(text)

		PRINTTKN(bopen)
		PRINTTKN(bclose)
		PRINTTKN(fopen)
		PRINTTKN(fclose)
		PRINTTKN(semicol)
		PRINTTKN(col)

		PRINTTKN(bt_bgn)
		PRINTTKN(bt_root_bgn)
		PRINTTKN(par_bgn)
		PRINTTKN(seq_bgn)
		PRINTTKN(sel_bgn)
		PRINTTKN(task_bgn)

		PRINTTKN(bt_end)
		PRINTTKN(par_end)
		PRINTTKN(seq_end)
		PRINTTKN(sel_end)
		PRINTTKN(task_end)

		PRINTTKN(call_bt)
		PRINTTKN(call_task)
		PRINTTKN(call_fsm)
	}
	return out<<"???";
	#undef PRINTTKN
}


BTConstructor& parseBT(BTParser* p){
	return p->main();
}

BTParser* createBT(std::string filename){
	return new BTParser(filename);
}

void del(BTParser* p){ delete p; }


#undef PRINT
