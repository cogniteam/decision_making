/*
 * Filename: Parser_TAO.cpp
 *   Author: Igor Makhtes
 *     Date: Dec 18, 2013
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Cogniteam Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <iostream>
#include <fstream>
#include <deque>

#include "Tokenizer.h"
#include "TAOConstructor.h"
#include "ParserExceptions.h"

using namespace tao_constructor;

#define PRINT(X) //cout<<"parser: "<<X<<endl

class TAOParser {
public:


    /*************************************************************************************************
    *** Tokens
    **************************************************************************************************/

    enum TokenType{
        tkn_null=0,
        tkn_text=1,

        tkn_tao,
            tkn_tao_behs,
            tkn_tao_start_beh,
            tkn_tao_behs_bgn,
                tkn_tao_beh,
                    tkn_tao_start,
                    tkn_tao_call_task,
                    tkn_tao_alloc,
                        tkn_tao_role,
                    tkn_tao_stop,
                    tkn_tao_next,
                        tkn_tao_next_op,
                tkn_tao_behs_end,

        tkn_bopen,
        tkn_bclose,
        tkn_fopen,
        tkn_fclose,
        tkn_next,
        tkn_semicol,
        tkn_col,
        tkn_slesh,
    };

    typedef Parser::TokenStream<TokenType> tstream;
    typedef Parser::Token<TokenType> Token;

    struct TokenizerInit{
        void init(Parser::TokenizerData<TokenType>& tkn){
            tkn.string_token["TAO"]=tkn_tao;
            tkn.string_token["TAO_PLANS"]=tkn_tao_behs;
            tkn.string_token["TAO_START_PLAN"]=tkn_tao_start_beh;
            tkn.string_token["TAO_BGN"]=tkn_tao_behs_bgn;
            tkn.string_token["TAO_PLAN"]=tkn_tao_beh;
            tkn.string_token["TAO_START_CONDITION"]=tkn_tao_start;
            tkn.string_token["TAO_CALL_TASK"]=tkn_tao_call_task;
            tkn.string_token["TAO_ALLOCATE"]=tkn_tao_alloc;
            tkn.string_token["TAO_SUBPLAN"]=tkn_tao_role;
            tkn.string_token["TAO_STOP_CONDITION"]=tkn_tao_stop;
            tkn.string_token["TAO_NEXT"]=tkn_tao_next;
            tkn.string_token["TAO_NEXT_PLAN"]=tkn_tao_next_op;
            tkn.string_token["TAO_END"]=tkn_tao_behs_end;

            tkn.spec_token['(']=tkn_bopen;
            tkn.spec_token[')']=tkn_bclose;
            tkn.spec_token['{']=tkn_fopen;
            tkn.spec_token['}']=tkn_fclose;
            tkn.spec_token[',']=tkn_col;
            tkn.spec_token[';']=tkn_semicol;
            tkn.spec_token['/']=tkn_slesh;
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

    struct TokenizerContext {
        Parser::Tokenizer<TokenType, TokenizerInit> tokenizer;
        std::stringstream ss_fullText;
        stringstream buffer;
        size_t start_index, index;
        ifstream& file;
        char c;

        TokenizerContext(ifstream& file)
            : start_index(0), index(0), file(file), c(0)
        { }
    };

    bool skipComments( TokenizerContext& ctx ) {
        if(ctx.c=='/' and ctx.file.eof()==false){
            char cc;
            ctx.file.read(&cc, 1); ctx.ss_fullText<<cc;
            if(cc=='/'){
                ctx.index++;
                while(cc!='\n' and ctx.file.eof()==false){
                    ctx.file.read(&cc, 1);  saveReadedChar(ctx, cc); ctx.index++;
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

    void saveReadedChar(TokenizerContext& ctx , char c){
        ctx.ss_fullText<<c;
        if(c=='\n') lines.push_back(ctx.index);
    }

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

    string translateToken(TokenType tokenType) {

        for (map<string, TokenType>::iterator it = _stringTokens.begin();
                it != _stringTokens.end();
                ++it)
        {
            if (it->second == tokenType)
                return it->first;
        }

        for (map<char, TokenType>::iterator it = _specTokens.begin();
                        it != _specTokens.end();
                        ++it)
        {
            if (it->second == tokenType)
                return string(1, it->first);
        }

        return "";
    }

    /*************************************************************************************************
    *** Exceptions
    **************************************************************************************************/

    template <typename ExceptionType>
    void throwException(Token token) {
        int line;
        int position;

        searchLineInfo(token.start, line, position);

        throw ExceptionType("", line, position);
    }

    template <typename ExceptionType>
    void throwException(Token token, string param) {
        int line;
        int position;

        searchLineInfo(token.start, line, position);

        throw ExceptionType(param, line, position);
    }

    template <typename ExceptionType>
    void throwException(Token token, string param1, string param2) {
        int line;
        int position;

        searchLineInfo(token.start, line, position);

        throw ExceptionType(param1, param2, line, position);
    }


    /*************************************************************************************************
    *** Generic parsers
    **************************************************************************************************/

    bool testToken(tstream& stream, TokenType tokenType) {
        return stream.last().type == tokenType;
    }

    string parseText(tstream& stream) {
        return parseToken(stream, tkn_text).text;
    }

    Token parseToken(tstream& stream, TokenType tokenType) {
        if (stream.eof())
            throwException<UnexpectedEndOfFile>(stream.first());

        Token token;
        stream >> token;

//        cout << translateToken(tokenType) << ": " << token.text << endl;

        if (token.type != tokenType)
            throwException<UnexpectedToken>(token, token.text, translateToken(tokenType));

        return token;
    }

    size_t findClosingBracket(tstream& stream, TokenType openToken, TokenType closeToken) {
        size_t foundIndex = tkn_search_close_parent(stream, openToken, closeToken);

        if (foundIndex == size_t(-1))
            throw ClosingBracketNotFound();

        return foundIndex;
    }

    /**
     * Parses content of brackets
     * Notes:
     *       1) Current stream pointer must point to opening bracket
     *       2) The method will also fetch the closing bracket
     * @param stream
     * @return
     */
    string parseBracketContent(tstream& stream) {
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
            throwException<UnexpectedToken>(tkn, tkn.text, "(' or '{");


        size_t foundIndex = findClosingBracket(stream, openingBracket, closingBracket);

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

        return resultString;
    }

    bool skipTo(tstream& stream, TokenType targetToken, bool throwOnNotFound = true) {
        Token tkn;

        while (!stream.eof()) {

            if (stream.first().type == targetToken)
                return true;

            stream >> tkn;
        }

        if (throwOnNotFound)
            throw UnexpectedEndOfFile(translateToken(targetToken));

        return false;
    }

    /**
     * Parses a simple macro call of a form MACRO_NAME(PARAMETER) and returns PARAMETER as plain text
     * Note:
     *      1) Current stream position must point to the MACRO_NAME token
     * @param stream
     * @param expectedToken
     * @return
     */
    string parseMacroCall(tstream& stream, TokenType expectedToken) {
        parseToken(stream, expectedToken);
        return parseBracketContent(stream);
    }

    /*************************************************************************************************
    *** Element parsers
    **************************************************************************************************/

    void parseTaoStartBeh(tstream& stream) {
        string startBehName = parseMacroCall(stream, tkn_tao_start_beh);
        constructor.currentTao().start = startBehName;
    }

    void parseStart(tstream& stream) {
        string startBehConditions = parseMacroCall(stream, tkn_tao_start);
        constructor.currentTao().currentBeh().start = startBehConditions;
    }

    void parseCallTask(tstream& stream) {
        string callTaskName = parseMacroCall(stream, tkn_tao_call_task);
        constructor.currentTao().currentBeh().task_calls.push_back(callTaskName);
    }

    void parseRole(tstream& stream) {
        string taoRole = parseMacroCall(stream, tkn_tao_role);
        constructor.currentTao().currentBeh().alloc.roles.push_back(taoRole);
    }

    void parseAlloc(tstream& stream) {
        parseToken(stream, tkn_tao_alloc);
        string protocol = parseBracketContent(stream);
        constructor.currentTao().currentBeh().alloc.protocol = protocol;
        parseToken(stream, tkn_fopen);

        while (!stream.eof()) {
            Token tkn = stream.first();

            switch(tkn.type) {
                case tkn_tao_role:
                    parseRole(stream);
                    break;
                case tkn_fclose:
                    stream >> tkn;
                    return;
                default:
                    stream >> tkn;
                    break;
            }
        }
    }

    void parseStop(tstream& stream) {
        string stopCondition = parseMacroCall(stream, tkn_tao_stop);
        constructor.currentTao().currentBeh().stop = stopCondition;
    }

    void parseNextOp(tstream& stream) {
        string nextOp = parseMacroCall(stream, tkn_tao_next_op);
        constructor.currentTao().currentBeh().next.next_ops.push_back(nextOp);
    }

    void parseNext(tstream& stream) {
        string protocol = parseMacroCall(stream, tkn_tao_next);
        constructor.currentTao().currentBeh().next.protocol = protocol;
        parseToken(stream, tkn_fopen);

        while (!stream.eof()) {
            Token tkn = stream.first();

            switch(tkn.type) {
                case tkn_tao_next_op:
                    parseNextOp(stream);
                    break;
                case tkn_fclose:
                    stream >> tkn;
                    return;
                default:
                    stream >> tkn;
                    break;
            }
        }
    }

    void parseBeh(tstream& stream) {

        parseToken(stream, tkn_tao_beh);
        string behName = parseBracketContent(stream);
        parseToken(stream, tkn_fopen);

        constructor.currentTao().createBeh(behName);

        size_t endOfBeh = findClosingBracket(stream, tkn_fopen, tkn_fclose);

        while (!stream.eof()) {
            Token tkn = stream.first();

            if (endOfBeh == stream.i) {
                constructor.currentTao().add();
                return;
            }

            switch(tkn.type) {
                case tkn_tao_start:
                    parseStart(stream);
                    break;
                case tkn_tao_call_task:
                    parseCallTask(stream);
                    break;
                case tkn_tao_alloc:
                    parseAlloc(stream);
                    break;
                case tkn_tao_stop:
                    parseStop(stream);
                    break;
                case tkn_tao_next:
                    parseNext(stream);
                    break;
                default:
                    stream >> tkn;
                    break;
            }
        }

        constructor.currentTao().drop();
    }

    void parseTaoBehs(tstream& stream) {
        parseToken(stream, tkn_fopen);

        while (!stream.eof()) {
            Token tkn = stream.first();

            switch(tkn.type) {
                case tkn_tao_beh:
                    parseBeh(stream);
                    break;
                case tkn_fclose:
                    stream >> tkn;
                    return;
                default:
                    stream >> tkn;
                    break;
            }
        }
    }

    void parseTaoBehsBgn(tstream& stream) {
        parseToken(stream, tkn_tao_behs_bgn);
        parseTaoBehs(stream);
    }

    void parseTaoBehsDeclaration(tstream& stream) {
        // Nothing to do with declarations?
        parseToken(stream, tkn_tao_behs);
        string behs = parseBracketContent(stream);
    }

    void parseTao(tstream& stream){

        _curr_tao_name = parseMacroCall(stream, tkn_tao);
        parseToken(stream, tkn_fopen);

        constructor.currentTao().name = _curr_tao_name;

        skipTo(stream, tkn_tao_behs, true);
        parseTaoBehsDeclaration(stream);

        skipTo(stream, tkn_tao_start_beh, true);
        parseTaoStartBeh(stream);

        skipTo(stream, tkn_tao_behs_bgn, true);
        parseTaoBehsBgn(stream);

    }

    string filename;

    void parse(tstream& stream) {
        try {
            constructor.create();
            parseTao(stream);
            constructor.add();

        } catch (ParserException& e) {
            constructor.drop();
            errors << "Exception occured in file '" << filename << "'" << endl;
            errors << e.what() << endl;
        }
    }

    /*************************************************************************************************
    *** Initialization
    **************************************************************************************************/


    std::string fullText;
    tstream tokens;
    TAOConstructor constructor;
    std::stringstream errors;
    std::vector<int> lines;
    Token _curr_tao;
    std::string _curr_tao_name;
    Token _curr_state;
    std::string _curr_state_name;

    map<string, TokenType> _stringTokens;
    map<char, TokenType> _specTokens;

    TAOParser(string file):filename(file), constructor(errors, file){}

    TAOConstructor&  main()
    {
        PRINT(__TIME__);
        if(true){
            PRINT("Read input file");
            ifstream file(filename.c_str());
            TokenizerContext ctx(file);

            _stringTokens = ctx.tokenizer.string_token;
            _specTokens = ctx.tokenizer.spec_token;

            if(ctx.file.is_open()){
                PRINT("File ("<<filename<<") is open");

                while(ctx.file.eof()==false){
                    ctx.file.read(&ctx.c, 1);
                    saveReadedChar(ctx, ctx.c);

                    if (skipComments(ctx))
                        continue;

                    ctx.tokenizer.searchToken(ctx.index++, ctx.buffer, ctx.start_index, ctx.c, tokens);
                }

                PRINT("End of file");
                fullText = ctx.ss_fullText.str();
                file.close();
            }else{
                throw PEFileNotFound(filename);
            }
        }

        if(true){
            while(not tokens.eof()){
                if (skipTo(tokens, tkn_tao, false))
                    parse(tokens);
            }

            //PRINT(constructor);
        }
        return constructor;
    }

};

TAOParser* createTAO(string filename) {
    return new TAOParser(filename);
}

TAOConstructor& parseTAO(TAOParser* p) {
    return p->main();
}

void del(TAOParser* tao) {
    delete tao;
}
