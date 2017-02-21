#ifndef PARAMETERREADER_H_EIYVULQR
#define PARAMETERREADER_H_EIYVULQR

#include "CommonHeaders.h"

namespace LaserSlam {

    class ParameterReader {

        public:
            ParameterReader ( string filename = "./parameters.txt" ) {
                ifstream fin( filename.c_str() );
                if( !fin )  {
                    fin.open( "../parameters.txt" );
                    if( !fin ) {
                        cerr << "parameter file does not exist." << endl;
                        return ;
                    }
                }
                while ( !fin.eof() ) {
                    string str;
                    getline( fin, str );
                    if ( str[ 0 ] == '#' ) {
                        continue;
                    }
                    int pos = str.find( "#" );
                    if ( pos != -1 ) {
                        str = str.substr( 0, pos );
                    }
                    pos = str.find ( "=" );
                    if ( pos == -1 ) {
                        continue;
                    }
                    string key = str.substr( 0, pos );
                    string value = str.substr( pos + 1, str.length() );
                    data[key] = value;
                    //cout << "Parameter name " << key << " is" << value << endl;

                    if ( !fin.good() ) {
                        break;
                    }
                }
            }

            template <class T>
                T GetData ( const string & key ) const {
                    auto iter = data.find( key );
                    if ( iter == data.end() ) {
                        cerr << "Parameter name " << key << " not found !" << endl;
                        return boost::lexical_cast<T>("");
                    }
                    return boost::lexical_cast<T>( iter->second );
                }

        public:
            std::map<string, string> data;


    };

};

#endif /* end of include guard: PARAMETERREADER_H_EIYVULQR */
