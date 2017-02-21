#include "../include/LaserFrame.h"
#include "../include/ParameterReader.h"

using namespace LaserSlam;

FrameReader::FrameReader ( ParameterReader& para ) {

    InitData ( para );

}

FrameReader::~FrameReader () {
}

void FrameReader::InitData ( ParameterReader& para ) {

    string DataType;


   LaserDataDir = para.GetData<string>( "LaserDataDir" );
   //cout << "!!!!!!!!" << LaserDataDir << endl;
   LaserDataFile.open ( LaserDataDir.c_str() );
    //LaserDataFile.open("./data/data.txt");
   if ( !LaserDataFile ) {
       cerr << "Error : Unable find LaserDataFile !!!!!" << endl;
       return;
   }

   while ( !LaserDataFile.eof() ) {

       //LaserDataFile >> DataType;

       //if ( DataType == "L" ) {

           //DataType.clear();
           //laserscan.push_back( LaserScan() );
           //laserscan[ CurrentIndex ].ranges.resize( 720 );
           //cout << "#################################################"  << endl;
           //for ( int i = 0; i < 720; i ++ ) {
               //LaserDataFile >> laserscan[ CurrentIndex ].ranges[ i ];
               //cout << laserscan[ CurrentIndex ].ranges[ i ] << " ";
           //}
           //cout << endl;
           //CurrentIndex ++;
       //}


       LaserDataFile >> DataType;

       while ( DataType.compare( "L" ) == 0 ) {
           DataType.clear();

           laserscan.push_back( LaserScan() );
           laserscan[ CurrentIndex ].ranges.resize( 720 );
           //cout << "###########################################" << endl;
           for ( int i = 0; i < 720; i ++ ) {
               LaserDataFile >> laserscan[ CurrentIndex ].ranges[ i ];
               //cout << laserscan[ CurrentIndex ].ranges[ i ] << " ";
           }
           //printData(CurrentIndex);
           //cout << endl;
           CurrentIndex ++;
           LaserDataFile >> DataType;
       }


       //if ( !DataType.compare( "L" ) ) {
           //CurrentIndex ++;
       //}
//
//       //if ( !LaserDataFile.good() ) {
//           //cout << "!!before break!!" << endl;
//           //break;
//           //cout << "!!after break!!" << endl;
//       //}
   }

   LaserDataFile.close();

   cout << "一共读取到 " << CurrentIndex  << " LaserScan!!!"<< endl;

}


void FrameReader::printData( int idx ) {
    std::cout << "printData of laser " << idx << std::endl;
    for ( auto iter = laserscan[ idx ].ranges.begin(); iter != laserscan[ idx ].ranges.end(); iter ++) {
        cout << *iter << " ";
    }
    cout << endl;
    cout << "####################" <<  endl;
}
