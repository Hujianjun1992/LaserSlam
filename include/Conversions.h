#ifndef CONVERSIONS_H_2LYEPUNT
#define CONVERSIONS_H_2LYEPUNT

#include "CommonHeaders.h"
#include "LaserFrame.h"

namespace LaserSlam {

    static Scan *LaserToScan ( LaserSlam::LaserScan& s  ) {

        s.config.min_angle = -1.5708;
        s.config.min_range = 0.023;
        s.config.max_range= 39.0;
        s.config.ang_increment = 0.00436332;

        Scan *scan = new Scan();
        double angle = s.config.min_angle;
        for ( auto iter = s.ranges.begin(); iter != s.ranges.end(); iter ++ ) {

            //std::cout << s.config.min_range << "  " << s.config.max_range << std::endl;

            if ( *iter > s.config.min_range && *iter < s.config.max_range) {
                //std::cout << "hujianjun" << std::endl;
                scan->push_back( Point::Polar( *iter, angle ) );
                //cout << "range is " << *iter << "   angle is " << angle << " ";
                //std::cout << Point::Polar( *iter, angle ) << " ";
            }
            //cout << endl;
            angle += s.config.ang_increment;
            //cout << "range is " << *iter << "   angle is " << angle << " ";

        }
        //cout << "################" << endl;
        //std::cout << *scan << std::endl;

        return scan;


    }

};


#endif /* end of include guard: CONVERSIONS_H_2LYEPUNT */
