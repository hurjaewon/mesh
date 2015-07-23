#ifndef CORRELATION_MATRIX_H
#define CORRELATION_MATRIX_H

//#include <stdint.h>
namespace ns3 {

std::complex<double> InterferenceHelper::RTx [4][4] = 
{
    /*{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) },
		{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) },
		{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) },
		{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) }*/
    	{std::complex<double>(0.79763,0),std::complex<double>(0.54534,0),std::complex<double>(0.2576,0),std::complex<double>(0.0060189,0)},
{std::complex<double>(0.54534,0),std::complex<double>(0.6191,0),std::complex<double>(0.50295,0),std::complex<double>(0.2576,0)},
{std::complex<double>(0.2576,0),std::complex<double>(0.50295,0),std::complex<double>(0.6191,0),std::complex<double>(0.54534,0)},
{std::complex<double>(0.0060189,0),std::complex<double>(0.2576,0),std::complex<double>(0.54534,0),std::complex<double>(0.79763,0)}
};

std::complex<double> InterferenceHelper::RRx [4][4] = 
{
/*{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) },
		{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) },
		{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) },
		{std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0),std::complex<double>(1,0) }*/

    	{std::complex<double>(0.94841,0),std::complex<double>(0.19085,0),std::complex<double>(-0.24253,0),std::complex<double>(0.072621,0)},
{std::complex<double>(0.19085,0),std::complex<double>(0.92854,0),std::complex<double>(0.20631,0),std::complex<double>(-0.24253,0)},
{std::complex<double>(-0.24253,0),std::complex<double>(0.20631,0),std::complex<double>(0.92854,0),std::complex<double>(0.19085,0)},
{std::complex<double>(0.072621,0),std::complex<double>(-0.24253,0),std::complex<double>(0.19085,0),std::complex<double>(0.94841,0)}
};


}

#endif /* CORRELATION_MATRIX_H */
