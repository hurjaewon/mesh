/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "interference-helper.h"
#include "wifi-phy.h"
#include "error-rate-model.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <algorithm>
#include "rbir.h" //11ac: mutiple_stream_tx_per
#include "correlation-matrix.h" //11ac: mutiple_stream_tx_per

NS_LOG_COMPONENT_DEFINE ("InterferenceHelper");

namespace ns3 {

/****************************************************************
 *       Phy event class
 ****************************************************************/

InterferenceHelper::Event::Event (uint32_t size, WifiMode payloadMode,
                                  enum WifiPreamble preamble,
                                  Time duration, double rxPower, WifiTxVector txVector)
  : m_size (size),
    m_payloadMode (payloadMode),
    m_preamble (preamble),
    m_startTime (Simulator::Now ()),
    m_endTime (m_startTime + duration),
    m_rxPowerW (rxPower),
    m_txVector (txVector)
{
}
InterferenceHelper::Event::~Event ()
{
}

Time
InterferenceHelper::Event::GetDuration (void) const
{
  return m_endTime - m_startTime;
}
Time
InterferenceHelper::Event::GetStartTime (void) const
{
  return m_startTime;
}
Time
InterferenceHelper::Event::GetEndTime (void) const
{
  return m_endTime;
}
double
InterferenceHelper::Event::GetRxPowerW (void) const
{
  return m_rxPowerW;
}


// skim11 : consider txvector for power calculations
double 
InterferenceHelper::Event::GetRealRxPowerW (void) 
{	
	uint8_t nss = 	m_txVector.GetNss();
	double realRxPowerW = m_rxPowerW;
	if (nss == 1){
		std::complex<double> **ch = new std::complex<double> *[nss];
		for (int i=0;i<nss;i++)
			ch[i] = new std::complex<double>[nss];
		m_txVector.GetChannelMatrix (ch);
		double a = ch[0][0].real();
		double b = ch[0][0].imag();
		double squareValue = (a*a + b*b);
		realRxPowerW = m_rxPowerW*squareValue/2; 
		for(int j=0; j<nss; j++)
		{
			delete [] ch[j];
		}
		delete [] ch; 
	}		
	return realRxPowerW;
//  return m_rxPowerW;
}
uint32_t
InterferenceHelper::Event::GetSize (void) const
{
  return m_size;
}
WifiMode
InterferenceHelper::Event::GetPayloadMode (void) const
{
  return m_payloadMode;
}
enum WifiPreamble
InterferenceHelper::Event::GetPreambleType (void) const
{
  return m_preamble;
}

WifiTxVector
InterferenceHelper::Event::GetTxVector (void) const
{
  return m_txVector;
}


/****************************************************************
 *       Class which records SNIR change events for a
 *       short period of time.
 ****************************************************************/

InterferenceHelper::NiChange::NiChange (Time time, double delta)
  : m_time (time),
    m_delta (delta)
{
}
Time
InterferenceHelper::NiChange::GetTime (void) const
{
  return m_time;
}
double
InterferenceHelper::NiChange::GetDelta (void) const
{
  return m_delta;
}
bool
InterferenceHelper::NiChange::operator < (const InterferenceHelper::NiChange& o) const
{
  return (m_time < o.m_time);
}

/****************************************************************
 *       The actual InterferenceHelper
 ****************************************************************/

InterferenceHelper::InterferenceHelper ()
  : m_errorRateModel (0),
    m_firstPower (0.0),
    m_rxing (false),
    m_PI (3.141592653589793),
    m_antennaCorrelation (0)
{
}
InterferenceHelper::~InterferenceHelper ()
{
  EraseEvents ();
  m_errorRateModel = 0;
}

Ptr<InterferenceHelper::Event>
InterferenceHelper::Add (uint32_t size, WifiMode payloadMode,
                         enum WifiPreamble preamble,
                         Time duration, double rxPowerW, WifiTxVector txVector)
{
  Ptr<InterferenceHelper::Event> event;

  event = Create<InterferenceHelper::Event> (size,
                                             payloadMode,
                                             preamble,
                                             duration,
                                             rxPowerW,
                                             txVector);
  AppendEvent (event);
  return event;
}

//11ac: mutiple_stream_tx_per
void 
InterferenceHelper::SetAntennaCorrelation (bool antennaCorrelation)
{
	m_antennaCorrelation = antennaCorrelation;
}

void
InterferenceHelper::SetNoiseFigure (double value)
{
  m_noiseFigure = value;
}

double
InterferenceHelper::GetNoiseFigure (void) const
{
  return m_noiseFigure;
}

void
InterferenceHelper::SetErrorRateModel (Ptr<ErrorRateModel> rate)
{
  m_errorRateModel = rate;
}

Ptr<ErrorRateModel>
InterferenceHelper::GetErrorRateModel (void) const
{
  return m_errorRateModel;
}

Time
InterferenceHelper::GetEnergyDuration (double energyW)
{
  Time now = Simulator::Now ();
  double noiseInterferenceW = 0.0;
  Time end = now;
  noiseInterferenceW = m_firstPower;
  for (NiChanges::const_iterator i = m_niChanges.begin (); i != m_niChanges.end (); i++)
    {
      noiseInterferenceW += i->GetDelta ();
      end = i->GetTime ();
      if (end < now)
        {
          continue;
        }
      if (noiseInterferenceW < energyW)
        {
          break;
        }
    }
  return end > now ? end - now : MicroSeconds (0);
}

void
InterferenceHelper::AppendEvent (Ptr<InterferenceHelper::Event> event)
{
  Time now = Simulator::Now ();
	NS_LOG_DEBUG ("start=" << event->GetStartTime ().GetMicroSeconds() <<
				"us, end=" <<  event->GetEndTime ().GetMicroSeconds() <<
				"us, power=" << WToDbm(event->GetRealRxPowerW ()) );
  if (!m_rxing)
    {
      NiChanges::iterator nowIterator = GetPosition (now);
      for (NiChanges::iterator i = m_niChanges.begin (); i != nowIterator; i++)
        {
          m_firstPower += i->GetDelta ();
        }
      m_niChanges.erase (m_niChanges.begin (), nowIterator);
      m_niChanges.insert (m_niChanges.begin (), NiChange (event->GetStartTime (), event->GetRealRxPowerW ())); //160413 skim11 : channel bug fix
    }
  else
    {
      AddNiChangeEvent (NiChange (event->GetStartTime (), event->GetRealRxPowerW ()));//160413 skim11 : channel bug fix
    }
  AddNiChangeEvent (NiChange (event->GetEndTime (), -event->GetRealRxPowerW ()));//160413 skim11 : channel bug fix

}


double
InterferenceHelper::CalculateSnr (double signal, double noiseInterference, WifiMode mode) const
{
  // thermal noise at 290K in J/s = W
  static const double BOLTZMANN = 1.3803e-23;
  // Nt is the power of thermal noise in W
  double Nt = BOLTZMANN * 290.0 * 20000000; //mode.GetBandwidth (); // 171102 ywson: shall not be increased according to bandwidth
  // receiver noise Floor (W) which accounts for thermal noise and non-idealities of the receiver
  double noiseFloor = m_noiseFigure * Nt;
  double noise = noiseFloor + noiseInterference;
  double snr = signal / noise;
  return snr;
}

//11ac: mutiple_stream_tx_per
double
InterferenceHelper::CalculateSnr (double signal, double noiseInterference, WifiTxVector txVector)
{
	NS_LOG_DEBUG("signal " << signal << " noise " << noiseInterference << " nss " << (int) txVector.GetNss() << " mode " << txVector.GetMode() << " bw " << (int)txVector.GetBandwidth());
  // thermal noise at 290K in J/s = W
  static const double BOLTZMANN = 1.3803e-23;
  // Nt is the power of thermal noise in W
  double Nt = BOLTZMANN * 290.0 * 20000000;  // 171102 ywson: shall not be increased according to bandwidth
  // receiver noise Floor (W) which accounts for thermal noise and non-idealities of the receiver
  double noiseFloor = m_noiseFigure * Nt;
  double noise = noiseFloor + noiseInterference;
  double snr = signal / noise;
  
  //caudal loss: obtaining channel information
  uint8_t nss = txVector.GetNss();

  if (nss == 1)
  {
    std::complex<double> **ch = new std::complex<double> *[nss];
    for (int i=0;i<nss;i++)
  	  ch[i] = new std::complex<double>[nss];
    txVector.GetChannelMatrix (ch);
    double a = ch[0][0].real();
    double b = ch[0][0].imag();
    double squareValue = (a*a + b*b);
    snr = snr*squareValue/2; 
      for(int j=0; j<nss; j++)
      {
        delete [] ch[j];
      }
      delete [] ch;
    return snr;
  }

  std::complex<double> **ch = new std::complex<double> *[nss];
  for (int i=0;i<nss;i++)
  	{
  	ch[i] = new std::complex<double>[nss];
  	}
  std::complex<double> **mmse = new std::complex<double> *[nss];
  std::complex<double> **temp = new std::complex<double> *[nss];
  for (int i=0;i<nss;i++)
  	{
  	mmse[i] = new std::complex<double>[nss];
	temp[i] = new std::complex<double>[nss];
  	}

  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{	mmse[i][j] = RTx[i][j]; //mmse = RTx
	}

  txVector.GetChannelMatrix(ch);

  //shbyeon bug report : input channel variance 1
  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{	ch[i][j] /= std::sqrt(2);
	}

  MtxMultiplication(ch, mmse, nss, temp); //temp = ch*RTx
  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{	mmse[i][j] = RRx[i][j]; //mmse = RRx
	}

  if (m_antennaCorrelation)
      {MtxMultiplication(mmse, temp, nss, ch); }//ch = RRx*ch*RTx: channel which reflect antenna correlation
  	
  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{	temp[i][j] = ch[i][j];}

  
  Hermitian(temp, nss); //temp = hermitian(ch)
  MtxMultiplication(temp, ch, nss, mmse); //mmse = hermitian(ch)*ch
  
  for(int i=0;i<nss;i++)
      mmse[i][i].real() +=noise*nss /signal; //mmse = hermitian(ch)*ch + N0I

  Inverse(mmse, nss, temp); //temp = inverse(hermitian(ch)*ch + N0I)
  Hermitian(ch, nss); //ch = hermitian(ch)
  MtxMultiplication(temp, ch, nss, mmse); //mmse = MMSE weight matrix
  Hermitian(ch, nss); //ch = ch
  Transpose(ch, nss); //ch = transpose(ch)

  double *sinr = new double[nss];
  double interference;
  double noiseAmplification;
  for (int i=0; i<nss; i++)
  {
      interference = 0;
      noiseAmplification = 0;  
      for (int j=0; j<nss; j++)
      {
         noiseAmplification += std::norm (mmse[i][j]) ;
         if (j!=i)
         {	interference += std::norm(VectorMultiplication(mmse[i],ch[j],nss));}
      	}
	interference += nss*noise*noiseAmplification/signal;
	sinr[i] = std::norm(VectorMultiplication(mmse[i],ch[i],nss)) / interference;	
  }
  
  for(int i=0; i<nss;i++)
  {
  delete [] ch[i];
  delete [] mmse[i];
  delete [] temp[i];
  }
  delete [] ch;
  delete [] mmse;
  delete [] temp;
  
  uint8_t cons_mode;
  switch (txVector.GetMode().GetConstellationSize () ) 
  {
	case 2:cons_mode=0; break;
	case 4:cons_mode=1; break;
	case 16:cons_mode=2; break;
	case 64:cons_mode=3; break;
	default:cons_mode=0; 
  }

  double eesm = 0;
  double esnr ;
  uint8_t snr_idx ;
  double min_sinr = 100;
  for (int i=0; i<nss; i++)
  {
     sinr[i] = 10.0*std::log10(sinr[i]);
     if (sinr[i] < min_sinr)
	 {	min_sinr = sinr[i];}
     if (sinr[i] < -20.0){sinr[i]= -20.0;}
     if (sinr[i] > 27.0){sinr[i]= 27.0;}
     snr_idx = (uint8_t)( (sinr[i]+20)*2 );
     eesm += RBIR[	cons_mode][snr_idx];
     
  }
  eesm /= nss;
  
  delete [] sinr;

  uint8_t current, prev = 0;
  for (int sidx =0; sidx<95; sidx++)
  {
	current = sidx;
	if (eesm <= RBIR[cons_mode][sidx])
	{	break;}
       prev = sidx;
  }
  if ( (RBIR[cons_mode][current]-eesm) > (eesm-RBIR[cons_mode][prev]) )
  	{esnr = prev;}
  else
  	{esnr = current;}
  esnr = esnr/2 - 20;
  if(esnr < min_sinr)
  	{esnr = min_sinr;}


  esnr = std::pow (10.0, esnr/10.0);

  //return snr;
  return esnr;
}

double
InterferenceHelper::CalculateSnr (double signal, double noiseInterference, WifiTxVector txVector, uint16_t subframeIdx)
{
	NS_LOG_DEBUG("signal " << signal << " noise " << noiseInterference << " nss " << (int) txVector.GetNss() << " mode " << txVector.GetMode() << " bw " << (int)txVector.GetBandwidth());
  if(subframeIdx==0 || subframeIdx==1)
    subframeIdx=0;
  else
    subframeIdx=subframeIdx-1;
  // thermal noise at 290K in J/s = W
  static const double BOLTZMANN = 1.3803e-23;
  // Nt is the power of thermal noise in W
  double Nt = BOLTZMANN * 290.0 * 20000000; //txVector.GetMode().GetBandwidth ();  // 171102 ywson: shall not be increased according to bandwidth
  // receiver noise Floor (W) which accounts for thermal noise and non-idealities of the receiver
  double noiseFloor = m_noiseFigure * Nt;
  double noise = noiseFloor + noiseInterference;
  double snr = signal / noise;
  
  //caudal loss: obtaining channel information
  bool caudalOn = txVector.GetCaudalLoss();
  uint8_t nss = txVector.GetNss();

  if (nss == 1)
  { 
    std::complex<double> **ch = new std::complex<double> *[nss];
    std::complex<double> **ch2 = new std::complex<double> *[nss];
    for (int i=0;i<nss;i++)
    {
      ch[i] = new std::complex<double>[nss];
      ch2[i] = new std::complex<double>[nss];
    }
    txVector.GetChannelMatrix(ch);
    txVector.GetChannelMatrix(ch2, subframeIdx);

    double a = ch[0][0].real();
    double b = ch[0][0].imag();
    double squareValue = (a*a + b*b);
    snr = snr*squareValue/2; 
    
    if(caudalOn && subframeIdx > 0)
    {
      double c = ch2[0][0].real();
      double d = ch2[0][0].imag();
      double numer = (squareValue-a*c-b*d)*(squareValue-a*c-b*d)+(a*d-b*c)*(a*d-b*c);
      double noise_mob = numer/squareValue * signal / 2;
      double newSnr = signal * squareValue/2 / (noise + noise_mob) ;
      NS_LOG_DEBUG("org=" << 10*log10(snr) << " " << subframeIdx <<"th MPDU's=" << 10*log10(newSnr)); 
      snr = newSnr;
    }
    else
      NS_LOG_DEBUG("org=" << 10*log10(snr) << " " << subframeIdx <<"th MPDU's=" << 10*log10(snr));
    for(int j=0; j<nss; j++)
    {
      delete [] ch[j];
      delete [] ch2[j];
    }
    delete [] ch;
    delete [] ch2;
    return snr;
  }
  
 

  std::complex<double> **ch = new std::complex<double> *[nss];
  std::complex<double> **ch2 = new std::complex<double> *[nss];
  for (int i=0;i<nss;i++)
  	{
  	ch[i] = new std::complex<double>[nss];
  	ch2[i] = new std::complex<double>[nss];
  	}
  std::complex<double> **mmse = new std::complex<double> *[nss];
  std::complex<double> **temp = new std::complex<double> *[nss];
  for (int i=0;i<nss;i++)
  	{
  	mmse[i] = new std::complex<double>[nss];
	temp[i] = new std::complex<double>[nss];
  	}

  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{	mmse[i][j] = RTx[i][j]; //mmse = RTx
	}

  txVector.GetChannelMatrix(ch);

  //shbyeon bug report : input channel variance 1
  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{	ch[i][j] /= std::sqrt(2);
	}

  MtxMultiplication(ch, mmse, nss, temp); //temp = ch*RTx
  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{ mmse[i][j] = RRx[i][j]; //mmse = RRx
	}

  if (m_antennaCorrelation)
      {MtxMultiplication(mmse, temp, nss, ch); }//ch = RRx*ch*RTx: channel which reflect antenna correlation
  	
  for (int i=0;i<nss;i++)
  	for (int j=0;j<nss;j++)
	{	temp[i][j] = ch[i][j];}

  
  Hermitian(temp, nss); //temp = hermitian(ch)
  MtxMultiplication(temp, ch, nss, mmse); //mmse = hermitian(ch)*ch
  
  for(int i=0;i<nss;i++)
      mmse[i][i].real() +=noise*nss /signal; //mmse = hermitian(ch)*ch + N0I

  Inverse(mmse, nss, temp); //temp = inverse(hermitian(ch)*ch + N0I)
  Hermitian(ch, nss); //ch = hermitian(ch)
  MtxMultiplication(temp, ch, nss, mmse); //mmse = MMSE weight matrix
  Hermitian(ch, nss); //ch = ch
  Transpose(ch, nss); //ch = transpose(ch)

  double noise_mob[nss];
  for (int i=0;i<nss;i++)
    noise_mob[i] = 0;

  if(caudalOn && subframeIdx > 0)
  {
    txVector.GetChannelMatrix(ch2, subframeIdx);

    //shbyeon bug report : input channel variance 1
    for (int i=0;i<nss;i++)
  	  for (int j=0;j<nss;j++)
		  ch2[i][j] /= std::sqrt(2);
	
    Transpose(ch2, nss);

    for(int i=0; i<nss; i++)
    {
//      double sgain = std::norm(VectorMultiplication(mmse[i],ch[i],nss));
      for(int j=0; j<nss; j++)
      {
        std::complex<double> *chdiff = new std::complex<double>[nss];
        for(int l = 0; l<nss; l++){
            chdiff[l] = ch[j][l] - ch2[j][l];
            NS_LOG_DEBUG(subframeIdx << "th frame, channel diff of" << l << " and " << j << " =" << chdiff[l]); 
        }
        double nmob = std::norm(VectorMultiplication(mmse[i],chdiff,nss));
        //double nmob = std::norm(VectorMultiplication(mmse[i],chdiff,nss))*std::norm(VectorMultiplication(mmse[i],ch[j],nss));
        noise_mob[i] += nmob;
        delete [] chdiff;
      }
//      noise_mob[i] *= 1/sgain;
    }
  }

  double *sinr = new double[nss];
  double interference;
  double noiseAmplification;
  for (int i=0; i<nss; i++)
  {
    interference = 0;
    noiseAmplification = 0;  
    for (int j=0; j<nss; j++)
    {
      noiseAmplification += std::norm (mmse[i][j]) ;
      if (j!=i)
      	interference += std::norm(VectorMultiplication(mmse[i],ch[j],nss));
    }
    interference += nss*noise*noiseAmplification/signal;
    sinr[i] = std::norm(VectorMultiplication(mmse[i],ch[i],nss)) / (interference+noise_mob[i]);
    NS_LOG_DEBUG("sinr of " << i << "th stream=" << 10*log10(sinr[i]) << " if=" << 10*log10(interference) << " Nmob=" << noise_mob[i]);
  }

  for(int i=0; i<nss;i++)
  {
  delete [] ch[i];
  delete [] ch2[i];
  delete [] mmse[i];
  delete [] temp[i];
  }
  delete [] ch;
  delete [] ch2;
  delete [] mmse;
  delete [] temp;
  
  uint8_t cons_mode;
  switch (txVector.GetMode().GetConstellationSize () ) 
  {
	case 2:cons_mode=0; break;
	case 4:cons_mode=1; break;
	case 16:cons_mode=2; break;
	case 64:cons_mode=3; break;
	default:cons_mode=0; 
  }

  double eesm = 0;
  double esnr ;
  uint8_t snr_idx ;
  double min_sinr = 100;
  for (int i=0; i<nss; i++)
  {
     sinr[i] = 10.0*std::log10(sinr[i]);
     if (sinr[i] < min_sinr)
	 {	min_sinr = sinr[i];}
     if (sinr[i] < -20.0){sinr[i]= -20.0;}
     if (sinr[i] > 27.0){sinr[i]= 27.0;}
     snr_idx = (uint8_t)( (sinr[i]+20)*2 );
     eesm += RBIR[	cons_mode][snr_idx];
     
  }
  eesm /= nss;
  
  delete [] sinr;

  uint8_t current, prev = 0;
  for (int sidx =0; sidx<95; sidx++)
  {
	current = sidx;
	if (eesm <= RBIR[cons_mode][sidx])
	{	break;}
       prev = sidx;
  }
  if ( (RBIR[cons_mode][current]-eesm) > (eesm-RBIR[cons_mode][prev]) )
  	{esnr = prev;}
  else
  	{esnr = current;}
  esnr = esnr/2 - 20;
  if(esnr < min_sinr)
  	{esnr = min_sinr;}


  esnr = std::pow (10.0, esnr/10.0);
  NS_LOG_DEBUG("esnr=" << 10*std::log10(esnr)); 
  return esnr;
}
double
InterferenceHelper::CalculateNoiseInterferenceW (Ptr<InterferenceHelper::Event> event, NiChanges *ni) const
{
  double noiseInterference = m_firstPower;
  NS_ASSERT (m_rxing);
  for (NiChanges::const_iterator i = m_niChanges.begin () + 1; i != m_niChanges.end (); i++)
    {
      if ((event->GetEndTime () == i->GetTime ()) && event->GetRealRxPowerW () == -i->GetDelta ())
        {
          break;
        }
      ni->push_back (*i);
    }
  ni->insert (ni->begin (), NiChange (event->GetStartTime (), noiseInterference));
  ni->push_back (NiChange (event->GetEndTime (), 0));
  return noiseInterference;
}

//802.11ac channel bonding
double
InterferenceHelper::CalculateChunkSuccessRate (double snir, Time duration, WifiMode mode, uint16_t bw) const
{
  if (duration == NanoSeconds (0))
    {
      return 1.0;
    }
  uint32_t rate = mode.GetPhyRate ();
  uint16_t bandWidth = bw/20;
  uint64_t nbits = (uint64_t)(rate * duration.GetSeconds ()) / bandWidth;
  double csr = m_errorRateModel->GetChunkSuccessRate (mode, snir, (uint32_t)nbits);
  return csr;
}

double
InterferenceHelper::CalculateChunkSuccessRate (double snir, Time duration, WifiMode mode) const
{
  if (duration == NanoSeconds (0))
    {
      return 1.0;
    }
  uint32_t rate = mode.GetPhyRate ();
  uint64_t nbits = (uint64_t)(rate * duration.GetSeconds ());
  double csr = m_errorRateModel->GetChunkSuccessRate (mode, snir, (uint32_t)nbits);
  return csr;
}

//11ac: mutiple_stream_tx_per -CalculateSnr(payloadMode) -> CalculateSnr(payloadTxVector)
double
InterferenceHelper::CalculatePer (Ptr<const InterferenceHelper::Event> event, NiChanges *ni) 
{
  double psr = 1.0; /* Packet Success Rate */
  NiChanges::iterator j = ni->begin ();
  Time previous = (*j).GetTime ();
  WifiMode payloadMode = event->GetPayloadMode ();
  WifiTxVector payloadTxVector = event->GetTxVector ();
  WifiPreamble preamble = event->GetPreambleType ();
 WifiMode MfHeaderMode ;
 if (preamble==WIFI_PREAMBLE_HT_MF ||preamble==WIFI_PREAMBLE_VHT)
   {
    MfHeaderMode = WifiPhy::GetMFPlcpHeaderMode (payloadMode, preamble); //return L-SIG mode

   }
  WifiMode headerMode = WifiPhy::GetPlcpHeaderMode (payloadMode, preamble);
  Time plcpHeaderStart = (*j).GetTime () + MicroSeconds (WifiPhy::GetPlcpPreambleDurationMicroSeconds (payloadMode, preamble)); //packet start time+ preamble
  Time plcpHsigHeaderStart=plcpHeaderStart+ MicroSeconds (WifiPhy::GetPlcpHeaderDurationMicroSeconds (payloadMode, preamble));//packet start time+ preamble+L SIG
  Time plcpHtTrainingSymbolsStart = plcpHsigHeaderStart + MicroSeconds (WifiPhy::GetPlcpHtSigHeaderDurationMicroSeconds (payloadMode, preamble));//packet start time+ preamble+L SIG+HT SIG
  Time plcpPayloadStart =plcpHtTrainingSymbolsStart + MicroSeconds (WifiPhy::GetPlcpHtTrainingSymbolDurationMicroSeconds (payloadMode, preamble,event->GetTxVector())); //packet start time+ preamble+L SIG+HT SIG+Training
  double noiseInterferenceW = (*j).GetDelta ();
  double powerW = event->GetRxPowerW ();
  uint16_t bw = payloadMode.GetBandwidth()/1000000; // 171102 ywson : bandwidth consideration
    j++;
  while (ni->end () != j)
    {
      Time current = (*j).GetTime ();
      NS_ASSERT (current >= previous);
      //Case 1: Both prev and curr point to the payload
      if (previous >= plcpPayloadStart)
        {
          psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                          noiseInterferenceW,
                                                          payloadTxVector),
                                            current - previous,
                                            payloadMode, bw);
        }
      //Case 2: previous is before payload
      else if (previous >= plcpHtTrainingSymbolsStart)
        {
          //Case 2a: current is after payload
          if (current >= plcpPayloadStart)
            { 
               //Case 2ai and 2aii: All formats
               psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                current - plcpPayloadStart,
                                                payloadMode, bw);
                
              }
        }
      //Case 3: previous is in HT-SIG: Non HT will not enter here since it didn't enter in the last two and they are all the same for non HT
      else if (previous >=plcpHsigHeaderStart)
        {
          //Case 3a: cuurent after payload start
          if (current >=plcpPayloadStart)
             {
                   psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                current - plcpPayloadStart,
                                                payloadMode, bw);
                 
                    psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              headerMode),
                                               plcpHtTrainingSymbolsStart - previous,
                                                headerMode);
              }
          //case 3b: current after HT training symbols start
          else if (current >=plcpHtTrainingSymbolsStart)
             {
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                noiseInterferenceW,
                                                                payloadTxVector),
                                                   plcpHtTrainingSymbolsStart - previous,
                                                   headerMode);  
                   
             }
         //Case 3c: current is with previous in HT sig
         else
            {
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                noiseInterferenceW,
                                                                payloadTxVector),
                                                   current- previous,
                                                   headerMode);  
                   
            }
      }
      //Case 4: previous in L-SIG: GF will not reach here because it will execute the previous if and exit
      else if (previous >= plcpHeaderStart)
        {
          //Case 4a: current after payload start  
          if (current >=plcpPayloadStart)
             {
                   psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                      current - plcpPayloadStart,
                                                      payloadMode, bw);
                    //Case 4ai: Non HT format (No HT-SIG or Training Symbols)
              if (preamble == WIFI_PREAMBLE_LONG || preamble == WIFI_PREAMBLE_SHORT) //plcpHtTrainingSymbolsStart==plcpHeaderStart)
                {
                    psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                plcpPayloadStart - previous,
                                                headerMode);
                }

               else{
                    psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                      plcpHtTrainingSymbolsStart - plcpHsigHeaderStart,
                                                      headerMode);
                    psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                    noiseInterferenceW,
                                                                    payloadTxVector),
                                                      plcpHsigHeaderStart - previous,
                                                      MfHeaderMode);
                 }
              }
           //Case 4b: current in HT training symbol. non HT will not come here since it went in previous if or if the previous ifis not true this will be not true        
          else if (current >=plcpHtTrainingSymbolsStart)
             {
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                  plcpHtTrainingSymbolsStart - plcpHsigHeaderStart,
                                                  headerMode);
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                noiseInterferenceW,
                                                                payloadTxVector),
                                                   plcpHsigHeaderStart - previous,
                                                   MfHeaderMode);
              }
          //Case 4c: current in H sig.non HT will not come here since it went in previous if or if the previous ifis not true this will be not true
          else if (current >=plcpHsigHeaderStart)
             {
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                noiseInterferenceW,
                                                                payloadTxVector),
                                                  current - plcpHsigHeaderStart,
                                                  headerMode);
                 psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                 noiseInterferenceW,
                                                                 payloadTxVector),
                                                   plcpHsigHeaderStart - previous,
                                                   MfHeaderMode);

             }
         //Case 4d: Current with prev in L SIG
         else 
            {
                //Case 4di: Non HT format (No HT-SIG or Training Symbols)
              if (preamble == WIFI_PREAMBLE_LONG || preamble == WIFI_PREAMBLE_SHORT) //plcpHtTrainingSymbolsStart==plcpHeaderStart)
                {
                    psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                current - previous,
                                                headerMode);
                }
               else
                {
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                               noiseInterferenceW,
                                                               payloadTxVector),
                                                 current - previous,
                                                 MfHeaderMode);
                }
            }
        }
      //Case 5: previous is in the preamble works for all cases
      else
        {
          if (current >= plcpPayloadStart)
            {
              //for all
              psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                              payloadTxVector),
                                                current - plcpPayloadStart,
                                                payloadMode, bw); 
             
               // Non HT format (No HT-SIG or Training Symbols)
              if (preamble == WIFI_PREAMBLE_LONG || preamble == WIFI_PREAMBLE_SHORT)
                 psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                 noiseInterferenceW,
                                                                  payloadTxVector),
                                                    plcpPayloadStart - plcpHeaderStart,
                                                    headerMode);
              else
              // Greenfield or Mixed format
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                noiseInterferenceW,
                                                                payloadTxVector),
                                                  plcpHtTrainingSymbolsStart - plcpHsigHeaderStart,
                                                  headerMode);
              if (preamble == WIFI_PREAMBLE_HT_MF)
                 psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                 noiseInterferenceW,
                                                                 payloadTxVector),
                                                   plcpHsigHeaderStart-plcpHeaderStart,
                                                   MfHeaderMode);             
            }
          else if (current >=plcpHtTrainingSymbolsStart )
          { 
              // Non HT format will not come here since it will execute prev if
              // Greenfield or Mixed format
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                noiseInterferenceW,
                                                                payloadTxVector),
                                                  plcpHtTrainingSymbolsStart - plcpHsigHeaderStart,
                                                  headerMode);
              if (preamble == WIFI_PREAMBLE_HT_MF)
                 psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                 noiseInterferenceW,
                                                                 payloadTxVector),
                                                   plcpHsigHeaderStart-plcpHeaderStart,
                                                   MfHeaderMode);       
           }
          //non HT will not come here     
          else if (current >=plcpHsigHeaderStart)
             { 
                psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                noiseInterferenceW,
                                                                payloadTxVector),
                                                  current- plcpHsigHeaderStart,
                                                  headerMode); 
                if  (preamble != WIFI_PREAMBLE_HT_GF)
                 {
                   psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                   noiseInterferenceW,
                                                                   payloadTxVector),
                                                     plcpHsigHeaderStart-plcpHeaderStart,
                                                     MfHeaderMode);    
                  }          
             }
          // GF will not come here
          else if (current >= plcpHeaderStart)
            {
               if (preamble == WIFI_PREAMBLE_LONG || preamble == WIFI_PREAMBLE_SHORT)
                 {
                 psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                                 noiseInterferenceW,
                                                                  payloadTxVector),
                                                    current - plcpHeaderStart,
                                                    headerMode);
                 }
              else
                 {
              psr *= CalculateChunkSuccessRate (CalculateSnr (powerW,
                                                              noiseInterferenceW,
                                                             payloadTxVector),
                                               current - plcpHeaderStart,
                                               MfHeaderMode);
                       }
            }
        }

      noiseInterferenceW += (*j).GetDelta ();
      previous = (*j).GetTime ();
      j++;
    }

  double per = 1 - psr;
  return per;
}


struct InterferenceHelper::SnrPer
InterferenceHelper::CalculateSnrPer (Ptr<InterferenceHelper::Event> event)
{
  NiChanges ni;
  double noiseInterferenceW = CalculateNoiseInterferenceW (event, &ni);
  double snr = CalculateSnr (event->GetRxPowerW (),
                             noiseInterferenceW,
                             event->GetTxVector());
  /* calculate the SNIR at the start of the packet and accumulate
   * all SNIR changes in the snir vector.
   */
  double per = CalculatePer (event, &ni);

  struct SnrPer snrPer;
  snrPer.snr = snr;
  snrPer.per = per;
  
  return snrPer;
}

//shbyeon phy module for ampdu reception
double
InterferenceHelper::CalculateAmpduSubframePer (WifiMode payloadMode,double snr, uint32_t size) const
{
	NS_LOG_FUNCTION(this);

	double per = 1 - m_errorRateModel->GetChunkSuccessRate (payloadMode, snr, size * 8);

  return per;
}

//802.11ac channel bonding: per mpdu error calculation
std::list<struct InterferenceHelper::SnrPer>
InterferenceHelper::CalculateAmpduSnrPer (Ptr<InterferenceHelper::Event> event, uint32_t size, uint16_t ampduSize, Ptr<Packet> packet)
{
	NS_LOG_FUNCTION(this);
  NiChanges ni;
  double noiseInterferenceW = CalculateNoiseInterferenceW (event, &ni);
  double snr = CalculateSnr (event->GetRxPowerW (),
                             noiseInterferenceW,
                             event->GetTxVector ());
	std::list<struct InterferenceHelper::SnrPer> snrPers;
  double per[ampduSize+1];
  double sinr[ampduSize+1];
  uint16_t k=0;
  for(k=0;k<ampduSize+1;k++)
  {
    per[k]=0;
    sinr[k]=0;
  }
  sinr[0] = snr;
  CalculateAmpduPer(event, &ni, packet, per, sinr);
  struct SnrPer snrPer;
  for(k=0;k<ampduSize+1;k++)
  {
    snrPer.snr = sinr[k];
    snrPer.per = per[k];
    snrPers.push_back(snrPer);
    NS_LOG_DEBUG("subframe=" << k << "th' sinr= " << 10*log10(sinr[k]) << " per=" << per[k]);
  }
  return snrPers;
}

void
InterferenceHelper::DeleteChannelMatrix(Ptr<const InterferenceHelper::Event> event)
{
  event->GetTxVector().DeleteChannelMatrix();
}

//shbyeon ampdu per calculation
void
InterferenceHelper::CalculateAmpduPer (Ptr<const InterferenceHelper::Event> event, NiChanges *ni, Ptr<Packet> packet, double per[], double sinr[]) 
{
	NS_LOG_FUNCTION(this);
  NiChanges::iterator j = ni->begin ();
  Time previous = (*j).GetTime ();
  WifiMode payloadMode = event->GetPayloadMode ();
	WifiTxVector txVector = event->GetTxVector();
  WifiPreamble preamble = event->GetPreambleType ();
  WifiMode headerMode = WifiPhy::GetPlcpHeaderMode (payloadMode, preamble);
  Time plcpHeaderStart = (*j).GetTime () + MicroSeconds (WifiPhy::GetPlcpPreambleDurationMicroSeconds (payloadMode, preamble));
  Time plcpPayloadStart = plcpHeaderStart + MicroSeconds (WifiPhy::GetPlcpHeaderDurationMicroSeconds (payloadMode, preamble)) + MicroSeconds (WifiPhy::GetPlcpHtSigHeaderDurationMicroSeconds (payloadMode, preamble)) + MicroSeconds (WifiPhy::GetPlcpHtTrainingSymbolDurationMicroSeconds (payloadMode, preamble, txVector)); // 171102 ywson : extended PHY header consideration
  double noiseInterferenceW = (*j).GetDelta ();
  double powerW = event->GetRxPowerW ();
  NS_LOG_DEBUG("Preamble RxPower=" << 10*log10(powerW));
  double ampduSize = packet->GetSize();
  uint16_t bw = payloadMode.GetBandwidth()/1000000;
  //shbyeon ampdu subframe time location
  MpduAggregator::DeaggregatedMpdus packets;
  packets = MpduAggregator::Deaggregate(packet);
  uint16_t numOfSubframe = packets.size();
  double psr[numOfSubframe+1];
  uint16_t k=0;
  Time subframeDuration_us[numOfSubframe+1];
  for(k=0;k<numOfSubframe;k++)
  {
    subframeDuration_us[k+1] = plcpPayloadStart;
    psr[k+1]=1.0;
  }
  //shbyeon '0' is only for the plcp header
  psr[0]=1.0; 
  subframeDuration_us[0] = plcpPayloadStart;

  j++;

  //shbyeon set subframe transmission time
  Time ampduTime = (*(--ni->end())).GetTime() - plcpPayloadStart;
  uint16_t subframeIndex = 1;
  for (MpduAggregator::DeaggregatedMpdusCI i = packets.begin();
      i != packets.end(); ++i)
  {
    if (subframeIndex == numOfSubframe)
    {
      subframeDuration_us[numOfSubframe] = (*(--ni->end())).GetTime();
    }
    else
    {
      subframeDuration_us[subframeIndex] = subframeDuration_us[subframeIndex-1]
        + MicroSeconds(ampduTime.GetMicroSeconds() *(*i).first->GetSize()/ampduSize);
    }
    subframeIndex++;
  }

  subframeIndex=0;

  NS_LOG_DEBUG("subframeEnd=" << subframeDuration_us[numOfSubframe] <<
      ", nichangeEnd=" << (*(--ni->end())).GetTime() << 
      ", diff=" << subframeDuration_us[numOfSubframe] - (*(--ni->end())).GetTime() <<
      ", datarate=" << (double)payloadMode.GetDataRate()/1000000);
  
  while(ni->end () != j)
  {
  	if (subframeIndex > packets.size())//160404 skim11 
			break;
    Time current = (*j).GetTime ();
    NS_LOG_DEBUG("previous="<< previous << " current=" << current << ", subframeIndex=" << subframeIndex << ", nisize=" << ni->size());
    NS_ASSERT(current >= previous);

    if(previous >= plcpPayloadStart)
    {
      if(current >= subframeDuration_us[subframeIndex])
      {
        psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex),
                                                      subframeDuration_us[subframeIndex] - previous, payloadMode, bw);
        sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex); 
        previous = subframeDuration_us[subframeIndex];
        
        if(subframeIndex == packets.size()+0)//160404 skim11 +1 -> +0
        {
          noiseInterferenceW += (*j).GetDelta ();
          previous = (*j).GetTime ();
          j++;
          if(ni->end() == j)
          {
            break;
          }
        }
        else
          subframeIndex++;

        if(current <= previous)
        {
          j++;
        }
        continue;
      }
      else
      {
        psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex),
                                                      current - previous, payloadMode, bw);
        sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
      }
    }
    else if (previous >= plcpHeaderStart)
    {
      if (current >= plcpPayloadStart)
      {
        psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector),
                                                      plcpPayloadStart - previous, headerMode);
        sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
        subframeIndex++; //from now on, subframe psr will be calculated
        if(current >= subframeDuration_us[subframeIndex])
        {
          psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex),
                                                        subframeDuration_us[subframeIndex] - plcpPayloadStart, payloadMode, bw);
          sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
          previous = subframeDuration_us[subframeIndex];
          subframeIndex++;
          if(current <= previous)
          {
            j++;
          }
          continue;
        }
        else
        {
          psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex),
                                                        current - plcpPayloadStart, payloadMode, bw);
          sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
        }
      }
      else
      {
        NS_ASSERT (current >= plcpHeaderStart);
        psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector),
                                                      current - previous, headerMode);
        sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
      }
    }
    else //shbyeon: this is for initial position
    {
      if (current >= plcpPayloadStart)
      {
        psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector),
                                                      plcpPayloadStart - plcpHeaderStart, headerMode);
        sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
        subframeIndex++;

        if(current >= subframeDuration_us[subframeIndex])
        {
          psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex),
                                                        subframeDuration_us[subframeIndex] - plcpPayloadStart, payloadMode, bw);
          sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
          previous = subframeDuration_us[subframeIndex];
          subframeIndex++;
          if(current <= previous)
          {
            j++;
          }
          continue;
        }
        else
        {
          psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex),
                                                        current - plcpPayloadStart, payloadMode, bw);
          sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
        }
      }
      else if (current >= plcpHeaderStart)
      {
        psr[subframeIndex] *= CalculateChunkSuccessRate (CalculateSnr (powerW, noiseInterferenceW, txVector),
                                                      current - plcpHeaderStart, headerMode);
        sinr[subframeIndex] = CalculateSnr (powerW, noiseInterferenceW, txVector, subframeIndex);
      }
    }
    noiseInterferenceW += (*j).GetDelta ();
    previous = (*j).GetTime ();
    j++;
  }

  for(k=0;k<numOfSubframe+1;k++)
    per[k] = 1 - psr[k];
  return;
}


double
InterferenceHelper::CalculatePlcpPer (WifiMode payloadMode, double snr, WifiPreamble preamble) const
{
	NS_LOG_FUNCTION(this);
  WifiMode headerMode = WifiPhy::GetPlcpHeaderMode (payloadMode, preamble);
  Time plcpHeaderDuration= MicroSeconds (WifiPhy::GetPlcpHeaderDurationMicroSeconds (payloadMode, preamble));
  double per= 1- CalculateChunkSuccessRate (snr, plcpHeaderDuration, headerMode);

  return per;
}

struct InterferenceHelper::SnrPer
InterferenceHelper::CalculatePlcpSnrPer(Ptr<InterferenceHelper::Event> event)
{
	NS_LOG_FUNCTION(this);
  NiChanges ni;
  double noiseInterferenceW = CalculateNoiseInterferenceW (event, &ni);
  double snr = CalculateSnr (event->GetRxPowerW (),
                             noiseInterferenceW,
                             event->GetPayloadMode ());
  /* calculate the SNIR at the start of the packet and accumulate
   * all SNIR changes in the snir vector.
   */
  double per = CalculatePlcpPer (event->GetPayloadMode (), snr , event->GetPreambleType() );

  struct SnrPer snrPer;
  snrPer.snr = snr;
  snrPer.per = per;
  return snrPer;
}

void
InterferenceHelper::EraseEvents (void)
{
  m_niChanges.clear ();
  m_rxing = false;
  m_firstPower = 0.0;
}
InterferenceHelper::NiChanges::iterator
InterferenceHelper::GetPosition (Time moment)
{
  return std::upper_bound (m_niChanges.begin (), m_niChanges.end (), NiChange (moment, 0));

}
void
InterferenceHelper::AddNiChangeEvent (NiChange change)
{
  m_niChanges.insert (GetPosition (change.GetTime ()), change);
}
void
InterferenceHelper::NotifyRxStart ()
{
  m_rxing = true;
}
void
InterferenceHelper::NotifyRxEnd ()
{
  m_rxing = false;
}

//11ac: mutiple_stream_tx_per 
std::complex<double> 
InterferenceHelper::Determinant (std::complex<double> **mtx, int size)
 {
   
   std::complex<double> det = std::complex<double>(0,0);
   

   if (size< 1) { /* Error */   }
   else if (size== 1) 
   { /* Shouldn't get used */
   	det = mtx[0][0];
    }
   else if (size == 2) 
   {
      det = mtx[0][0] * mtx[1][1] - mtx[1][0] * mtx[0][1];
   } 
   else 
   {
   
      int i,j,j1,j2;
      std::complex<double> **m = new std::complex<double> *[size];
      for (int i=0;i<size;i++)
     {
  	 m[i] = new std::complex<double>[size];
     }
        for (j1=0;j1<size;j1++) 
      {
         for (i=1;i<size;i++) 
	  {
            j2 = 0;
            for (j=0;j<size;j++) 
	     {
               if (j == j1)
                  continue;
               m[i-1][j2] = mtx[i][j];
               j2++;
            }
         }
         det += std::pow(-1.0,j1+2.0) * mtx[0][j1] * Determinant(m,size-1);
      }
      
      for(int i=0; i<size;i++)
	{
	  delete [] m[i];
	}
      delete [] m;

   }
   //NS_LOG_DEBUG("determinant: "<<det);
   //NS_LOG_DEBUG("mtx: "<<mtx[0][0]<<"  "<<mtx[0][1]<<"  "<<mtx[1][0]<<"  "<<mtx[1][1]);
   return(det);
 }
void 
InterferenceHelper::Inverse (std::complex<double> **mtx, int size, std::complex<double> **cmtx)
{
   int i,j,ii,jj,i1,j1;
   std::complex<double> det;
   std::complex<double> **c = new std::complex<double> *[size];
   for (int i=0;i<size;i++)
   {
  	 c[i] = new std::complex<double>[size];
   }

   for (j=0;j<size;j++) {
      for (i=0;i<size;i++) {

         /* Form the adjoint a_ij */
         i1 = 0;
         for (ii=0;ii<size;ii++) {
            if (ii == i)
               continue;
            j1 = 0;
            for (jj=0;jj<size;jj++) {
               if (jj == j)
                  continue;
               c[i1][j1] = mtx[ii][jj];
               j1++;
            }
            i1++;
         }

         /* Calculate the determinate */
         det = Determinant(c,size-1);

         /* Fill in the elements of the cofactor */
         cmtx[i][j] = std::pow(-1.0,i+j+2.0) * det;
      }
   }
   for(int i=0; i<size;i++)
   {
      delete [] c[i];
   }
   delete [] c;
   Transpose(cmtx,size);
   std::complex<double> determinant = Determinant(mtx,size);
   for(int i=0;i<size;i++)
   	for(int j=0; j<size; j++)
		cmtx[i][j] /=determinant;
	
}
void 
InterferenceHelper::Transpose (std::complex<double> **mtx, int size)
{
   int i,j;
   std::complex<double> tmp;

   for (i=1;i<size;i++) {
      for (j=0;j<i;j++) {
         tmp = mtx[i][j];
         mtx[i][j] = mtx[j][i];
         mtx[j][i] = tmp;
      }
   }
}
void 
InterferenceHelper::Hermitian (std::complex<double> **mtx, int size)
{
   Transpose(mtx,size);
   for (int i=0;i<size;i++) {
      for (int j=0;j<size;j++) {
         mtx[i][j] = std::conj(mtx[i][j]);
      }
   }
}
void 
InterferenceHelper::MtxMultiplication (std::complex<double> **mtx1, std::complex<double> **mtx2, int size, std::complex<double> **cmtx)
{
  for (int i=0;i<size;i++)
  {
  for (int j=0;j<size;j++)
  	{
		cmtx[i][j]=0;
		for (int k=0;k<size;k++)
  		{
           	    cmtx[i][j] += mtx1[i][k]*mtx2[k][j]; 
  		}
  	}
  }  		
}
std::complex<double>
InterferenceHelper::VectorMultiplication (std::complex<double> *vt1, std::complex<double> *vt2, int size)
{
  std::complex<double> innerproduct = std::complex<double>(0,0);
  for (int i=0;i<size;i++)
  {
      innerproduct += vt1[i]*vt2[i];
  }		
  return innerproduct;
}

//160328 skim11
double
InterferenceHelper::WToDbm(double powerW){
	return 10.0*std::log10(powerW*1000.0);
}
	
} // namespace ns3
