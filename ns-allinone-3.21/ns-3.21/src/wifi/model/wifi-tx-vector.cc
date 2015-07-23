/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 CTTC
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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 *       : Ghada Badawy <gbadawy@gmail.com>
 */

#include "ns3/wifi-tx-vector.h"
#include "ns3/log.h"

namespace ns3 {

WifiTxVector::WifiTxVector ()
{
  SetNumberMpdus(1);
}

WifiTxVector::WifiTxVector (WifiMode mode, uint8_t powerLevel, uint8_t retries,
                            bool shortGuardInterval, uint8_t nss, uint8_t ness, bool stbc)
  : m_mode (mode),
    m_txPowerLevel (powerLevel),
    m_retries (retries),
    m_shortGuardInterval(shortGuardInterval),
    m_nss(nss),
    m_ness(ness),
    m_stbc(stbc),
    m_bw(20),
    m_mpdus(1),
    m_caudalLoss(false),
    m_lowRate(false),
    m_setChannel(false),
    m_prevMpdus(1)
{
}
WifiTxVector::~WifiTxVector()
{
//  if(m_hmatrix && m_setChannel)
//    DeleteChannelMatrix();
}

WifiMode
WifiTxVector::GetMode (void) const
{
  return m_mode;
}
uint8_t 
WifiTxVector::GetTxPowerLevel (void) const
{
  return m_txPowerLevel;
}
uint8_t 
WifiTxVector::GetRetries (void) const
{
  return m_retries;
}
bool 
WifiTxVector::IsShortGuardInterval (void) const
{
 return m_shortGuardInterval;
}
uint8_t 
WifiTxVector::GetNss (void) const
{
  return m_nss;
}
uint8_t 
WifiTxVector::GetNess (void) const
{
  return m_ness;
}
//11ac: mutiple_stream_tx_channel
void
WifiTxVector::GetChannelMatrix ( std::complex<double> ** ch) 
{
  for (int i=0;i<m_nss;i++)
    for (int j=0; j<m_nss; j++)
      ch[i][j] = m_hmatrix[0][i][j];
}
//caudal loss
void
WifiTxVector::GetChannelMatrix (std::complex<double> ** ch, uint16_t subframeIdx) 
{
    for (int j=0;j<m_nss;j++)
      for (int k=0; k<m_nss; k++)
        ch[j][k] = m_hmatrix[subframeIdx][j][k];
}
bool 
WifiTxVector::IsStbc (void) const
{
  return m_stbc;
}

void 
WifiTxVector::SetMode (WifiMode mode)
{
  m_mode=mode;
}

//802.11ac channel bonding
void
WifiTxVector::SetBandwidth (uint8_t bw)
{
	//TO DO:
	//if m_mode is 802.11ac or 802.11n, change m_mode if bw > 20
	m_bw = bw;
}
uint8_t 
WifiTxVector::GetBandwidth (void)
{
	return m_bw;
}
//caudal loss
bool
WifiTxVector::GetCaudalLoss (void)
{
  return m_caudalLoss;
}
void
WifiTxVector::SetCaudalLoss (bool cl)
{
  m_caudalLoss = cl;
}
bool
WifiTxVector::GetLowRate (void)
{
  return m_lowRate;
}
void
WifiTxVector::SetLowRate (bool input)
{
  m_lowRate = input;
}
void
WifiTxVector::SetNumberMpdus (uint16_t mpdus)
{
  m_mpdus = mpdus;
}
uint16_t
WifiTxVector::GetNumberMpdus (void)
{
  return m_mpdus;
}

void 
WifiTxVector::SetTxPowerLevel (uint8_t powerlevel)
{
  m_txPowerLevel=powerlevel;
}
void 
WifiTxVector::SetRetries (uint8_t retries)
{
  m_retries = retries;
}
void 
WifiTxVector::SetShortGuardInterval (bool guardinterval)
{
  m_shortGuardInterval=guardinterval;
}
void 
WifiTxVector::SetNss (uint8_t nss)
{
  m_nss= nss;
}
void 
WifiTxVector::SetNess (uint8_t ness)
{
  m_ness=ness;
}
//11ac: mutiple_stream_tx_channel
void 
WifiTxVector::SetChannelMatrix (std::complex<double> * vector, uint8_t nss, uint16_t nMpdus)
{
  //if(m_setChannel)
  //  DeleteChannelMatrix();
  m_hmatrix = new std::complex<double> ** [nMpdus];
  for(int i = 0; i < nMpdus; i++)
  {
    m_hmatrix[i] = new std::complex<double> * [nss];
    for (int j = 0; j < nss; j++)
      m_hmatrix[i][j] = new std::complex<double> [nss];
  }

  for(int i = 0; i < nMpdus; i++)
  {
    for(int r = 0; r < nss; r++)
    {
      for(int c = 0; c < nss; c++)
      {
        m_hmatrix[i][r][c] = vector[1+c+r*nss+i*nss*nss];
      }
    }
  }
  m_prevMpdus = nMpdus;
  m_setChannel = true;
}
void
WifiTxVector::DeleteChannelMatrix (void)
{
  m_setChannel = false;
  uint16_t nMpdus = m_prevMpdus;
  uint16_t nss = GetNss();
  for(int i=0; i<nMpdus; i++)
  {
    for(int j=0; j<nss; j++)
    {
      delete [] m_hmatrix[i][j];
    }
    delete [] m_hmatrix[i];
  }
  delete [] m_hmatrix;
  m_hmatrix = NULL;
}
void 
WifiTxVector::SetStbc (bool stbc)
{
  m_stbc=stbc;
}

std::ostream & operator << ( std::ostream &os, const WifiTxVector &v)
{ 
  os << "mode:" << v.GetMode() <<
    " txpwrlvl:" << (uint32_t)v.GetTxPowerLevel() <<
    " retries:" << (uint32_t)v.GetRetries() <<
    " Short GI: " << v.IsShortGuardInterval() <<
    " Nss: " << (uint32_t)v.GetNss() <<
    " Ness: " << (uint32_t)v.GetNess() <<
    " STBC: " << v.IsStbc();
  return os;
}

} // namespace ns3
