/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Telum (www.telum.ru)
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
 * Author: Kirill Andreev <andreev@telum.ru>
 */

#include "jakes-propagation-loss-model.h"
#include "ns3/double.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("Jakes");

namespace ns3
{
NS_OBJECT_ENSURE_REGISTERED (JakesPropagationLossModel);


const double JakesPropagationLossModel::PI = 3.14159265358979323846;

JakesPropagationLossModel::JakesPropagationLossModel()
{
  m_uniformVariable = CreateObject<UniformRandomVariable> ();
  m_uniformVariable->SetAttribute ("Min", DoubleValue (-1.0 * PI));
  m_uniformVariable->SetAttribute ("Max", DoubleValue (PI));
}

JakesPropagationLossModel::~JakesPropagationLossModel()
{}

TypeId
JakesPropagationLossModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::JakesPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .AddConstructor<JakesPropagationLossModel> ()
  ;
  return tid;
}

double
JakesPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
  Ptr<JakesProcess> pathData = m_propagationCache.GetPathData (a, b, 0 /**Spectrum model uid is not used in PropagationLossModel*/);
  if (pathData == 0)
    {
      pathData = CreateObject<JakesProcess> ();
      pathData->SetPropagationLossModel (this);
      m_propagationCache.AddPathData (pathData, a, b, 0/**Spectrum model uid is not used in PropagationLossModel*/);
    }
  return txPowerDbm + pathData->GetChannelGainDb ();
}

//11ac: multiple strem tx - by ywson
std::complex<double> * JakesPropagationLossModel::DoCalcRxPower (std::complex<double> * hvector, Ptr<MobilityModel> a, Ptr<MobilityModel> b, uint8_t nss, double * mpduTx) const
{
  if(mpduTx)
  {
    NS_LOG_DEBUG("caudal enabled");
    int count = mpduTx[0];
    count = mpduTx[0];
    mpduTx[0] = 0;
    for(int j = 0; j < count; j++)
    {
      Time refTime = Seconds(mpduTx[j]);
      NS_LOG_DEBUG(j << " " << refTime.GetMicroSeconds() << " " << mpduTx[j]);
      for(int i = 1; i <= (nss*nss); i++)
      {
        Ptr<JakesProcess> pathData = m_propagationCache.GetPathData (a, b, i);
        if (pathData == 0)
        {
          pathData = CreateObject<JakesProcess> ();
          pathData->SetPropagationLossModel (this);
          m_propagationCache.AddPathData (pathData, a, b, i);
        }
        hvector[i+j*(nss*nss)] = pathData->GetComplexGain(refTime);
      }
    }
    mpduTx[0]=count;
  }
  else
  {
    NS_LOG_DEBUG("caudal disabled");
    for(int i = 1; i <= (nss*nss); i++)
    {
      Ptr<JakesProcess> pathData = m_propagationCache.GetPathData (a, b, i);
      if (pathData == 0)
      {
        pathData = CreateObject<JakesProcess> ();
        pathData->SetPropagationLossModel (this);
        m_propagationCache.AddPathData (pathData, a, b, i);
      }
      hvector[i] = pathData->GetComplexGain();
    }

  }
  return hvector;
}

Ptr<UniformRandomVariable>
JakesPropagationLossModel::GetUniformRandomVariable () const
{
  return m_uniformVariable;
}

int64_t
JakesPropagationLossModel::DoAssignStreams (int64_t stream)
{
  m_uniformVariable->SetStream (stream);
  return 1;
}

} // namespace ns3

