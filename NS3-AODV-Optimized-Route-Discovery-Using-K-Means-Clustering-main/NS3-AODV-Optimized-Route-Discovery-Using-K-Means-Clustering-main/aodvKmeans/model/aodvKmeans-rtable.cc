/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 * Based on
 *      NS-2 aodvKmeans model developed by the CMU/MONARCH group and optimized and
 *      tuned by Samir Das and Mahesh Marina, University of Cincinnati;
 *
 *      aodvKmeans-UU implementation by Erik Nordström of Uppsala University
 *      http://core.it.uu.se/core/index.php/aodvKmeans-UU
 *
 * Authors: Elena Buchatskaia <borovkovaes@iitp.ru>
 *          Pavel Boyko <boyko@iitp.ru>
 */

#include "aodvKmeans-rtable.h"
#include <algorithm>
#include <iomanip>
#include "ns3/simulator.h"
#include "ns3/log.h"

using namespace std;
namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("aodvKmeansRoutingTable");

namespace aodvKmeans {

/*
 The Routing Table
 */

RoutingTableEntry::RoutingTableEntry (Ptr<NetDevice> dev, Ipv4Address dst, bool vSeqNo, uint32_t seqNo,
                                      Ipv4InterfaceAddress iface, uint16_t hops, Ipv4Address nextHop, Time lifetime,
                                      uint32_t txError, uint32_t positionX, uint32_t positionY, uint32_t freeSpace)
  : m_ackTimer (Timer::CANCEL_ON_DESTROY),
    m_validSeqNo (vSeqNo),
    m_seqNo (seqNo),
    m_hops (hops),
    m_lifeTime (lifetime + Simulator::Now ()),
    m_iface (iface),
    m_flag (VALID),
    m_reqCount (0),
    m_blackListState (false),
    m_blackListTimeout (Simulator::Now ()),
    m_txerrorCount(txError),
    m_positionX(positionX),
    m_positionY(positionY),
    m_freeSpace(freeSpace)
{
  m_ipv4Route = Create<Ipv4Route> ();
  m_ipv4Route->SetDestination (dst);
  m_ipv4Route->SetGateway (nextHop);
  m_ipv4Route->SetSource (m_iface.GetLocal ());
  m_ipv4Route->SetOutputDevice (dev);
}

RoutingTableEntry::~RoutingTableEntry ()
{
}

bool
RoutingTableEntry::InsertPrecursor (Ipv4Address id)
{
  NS_LOG_FUNCTION (this << id);
  if (!LookupPrecursor (id))
    {
      m_precursorList.push_back (id);
      return true;
    }
  else
    {
      return false;
    }
}

bool
RoutingTableEntry::LookupPrecursor (Ipv4Address id)
{
  NS_LOG_FUNCTION (this << id);
  for (std::vector<Ipv4Address>::const_iterator i = m_precursorList.begin (); i
       != m_precursorList.end (); ++i)
    {
      if (*i == id)
        {
          NS_LOG_LOGIC ("Precursor " << id << " found");
          return true;
        }
    }
  NS_LOG_LOGIC ("Precursor " << id << " not found");
  return false;
}

bool
RoutingTableEntry::DeletePrecursor (Ipv4Address id)
{
  NS_LOG_FUNCTION (this << id);
  std::vector<Ipv4Address>::iterator i = std::remove (m_precursorList.begin (),
                                                      m_precursorList.end (), id);
  if (i == m_precursorList.end ())
    {
      NS_LOG_LOGIC ("Precursor " << id << " not found");
      return false;
    }
  else
    {
      NS_LOG_LOGIC ("Precursor " << id << " found");
      m_precursorList.erase (i, m_precursorList.end ());
    }
  return true;
}

void
RoutingTableEntry::DeleteAllPrecursors ()
{
  NS_LOG_FUNCTION (this);
  m_precursorList.clear ();
}

bool
RoutingTableEntry::IsPrecursorListEmpty () const
{
  return m_precursorList.empty ();
}

void
RoutingTableEntry::GetPrecursors (std::vector<Ipv4Address> & prec) const
{
  NS_LOG_FUNCTION (this);
  if (IsPrecursorListEmpty ())
    {
      return;
    }
  for (std::vector<Ipv4Address>::const_iterator i = m_precursorList.begin (); i
       != m_precursorList.end (); ++i)
    {
      bool result = true;
      for (std::vector<Ipv4Address>::const_iterator j = prec.begin (); j
           != prec.end (); ++j)
        {
          if (*j == *i)
            {
              result = false;
            }
        }
      if (result)
        {
          prec.push_back (*i);
        }
    }
}

void
RoutingTableEntry::Invalidate (Time badLinkLifetime)
{
  NS_LOG_FUNCTION (this << badLinkLifetime.As (Time::S));
  if (m_flag == INVALID)
    {
      return;
    }
  m_flag = INVALID;
  m_reqCount = 0;
  m_lifeTime = badLinkLifetime + Simulator::Now ();
}

void
RoutingTableEntry::Print (Ptr<OutputStreamWrapper> stream, Time::Unit unit /* = Time::S */) const
{
  std::ostream* os = stream->GetStream ();
  // Copy the current ostream state
  std::ios oldState (nullptr);
  oldState.copyfmt (*os);

  *os << std::resetiosflags (std::ios::adjustfield) << std::setiosflags (std::ios::left);

  std::ostringstream dest, gw, iface, expire;
  dest << m_ipv4Route->GetDestination ();
  gw << m_ipv4Route->GetGateway ();
  iface << m_iface.GetLocal ();
  expire << std::setprecision (2) << (m_lifeTime - Simulator::Now ()).As (unit);
  *os << std::setw (16) << dest.str();
  *os << std::setw (16) << gw.str();
  *os << std::setw (16) << iface.str();
  *os << std::setw (16);
  switch (m_flag)
    {
    case VALID:
      {
        *os << "UP";
        break;
      }
    case INVALID:
      {
        *os << "DOWN";
        break;
      }
    case IN_SEARCH:
      {
        *os << "IN_SEARCH";
        break;
      }
    }

  *os << std::setw (16) << expire.str();
  *os << m_hops << std::endl;
  // Restore the previous ostream state
  (*os).copyfmt (oldState);
}

/*
 The Routing Table
 */

RoutingTable::RoutingTable (Time t)
  : m_badLinkLifetime (t)
{
}

bool
RoutingTable::LookupRoute (Ipv4Address id, RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this << id);
  Purge ();
  if (m_ipv4AddressEntry.empty ())
    {
      NS_LOG_LOGIC ("Route to " << id << " not found; m_ipv4AddressEntry is empty");
      return false;
    }
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator i =
    m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Route to " << id << " not found");
      return false;
    }
  rt = i->second;
  NS_LOG_LOGIC ("Route to " << id << " found");
  return true;
}

bool
RoutingTable::LookupValidRoute (Ipv4Address id, RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this << id);
  if (!LookupRoute (id, rt))
    {
      NS_LOG_LOGIC ("Route to " << id << " not found");
      return false;
    }
  NS_LOG_LOGIC ("Route to " << id << " flag is " << ((rt.GetFlag () == VALID) ? "valid" : "not valid"));
  return (rt.GetFlag () == VALID);
}

bool
RoutingTable::DeleteRoute (Ipv4Address dst)
{
  NS_LOG_FUNCTION (this << dst);
  Purge ();
  if (m_ipv4AddressEntry.erase (dst) != 0)
    {
      NS_LOG_LOGIC ("Route deletion to " << dst << " successful");
      return true;
    }
  NS_LOG_LOGIC ("Route deletion to " << dst << " not successful");
  return false;
}

bool
RoutingTable::AddRoute (RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this);
  Purge ();
  if (rt.GetFlag () != IN_SEARCH)
    {
      rt.SetRreqCnt (0);
    }
  std::pair<std::map<Ipv4Address, RoutingTableEntry>::iterator, bool> result =
    m_ipv4AddressEntry.insert (std::make_pair (rt.GetDestination (), rt));
  return result.second;
}

bool
RoutingTable::Update (RoutingTableEntry & rt)
{
  NS_LOG_FUNCTION (this);
  std::map<Ipv4Address, RoutingTableEntry>::iterator i =
    m_ipv4AddressEntry.find (rt.GetDestination ());
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Route update to " << rt.GetDestination () << " fails; not found");
      return false;
    }
  i->second = rt;
  if (i->second.GetFlag () != IN_SEARCH)
    {
      NS_LOG_LOGIC ("Route update to " << rt.GetDestination () << " set RreqCnt to 0");
      i->second.SetRreqCnt (0);
    }
  return true;
}

bool
RoutingTable::SetEntryState (Ipv4Address id, RouteFlags state)
{
  NS_LOG_FUNCTION (this);
  std::map<Ipv4Address, RoutingTableEntry>::iterator i =
    m_ipv4AddressEntry.find (id);
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Route set entry state to " << id << " fails; not found");
      return false;
    }
  i->second.SetFlag (state);
  i->second.SetRreqCnt (0);
  NS_LOG_LOGIC ("Route set entry state to " << id << ": new state is " << state);
  return true;
}

void
RoutingTable::GetListOfDestinationWithNextHop (Ipv4Address nextHop, std::map<Ipv4Address, uint32_t> & unreachable )
{
  NS_LOG_FUNCTION (this);
  Purge ();
  unreachable.clear ();
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      if (i->second.GetNextHop () == nextHop)
        {
          NS_LOG_LOGIC ("Unreachable insert " << i->first << " " << i->second.GetSeqNo ());
          unreachable.insert (std::make_pair (i->first, i->second.GetSeqNo ()));
        }
    }
}

void
RoutingTable::InvalidateRoutesWithDst (const std::map<Ipv4Address, uint32_t> & unreachable)
{
  NS_LOG_FUNCTION (this);
  Purge ();
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      for (std::map<Ipv4Address, uint32_t>::const_iterator j =
             unreachable.begin (); j != unreachable.end (); ++j)
        {
          if ((i->first == j->first) && (i->second.GetFlag () == VALID))
            {
              NS_LOG_LOGIC ("Invalidate route with destination address " << i->first);
              i->second.Invalidate (m_badLinkLifetime);
            }
        }
    }
}

void
RoutingTable::DeleteAllRoutesFromInterface (Ipv4InterfaceAddress iface)
{
  NS_LOG_FUNCTION (this);
  if (m_ipv4AddressEntry.empty ())
    {
      return;
    }
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); )
    {
      if (i->second.GetInterface () == iface)
        {
          std::map<Ipv4Address, RoutingTableEntry>::iterator tmp = i;
          ++i;
          m_ipv4AddressEntry.erase (tmp);
        }
      else
        {
          ++i;
        }
    }
}
std::vector<Ipv4Address> 
RoutingTable::Kmeans (Ipv4Address dst, uint32_t positionX, uint32_t positionY)
{
    Purge();
  
    // features list 为了获取每个节点的特征值
    cout<<"------初始化特征列表------"<<endl;
    int n = m_ipv4AddressEntry.size();
    cout<<"m_ipv4AddressEntry.size()------"<<n<<endl;
    double features[n][3];
    int k = 2;
    double cluster_center[k][3];
    int cluster_assignments[n];

  
    int i = 0;
    
    for (std::map<Ipv4Address, RoutingTableEntry>::iterator it = m_ipv4AddressEntry.begin (); it != m_ipv4AddressEntry.end (); ++it)
    {
      cout<<"it->first.IsBroadcast()------"<<it->first.IsBroadcast()<<endl;
      cout<<"it->first.IsLocalhost()------"<<it->first.IsLocalhost()<<endl;
      cout<<"it->first.IsMulticast()------"<<it->first.IsMulticast()<<endl;
    //  cout<<"it->first.IsSubnetDirectedBroadcast(Ipv4Mask((char *)"255.255.255.0"))------"<<it->first.IsSubnetDirectedBroadcast(Ipv4Mask((char *)"255.255.255.0"))<<endl;
      cout<<"it->second.GetFlag()------"<<it->second.GetFlag()<<endl;
      cout<<"it->second.GetHop()------"<<it->second.GetHop()<<endl;
      if(it->first.IsBroadcast() || it->first.IsLocalhost() || it->first.IsMulticast() || 
      it->first.IsSubnetDirectedBroadcast(Ipv4Mask((char *)"255.255.255.0")) || it->second.GetFlag() == INVALID || it->second.GetHop() > 2)
      {
        continue;
      }
      //三个特征：到目的地的距离、传输错误数、可用缓冲空间
      features[i][0] = 1.0 * (positionX - it->second.GetPositionX()) * (positionX - it->second.GetPositionX()) + 
                      1.0 * (positionY - it->second.GetPositionY()) * (positionY - it->second.GetPositionY());
                    
      features[i][1] = 1.0 * it->second.GetTxErrorCount();
      features[i][2] = 1.0 * it->second.GetFreeSpace();
      cout<<"feature["<<i<<"][0]------"<<features[i][0]<<endl;
      cout<<"feature["<<i<<"][1]------"<<features[i][1]<<endl;
      cout<<"feature["<<i<<"][2]------"<<features[i][2]<<endl;
      i++;
    }
    cout<<"i-------"<<i<<endl;
    n = i;
    cout<<"n-------"<<n<<endl;
    double mini[3], maxi[3];


    // normalize the features  规范化特征，计算每个特征的最大值和最小值，为了后面的“去规范化聚类中心”
    cout<<"------规范化特征------"<<endl;
    for(int j=0;j<3;j++)
    {
      mini[j] = features[0][j];
      maxi[j] = features[0][j];
      for(int i=1;i<n;i++) //
      {
        mini[j] = std::min(mini[j], features[i][j]);
        maxi[j]= std::max(maxi[j], features[i][j]);
      }

      for(int i=0;i<n;i++)
      {
        features[i][j] = (features[i][j] - mini[j]); //规范化特征，重新计算每个节点的特征值
        if(mini[j] != maxi[j]) features[i][j]/= (maxi[j] - mini[j]);
      }

    }
    

    // define an idea feature list 定义了理想情况下的特征列表
    double ideal[3];
    ideal[0] = 0.0; // minimum possible distance
    ideal[1] = 0.0; // minimum possible error
    ideal[2] = maxi[2]; // buffer empty
    

    // randomly pick k cluster heads  随机挑选K个集群的簇头，这里是分为了2个集群
    //集群簇头初始的特征值是随机分配的
    cout<<"------随机挑选K个集群的簇头------"<<endl;
    srand(time(0));
    for(int i=0;i<k;i++)
    {
      int p = rand() % n;
      for(int j=0;j<3;j++)  //k=2，上面定义的，为什么是2？ 自己定义
      {
        cluster_center[i][j] = features[p][j];
        cout<<"p------"<<p<<endl;
        cout<<"features["<<p<<"]["<<j<<"]------"<<features[p][j]<<endl;
        cout<<"cluster_center["<<i<<"]["<<j<<"]------"<<cluster_center[i][j]<<endl;
      }
    }

    int num_iterations = 3;
    // run iterations
    for(int iteration=0;iteration<=num_iterations;iteration++)
    {
      
      // assign clusters  //分配集群  计算特征值，把所有的节点分配到不同的簇头下面
      cout<<"------分配集群，第"<<iteration<<"轮------"<<endl;
      for(int i=0;i<n;i++)
      {
          double mini_dist = -1.0;
          for(int j=0;j<k;j++)
          {
            double dist = 0.0;
            for(int d=0; d<3; d++)
            {  //计算其他节点到集群簇头节点的一个值，三个特征
              dist += (features[i][d] - cluster_center[j][d]) * (features[i][d] - cluster_center[j][d]);
              
            }
            if(mini_dist == -1.0 || dist < mini_dist)
            {
              cout<<"mini_dist_before****** "<<mini_dist<<endl;
              mini_dist = dist;
              cout<<"mini_dist_after******"<<mini_dist<<endl;
              cluster_assignments[i] = j; //把每个节点分配到对应的集群里面
              cout<<"cluster_assignments["<<i<<"]------"<<cluster_assignments[i]<<endl;
            }
          }
        cout<<"cluster_assignments["<<i<<"]------"<<cluster_assignments[i]<<endl; //最后决定节点i分配到哪个集群
          
      }

      if(iteration == num_iterations) 
      {
        break;
      }
      
      // re-position cluster centers 重新定位集群中心  因为将所有的节点都分配到了对应的簇头
      // 因此将簇头的特征值做一个重新处理，主要是根据集群内部其他节点的特征值做一个平均
      cout<<"------重新定位集群中心------"<<endl;
      for(int j=0;j<k;j++)
      {
        int cnt = 0;
        for(int d=0;d<3;d++)
        {
          cluster_center[j][d] = 0.0; //集群簇头的特征值被初始化
        }


        for(int i=0;i<n;i++)
        {
          if(cluster_assignments[i] == j)
          {
            for(int d=0;d<3;d++)
            {
              cluster_center[j][d] += features[i][d]; //集群簇头的特征值定义为，它集群里面其他节点特征值的总和
              
            }
            
            cnt++; //统计该集群下面有多少个子节点
          }
          
        }
        cout<<"cluster_center["<<j<<"][0]------"<<cluster_center[j][0]<<endl;
        cout<<"cluster_center["<<j<<"][1]------"<<cluster_center[j][1]<<endl;
        cout<<"cluster_center["<<j<<"][2]------"<<cluster_center[j][2]<<endl;
        cout<<"cnt------"<<cnt<<endl;
        if(cnt)//如果子节点数目不为0，那么簇头的特征值为平均值
        {
          for(int d=0;d<3;d++)
          {
            cluster_center[j][d] /= cnt; 
          }
        }
        else //否则簇头的特征值为随机数
        {
          int p = rand() % n;
          for(int d=0;d<3;d++)
          {
            cluster_center[j][d] = features[p][d];
          }
        }

        
      }
      

    }


    // de-normalize cluster centers 去规范化聚类中心
    //对集群内部节点的特征值重新做一个计算，为什么这样算？是Kmeans算法本身定义的吗？
    cout<<"------去规范化聚类中心------"<<endl;
    for(int j=0;j<k;j++)
    {
      for(int d=0;d<3;d++)
      {
        cluster_center[j][d] *= (maxi[d] - mini[d]);
        cluster_center[j][d] += mini[d];
      }
    }
    for(int i=0;i<n;i++)
    {
      for(int d=0;d<3;d++)
      {
        features[i][d] *= (maxi[d] - mini[d]);
        features[i][d] += mini[d];
      }
    }

    // select optimal cluster 选择最优集群
    cout<<"------选择最优集群------"<<endl;
    int optimal_cluster = -1; 
    double mini_dist = -1.0;
    for(int j=0;j<k;j++)
    {
      double dist = 0;
      for(int d=0; d<3; d++)
      {
          dist += (cluster_center[j][d] - ideal[d]) * (cluster_center[j][d] - ideal[d]); 
      }
      if(mini_dist == -1.0 || dist < mini_dist)
      {
        mini_dist = dist;
        optimal_cluster = j;
      }
      cout<<"第"<<j<<"个集群的dist------"<<dist<<endl;

    }
    cout<<"optimal_cluster------"<<optimal_cluster<<endl;

    std::vector<Ipv4Address>selectedCluster;
    i=0;
    for (std::map<Ipv4Address, RoutingTableEntry>::iterator it = m_ipv4AddressEntry.begin (); it != m_ipv4AddressEntry.end (); ++it)
    {
      if(it->first.IsBroadcast() || it->first.IsLocalhost() || it->first.IsMulticast() || 
      it->first.IsSubnetDirectedBroadcast(Ipv4Mask((char *)"255.255.255.0")) || it->second.GetFlag() == INVALID || it->second.GetHop() > 2)
      {
        continue;
      }
      if(cluster_assignments[i] == optimal_cluster)
      {
        selectedCluster.push_back(it->first);
        cout<<"it->first------"<<it->first<<endl;
      }
      i++;
    }
    
    return selectedCluster;

}


void
RoutingTable::Purge ()
{
  NS_LOG_FUNCTION (this);
  if (m_ipv4AddressEntry.empty ())
    {
      return;
    }
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); )
    {
      if (i->second.GetLifeTime () < Seconds (0))
        {
          if (i->second.GetFlag () == INVALID)
            {
              std::map<Ipv4Address, RoutingTableEntry>::iterator tmp = i;
              ++i;
              m_ipv4AddressEntry.erase (tmp);
            }
          else if (i->second.GetFlag () == VALID)
            {
              NS_LOG_LOGIC ("Invalidate route with destination address " << i->first);
              i->second.Invalidate (m_badLinkLifetime);
              ++i;
            }
          else
            {
              ++i;
            }
        }
      else
        {
          ++i;
        }
    }
}

void
RoutingTable::Purge (std::map<Ipv4Address, RoutingTableEntry> &table) const
{
  NS_LOG_FUNCTION (this);
  if (table.empty ())
    {
      return;
    }
  for (std::map<Ipv4Address, RoutingTableEntry>::iterator i =
         table.begin (); i != table.end (); )
    {
      if (i->second.GetLifeTime () < Seconds (0))
        {
          if (i->second.GetFlag () == INVALID)
            {
              std::map<Ipv4Address, RoutingTableEntry>::iterator tmp = i;
              ++i;
              table.erase (tmp);
            }
          else if (i->second.GetFlag () == VALID)
            {
              NS_LOG_LOGIC ("Invalidate route with destination address " << i->first);
              i->second.Invalidate (m_badLinkLifetime);
              ++i;
            }
          else
            {
              ++i;
            }
        }
      else
        {
          ++i;
        }
    }
}

bool
RoutingTable::MarkLinkAsUnidirectional (Ipv4Address neighbor, Time blacklistTimeout)
{
  NS_LOG_FUNCTION (this << neighbor << blacklistTimeout.As (Time::S));
  std::map<Ipv4Address, RoutingTableEntry>::iterator i =
    m_ipv4AddressEntry.find (neighbor);
  if (i == m_ipv4AddressEntry.end ())
    {
      NS_LOG_LOGIC ("Mark link unidirectional to  " << neighbor << " fails; not found");
      return false;
    }
  i->second.SetUnidirectional (true);
  i->second.SetBlacklistTimeout (blacklistTimeout);
  i->second.SetRreqCnt (0);
  NS_LOG_LOGIC ("Set link to " << neighbor << " to unidirectional");
  return true;
}

void
RoutingTable::Print (Ptr<OutputStreamWrapper> stream, Time::Unit unit /* = Time::S */) const
{
  std::map<Ipv4Address, RoutingTableEntry> table = m_ipv4AddressEntry;
  Purge (table);
  std::ostream* os = stream->GetStream ();
  // Copy the current ostream state
  std::ios oldState (nullptr);
  oldState.copyfmt (*os);

  *os << std::resetiosflags (std::ios::adjustfield) << std::setiosflags (std::ios::left);
  *os << "\nAODV Routing table\n";
  *os << std::setw (16) << "Destination";
  *os << std::setw (16) << "Gateway";
  *os << std::setw (16) << "Interface";
  *os << std::setw (16) << "Flag";
  *os << std::setw (16) << "Expire";
  *os << "Hops" << std::endl;
  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator i =
         table.begin (); i != table.end (); ++i)
    {
      i->second.Print (stream, unit);
    }
  *stream->GetStream () << "\n";
}

}
}
