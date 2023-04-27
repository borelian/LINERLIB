/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include <iostream>     // std::cout, std::endl
#include <iterator>     // ostream_operator
#include <cfloat>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "bm_data.hpp"

#define DEBUG_OUT

void BM::data::print_demands()
{

  demand_map_t::iterator it, itend;
  itend=m_demands.end();
  for ( it=m_demands.begin(); it!=itend; it++ )
    {
      std::pair<vertex_descriptor, vertex_descriptor> key=it->first;
      std::tuple<double, double, double> value =it->second;
      std::cout << m_graph[key.first].UNLOCODE << "-" << m_graph[key.second].UNLOCODE << " FFE: " <<
           get<0> ( value ) << " $: " << get<1> ( value ) << " TT: " << get<2> ( value ) << std::endl;
    }
}

void BM::data::print_demands_by_id()
{
std::string unl_1, unl_2;
  demand_idx_map_t::iterator it, itend;
  port_map_t::iterator p_it, p_itend;
  itend=m_demands_by_id.end();
  p_itend=m_ports.end();
  for ( it=m_demands_by_id.begin(); it!=itend; it++ )
    {
      std::pair<int, int> key=it->first;
      std::tuple<double, double, double> value =it->second;
      p_it=m_ports.find(key.first);
      if(p_it!=p_itend) unl_1=p_it->second;
      p_it=m_ports.find(key.second);
      if(p_it!=p_itend) unl_2=p_it->second;
      std::cout << unl_1 << "-" << unl_2 << " FFE: " <<
      get<0> ( value ) << " $: " << get<1> ( value ) << " TT: " << get<2> ( value ) << std::endl;
    }
}

void BM::data::print_distances_by_id()
{
std::string unl_1, unl_2;
  data_map_t::iterator it, itend;
  port_map_t::iterator p_it, p_itend;
  itend=m_distances_by_id.end();
  p_itend=m_ports.end();
  for ( it=m_distances_by_id.begin(); it!=itend; it++ )
    {
      std::pair<int, int> key=it->first;
      std::tuple<double, bool, bool> value =it->second;
      p_it=m_ports.find(key.first);
      if(p_it!=p_itend) unl_1=p_it->second;
      p_it=m_ports.find(key.second);
      if(p_it!=p_itend) unl_2=p_it->second;
      std::cout << unl_1 << "(" << key.first << ")-"
                << unl_2 << "(" << key.second << ") nm:" <<
      get<0> ( value ) << " is suez: " << get<1> ( value ) << " is panama: " << get<2> ( value ) << std::endl;
    }
}



void BM::data::print_graph ( std::string filename )
{
  node_writer nw ( m_graph );
  edge_writer ew ( m_graph );
  std::ofstream of ( filename.c_str() );
  write_graphviz ( of, m_graph,nw, ew );
}

void BM::data::print_inst_graph ( std::string filename )
{
  node_writer nw ( m_instance_graph );
  edge_writer ew ( m_instance_graph );
  std::ofstream of ( filename.c_str() );
  write_graphviz ( of, m_instance_graph,nw, ew );
}

void BM::data::print_fleet()
{
  std::vector<vesselclass>::iterator it, it_end;
  it_end=m_fleet.end();
  for ( it=m_fleet.begin(); it != it_end; it++ )
    {
      std::cout << "Name: " <<  it->m_name << " FFE: " <<  it->m_capacity << " OPEX: " <<  it->m_OPEX_cost <<" design speed: " <<   it->m_design_speed << " fuel consumption " <<  it->m_fuel_consumption <<" idle consumption " <<  it->m_idle_consumption <<" quantity " << it->m_quantity << " suez cost " << it->m_suez_cost << " panama cost " << it->m_panama_cost << std::endl;
    }
}

void BM::data::print_ports()
{
  port_map_t::iterator it, itend;
  itend=m_ports.end();
  for ( it=m_ports.begin(); it!=itend; it++ )
    {
      std::cout << it->first << "-" << it->second<< std::endl;
    }
}

void BM::data::create_inverse_port_map() {
  for (auto& port: m_ports) {
    m_UNLOCODE_to_vertex.insert(std::make_pair(port.second, port.first));
  }
}

BM::vertex_descriptor BM::data::UNLOCODE_to_vertex ( std::string& UNLOCODE )
{
  vertex_descriptor rv;
  std::map<std::string, vertex_descriptor>::iterator it, it_end;
  it_end=m_UNLOCODE_to_vertex.end();
  //find the vertices
  it=m_UNLOCODE_to_vertex.find ( UNLOCODE );
  if ( it != it_end )
  {
    rv=it->second;
  }
  else
  {
    std::cout << "The port " << UNLOCODE << " is not defined!" << std::endl;
    return -1;
    //exit ( 0 );
  }
  return rv;
}

double BM::data::get_distance_between_ports(
    std::string& UNLOCODE1,
    std::string& UNLOCODE2) {
  auto id_from = UNLOCODE_to_vertex(UNLOCODE1);
  auto id_dest =  UNLOCODE_to_vertex(UNLOCODE2);
  if (auto search = m_distances_by_id.find({id_from, id_dest});
  search != m_distances_by_id.end()) {
    auto& dist = std::get<0>(search->second);
    return dist;
  }
  return DBL_MAX;
}

bool BM::data::use_suez_canal(
    std::string& UNLOCODE1,
    std::string& UNLOCODE2) {
  auto id_from = UNLOCODE_to_vertex(UNLOCODE1);
  auto id_dest =  UNLOCODE_to_vertex(UNLOCODE2);
  if (auto search = m_distances_by_id.find({id_from, id_dest});
      search != m_distances_by_id.end()) {
    auto& dist = std::get<2>(search->second);
    return dist;
  }
}

bool BM::data::use_panama_canal(
    std::string& UNLOCODE1,
    std::string& UNLOCODE2) {
  auto id_from = UNLOCODE_to_vertex(UNLOCODE1);
  auto id_dest =  UNLOCODE_to_vertex(UNLOCODE2);
  if (auto search = m_distances_by_id.find({id_from, id_dest});
      search != m_distances_by_id.end()) {
    auto& dist = std::get<1>(search->second);
    return dist;
  }
}