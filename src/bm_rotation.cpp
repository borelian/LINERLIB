/*
 * bm_rotation.cpp
 *
 *  Created on: Feb 23, 2014
 *      Author: berit
 */

#include "bm_rotation.h"

rotation::rotation(
    BM::data& data,
    struct BM::vesselclass& v,
    double speed,
    int num_v,
    std::vector<std::string>& calls) :
      m_class(v),
      m_speed(speed),
      m_num_v(num_v),
      m_port_calls(calls)
{
  assert(m_speed >= m_class.m_min_speed);
  assert(m_speed <= m_class.m_max_speed);
  m_draft_feasible=false;
  // Vessel Capital cost (weekly)
  m_vrc=m_class.m_OPEX_cost*7.0*m_num_v; // replaced 180 by week (7)
  m_num_round_trip=1.0; //initialised to zero such that if the costs are not
  // calculated all terms in equation 2a and 2b will return zero.
  m_num_calls=static_cast<int>(m_port_calls.size());


  const auto & graph = data.get_instance_graph();
  auto prev = m_port_calls.back();
  // set distance and costs
  m_distance = 0;
  m_cc = m_pc = 0.0;
  m_num_panama = m_num_suez = 0;
  for (auto& curr: m_port_calls) {
    const auto dist = data.get_distance_between_ports(prev, curr);
    m_distance += dist;
    const auto& port_id = data.UNLOCODE_to_vertex(curr);
    const auto& port_properties = graph.m_vertices[port_id].m_property;
    m_pc += port_properties.callCostFixed + port_properties.callCostPerFFE * static_cast<double>(m_class.m_capacity);
    if (data.use_panama_canal(prev, curr))
      m_num_panama++;
    if (data.use_suez_canal(prev, curr))
      m_num_suez++;
    m_cc = calculate_canal_cost();
    prev = curr;
  }
  m_round_trip_time = calculate_round_trip_time();
  m_bc = calculate_bunker_cost();


}

double rotation::calculate_round_trip_time() const{
  const double sail_hours= m_distance/m_speed;
  const double port_hours = 24.0*m_num_calls;
  const double total_hours= sail_hours+port_hours;
  const double rtt_days=total_hours/24.0;

  return rtt_days;
}

double rotation::fuel_burn(double speed) const{
  const double base=(speed/m_class.m_design_speed);
  const double rv=std::pow(base,3)*m_class.m_fuel_consumption;
  return rv;
}
