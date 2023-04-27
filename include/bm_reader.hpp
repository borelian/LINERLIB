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

#include "BM_Graph.hpp"
#include "bm_data.hpp"
#include "cl_parameters.hpp"
#include "bm_rotation.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <unordered_map>
#ifndef BM_READER_HPP
#define BM_READER_HPP


namespace BM
{
class reader
{
public:
	explicit reader ( cl_parameters& cmdline ):
        m_file_names(cmdline) {};

	BM::data parse_instance();
	void parse_ports_to_vertices();
	void parse_distances_to_edges();
	void parse_demands();
	void parse_vessel_classes();
	void retrieve_instance_data();
	void make_port_call_costs();
	void make_vessel_class_bunker_consumption();

	Graph get_graph() const
	{
		return m_graph;
	}

	Graph get_instance_graph() const
	{
		return m_instance_graph;
	}

	std::vector<vesselclass> get_fleet() const
    {
		return m_fleet;
    }

	demand_map_t get_demands() const
	{
		return m_demand_map;
	}

	demand_idx_map_t get_demands_by_id() const
	{
		return m_instance_dem;
	}

	data_map_t get_distances_by_id() const
	{
		return m_instance_dist;
	}

	std::map<std::string, vertex_descriptor> get_UNLOCODE_to_vertex() const
    {
		return m_UNLOCODE_to_vertex;
    }

	cl_parameters get_file_names() const
	{
		return m_file_names;
	}

	port_map_t get_ports() const
	{
		return m_instance_ports;
	}

//	void read_OPTIMIZEDinstance(const char* networkfile);

private:
	cl_parameters m_file_names;
	Graph m_graph; //a graph representation of the vertices
	demand_map_t m_demand_map;
	std::vector<vesselclass> m_fleet;
	std::map<std::string, vertex_descriptor> m_UNLOCODE_to_vertex;
	data_map_t m_instance_dist;
	demand_idx_map_t m_instance_dem;
	port_map_t m_instance_ports;
	Graph m_instance_graph;

	vertex_descriptor UNLOCODE_to_vertex ( std::string& UNLOCODE );
	//vesselclass get_vessel_class(int capacity);
	void test_bundled_properties();

	std::pair<double, double> m_suez;

	std::pair<double, double> m_panama;

	//vector<rotation> m_rotations; //rotation variables for validator
};
}
#endif
