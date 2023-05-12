// -p data/ports.csv -d data/Demand_Baltic.csv -t data/dist_dense.csv -q data/fleet_data.csv -f data/fleet_Baltic.csv -o nose

#include <iostream>
#include <fstream>


#include "cl_parameters.hpp"
#include "bm_reader.hpp"
#include "bm_solution.h"
#define DEBUG_instance //check if instance is parsed correctly


void display_usage ()
{
	std::cout << "Usage:" << std::endl;
	std::cout << "./APPLIKATION NAME ..." << std::endl; //Change this to the name of your binary!
	std::cout << "-p port file" << std::endl;
	std::cout << "-d demand file" << std::endl;
	std::cout << "-t distance file" << std::endl;
	std::cout << "-f fleet data file" << std::endl;
	std::cout << "-q fleet file" << std::endl;
	std::cout << "-o outputfile" << std::endl;
	std::cout << "-b timefile" <<"-? -h display usage" << std::endl;
	/* ... */
	exit ( EXIT_FAILURE );
}


void test_sol_baltic(BM::data& inst) {
  auto &fleet = inst.get_fleet();
  std::vector<std::vector<std::string>> calls;
  calls.push_back({"RULED", "FIKTK", "DEBRV", "RUKGD", "PLGDY", "DEBRV"});
  calls.push_back({"RULED", "DEBRV", "NOSVG", "SEGOT", "DEBRV"});
  calls.push_back({"DEBRV", "DKAAR"});
  std::vector<double> speeds = {11.1944, 15.4954, 10.0};
  std::vector<int> vesselType = {0, 1, 0};
  std::vector<int> numVessel = {3, 2, 1};

  bm_solution solution(inst);

  for (size_t rotId = 0; rotId < calls.size(); rotId++) {
    std::cout << "Rotation " << rotId << ": [";
    for (auto &call : calls[rotId])
      std::cout << " " << call;
    std::cout << "] with " << numVessel[rotId]
              << " vessels of capacity " << fleet[vesselType[rotId]].m_capacity
              << " at speed " << speeds[rotId]
              << "\n";
    rotation rot1(inst, fleet[vesselType[rotId]], speeds[rotId],
                  numVessel[rotId],calls[rotId]);
    std::cout << "\n\tDistance: " << rot1.getDistance()
              << "\n\tWeeks: " << rot1.getRoundTripTime() / 7.0
              << "\n\tbunker cost: " << rot1.get_bunker_cost()/numVessel[rotId]
              << "\n\tPort call cost: " << rot1.get_port_call_cost()
              << "\n\tVessel Class cost: " << rot1.get_vessel_running_cost()
              << "\n\tCanal cost: " << rot1.get_canal_cost()
              <<  std::endl;
    solution.add_rotation(rot1);
  }

  solution.compute_best_flows();
  solution.display_port_operations();
  std::cout << "Done:" << std::endl;
}

void test_sol_WAF(BM::data& inst) {
  auto &fleet = inst.get_fleet();
  std::vector<std::vector<std::string>> calls;
  calls.push_back({"ESALG", "SNDKR", "ESALG", "BJCOO", "CDMAT", "CDBOA"});
  calls.push_back({"ESALG", "BJCOO", "AOLOB", "CGPNR", "CIABJ"});
  calls.push_back({"ESALG", "GNCKY", "LRMLW", "ESALG", "SLFNA", "GHTKD",
                   "TGLFW", "GALBV", "AOLAD"});
  calls.push_back({"CMDLA", "NGAPP"});
  calls.push_back({"ESALG", "CIABJ", "TGLFW", "ESALG", "NGAPP"});
  calls.push_back({"ESALG", "AOLAD", "NGAPP", "GHTKD"});
  calls.push_back({"ESALG", "NGAPP"});
  calls.push_back({"CIABJ", "NGAPP", "ESALG"});
  std::vector<double> speeds = {10.6172, 11.6375, 13.1052, 10.0, 13.259,
                                10.7567, 13.1579, 10.0267};
  std::vector<int> vesselType = {1, 1, 0, 0, 1, 0, 1, 1};
  std::vector<int> numVessel = {7, 5, 7, 1, 6, 5, 3, 4};

  bm_solution solution(inst);

  for (size_t rotId = 0; rotId < calls.size(); rotId++) {
    std::cout << "Rotation " << rotId << ": [";
    for (auto &call : calls[rotId])
      std::cout << " " << call;
    std::cout << "] with " << numVessel[rotId]
              << " vessels of capacity " << fleet[vesselType[rotId]].m_capacity
              << " at speed " << speeds[rotId]
              << "\n";
    rotation rot1(inst, fleet[vesselType[rotId]], speeds[rotId],
                  numVessel[rotId],calls[rotId]);
    std::cout << "\n\tDistance: " << rot1.getDistance()
              << "\n\tWeeks: " << rot1.getRoundTripTime() / 7.0
              << "\n\tbunker cost: " << rot1.get_bunker_cost()/numVessel[rotId]
              << "\n\tPort call cost: " << rot1.get_port_call_cost()
              << "\n\tVessel Class cost: " << rot1.get_vessel_running_cost()
              << "\n\tCanal cost: " << rot1.get_canal_cost()
              <<  std::endl;
    solution.add_rotation(rot1);
  }

  solution.compute_best_flows();
  solution.display_port_operations();
  solution.display_solution_objective();
  std::cout << "Done:" << std::endl;
}


int main ( int argc, char* argv[] )
{
	cl_parameters cl_params; //command line parameters

	if ( argc > 13 ) //If you extend the number of cases and arguments please increase this number
	{
		std::cerr << "Wrong Number of arguments" << std::endl;
		display_usage();
		exit ( EXIT_FAILURE );
		
	}
	
	
	for ( int i = 1; i+1 < argc; i+=2 )
	{
		switch ( argv[i][1] )
		{
			case 'p':
				cl_params.port_file = argv[i+1];
				break;
			case 'd':
				cl_params.demand_file = argv[i+1];
				break;
			case 't':
				cl_params.distance_file = argv[i+1];
				break;
			case 'f':
				cl_params.fleet_file = argv[i+1];
				break;
			case 'q':
				cl_params.fleet_data_file = argv[i+1];
				break;
			case 'o':
				cl_params.output_file = argv[i+1];
				break;
			case 'b':
				cl_params.time_file = argv[i+1];
				break;
			case 'h':
			case '?':
				display_usage();
				break;
			default:
				/* Shouldn't get here */
				std::cerr << "Unknown problem" << std::endl;
				display_usage();
				break;
		}
	}

#ifdef DEBUG_instance
	std::cout << "Command line parameters: " << std::endl;
	cl_params.print();
#endif
	//parse benchmark instance
	BM::reader bm_reader ( cl_params );
	
	BM::data instanceBM=bm_reader.parse_instance();
#ifdef DEBUG_instance
	instanceBM.print_inst_graph ( "graph_inst.dot" );
	instanceBM.print_demands_by_id();
	instanceBM.print_fleet();
#endif
  instanceBM.print_distances_by_id();

  test_sol_WAF(instanceBM);
  //test_sol_baltic(instanceBM);
return 0;
}
