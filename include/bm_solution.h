//
// Created by Eduardo Moreno Araya on 11-05-23.
//

#ifndef BM_SOLUTION_H_
#define BM_SOLUTION_H_

#include "bm_data.hpp"
#include "bm_rotation.h"

enum class CargoOperation {
  Load = 0,
  Unload,
  Transshipment_leave,
  Transshipment_take,
  Move
};

// Class containing a set of rotations, and a demand. It computes the best
// flows to maximize the profit of the given rotations.
class bm_solution {
 public:
  bm_solution(
      BM::data &data);

  // Add a rotation to the current solution
  void add_rotation(const rotation& rot);

  // Compute the best flow of containers for the given rotation
  void compute_best_flows();

  // Display  operations of the current solution
  void display_port_operations();

  // Report costs
  void display_solution_objective();

  // Return flows of a given demand
  void get_route_demand(size_t demand_id);

 private:

  BM::data &data_;
  std::vector<rotation> rotations_;

  // Number of node on the network defined by rotations (note: duplicated
  // port in a given rotation count twice)
  size_t num_nodes_;
  size_t num_rotations_added_;
  std::vector<size_t> next_in_rotation_;
  std::vector<size_t> prev_in_rotation_;
  std::vector<size_t> rotation_ids_;
  std::vector<std::string> node_ids_;
  std::vector<std::string> node_UNLOCODE_;
  std::unordered_map<std::string, std::vector<size_t>> nodes_in_port_;

  // Solution of flows
  std::vector<std::vector<
    std::tuple<size_t, CargoOperation, double>>>
      cargo_operations_;
  std::vector<double> demand_satisfied_;

};

#endif //LBM_SOLUTION_H_
