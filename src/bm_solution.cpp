//
// Created by Eduardo Moreno Araya on 11-05-23.
//

#include "bm_solution.h"
#include "gurobi_c++.h"

bm_solution::bm_solution(
    BM::data &data) :
    data_(data),
    num_nodes_(0),
    num_rotations_added_(0)
{
}

void bm_solution::add_rotation(
    const rotation& rot) {
  // Add rotation
  rotations_.emplace_back(rot);
  num_rotations_added_++;

  // get calls and add
  const auto& calls = rot.getPortCalls();
  assert(calls.size() > 1);
  const size_t cur_nnodes = num_nodes_;
  num_nodes_ += calls.size();
  next_in_rotation_.resize(num_nodes_);
  prev_in_rotation_.resize(num_nodes_);
  // rotation_ids_.resize(num_nodes_);
  for (size_t i = 0 ; i < calls.size() ; i++)
  {
    const std::string id = "Rot-"
        + std::to_string(num_rotations_added_-1)
        +"-Pos-"+std::to_string(i)
        +"-"+calls[i];
    std::cout << "Added node " << cur_nnodes+i << " id=" << id << "\n";
    node_ids_.emplace_back(id);
    rotation_ids_.emplace_back(num_rotations_added_-1);
    node_UNLOCODE_.emplace_back(calls[i]);
    nodes_in_port_[calls[i]].emplace_back(cur_nnodes+i);
    if (i > 0) {
      next_in_rotation_[cur_nnodes+i-1] = (cur_nnodes+i);
      prev_in_rotation_[cur_nnodes+i] = (cur_nnodes+i-1);
    } else {
      next_in_rotation_[num_nodes_-1] = (cur_nnodes);
      prev_in_rotation_[cur_nnodes] = (num_nodes_-1);
    }
  }
}

void bm_solution::compute_best_flows() {
  auto *env = new GRBEnv();
  GRBModel model_(env);

  const auto &ports = data_.get_ports();
  auto demand_map = data_.get_demands_by_id();
  const size_t num_demand = demand_map.size();
  std::vector<bool> demand_is_feasible(num_demand,true);
  // Variables
  auto **flow_to_next_in_rotation = new GRBVar *[num_demand];
  for (size_t d = 0; d < num_demand; d++)
    flow_to_next_in_rotation[d] = model_.addVars(num_nodes_);

  auto demand_flow = model_.addVars(num_demand);
  auto demand_flow_load = new GRBVar *[num_demand];
  auto demand_flow_unload = new GRBVar *[num_demand];
  auto ****flow_transhipment = new GRBVar ***[num_demand];

  for (auto it_dem = demand_map.begin(); it_dem != demand_map.end();
       ++it_dem) {
    const size_t dem_id = std::distance(demand_map.begin(), it_dem);

    // Create load arcs and constraints
    auto origin = ports.find(it_dem->first.first)->second;
    auto dest = ports.find(it_dem->first.second)->second;
    std::cout << "Adding for demand id=" << dem_id
              << " [" << origin << "-" << dest << "]\n";

    auto origin_it = nodes_in_port_.find(origin);
    auto dest_it = nodes_in_port_.find(dest);

    bool found = true;
    if (origin_it == nodes_in_port_.end()) {
      std::cout << "\tOrigin node not found. Skip this demand\n";
      found = false;
    }
    if (dest_it == nodes_in_port_.end()) {
      std::cout << "\tDest node not found. Skip this demand\n";
      found = false;
    }
    if (found == false) {
      demand_flow[dem_id].set(GRB_DoubleAttr_UB, 0.0);
      demand_is_feasible[dem_id] = false;
      continue;
    }

    // set max demand and profit
    const auto max_demand = std::get<0>(it_dem->second);
    const auto profit = std::get<1>(it_dem->second);
    demand_flow[dem_id].set(GRB_DoubleAttr_UB, max_demand);
    demand_flow[dem_id].set(GRB_DoubleAttr_Obj, profit);

    // Set load and unload vars
    const auto &port_id_origin = data_.UNLOCODE_to_vertex(origin);
    const auto &port_id_dest = data_.UNLOCODE_to_vertex(dest);
    const auto port_load_cost = data_.get_instance_graph().m_vertices[port_id_origin]
        .m_property.localMoveCost;
    const auto port_unload_cost = data_.get_instance_graph().m_vertices[port_id_dest]
        .m_property.localMoveCost;

    const auto num_port_load_nodes = origin_it->second.size();
    const auto num_port_unload_nodes = dest_it->second.size();

    std::cout << "\tOrigin node has " << num_port_load_nodes
              << " potential input nodes for loading with cost "
              << port_load_cost << "\n";
    std::cout << "\tDest node has " << num_port_unload_nodes
              << " potential nodes for unloading with cost "
              << port_unload_cost << "\n";

    demand_flow_load[dem_id] = model_.addVars(num_port_load_nodes);
    demand_flow_unload[dem_id] = model_.addVars(num_port_unload_nodes);

    // Constraint demand = sum(load) = sum(unload)
    GRBLinExpr flow_demand_load = 0;
    for (size_t i = 0; i < num_port_load_nodes; i++) {
      demand_flow_load[dem_id][i].set(GRB_DoubleAttr_Obj, -port_load_cost);
      flow_demand_load += demand_flow_load[dem_id][i];
    }
    model_.addConstr(flow_demand_load == demand_flow[dem_id]);

    GRBLinExpr flow_demand_unload = 0;
    for (size_t i = 0; i < num_port_unload_nodes; i++) {
      demand_flow_unload[dem_id][i].set(GRB_DoubleAttr_Obj,
                                        -port_unload_cost);
      flow_demand_unload += demand_flow_unload[dem_id][i];
    }
    model_.addConstr(flow_demand_unload == demand_flow[dem_id]);

    // Add port variables and constraints
    std::cout << "\tTranshipment nodes:\n";
    const size_t num_ports = nodes_in_port_.size();

    flow_transhipment[dem_id] = new GRBVar **[num_ports];
    for (auto iter = nodes_in_port_.begin(); iter != nodes_in_port_.end();
         ++iter) {
      const size_t port_id = std::distance(nodes_in_port_.begin(), iter);
      auto port_name = iter->first;

      const auto &port_desc = data_.UNLOCODE_to_vertex(port_name);
      const auto
          &port_properties = data_.get_instance_graph().m_vertices[port_desc]
              .m_property;
      auto port_transshipment_cost = port_properties.transhipmentCost;
      if (port_transshipment_cost < 1e-6) port_transshipment_cost = 1e-6;
      const size_t num_nodes_in_port = iter->second.size();
      std::cout << "\tPort " << port_name << " has " << num_nodes_in_port <<
                " nodes:";
      flow_transhipment[dem_id][port_id] = new GRBVar *[num_nodes_in_port];
      for (size_t i = 0; i < num_nodes_in_port; i++) {
        flow_transhipment[dem_id][port_id][i] =
            model_.addVars(num_nodes_in_port);
        for (size_t j = 0; j < num_nodes_in_port; j++)
          flow_transhipment[dem_id][port_id][i][j].set(GRB_DoubleAttr_Obj,
                                                       -port_transshipment_cost);
      }
      // Traverse all port and its corresponding nodes (so all nodes are
      // visited)
      // Add flow constraint
      for (size_t id_1 = 0; id_1 < num_nodes_in_port; id_1++) {
        const auto node_id = iter->second[id_1];
        std::cout << " " << node_id;
        GRBLinExpr flow_out = flow_to_next_in_rotation[dem_id][node_id];
        GRBLinExpr flow_in =
            flow_to_next_in_rotation[dem_id][prev_in_rotation_[node_id]];
        if (port_name == origin)
          flow_in += demand_flow_load[dem_id][id_1];
        if (port_name == dest)
          flow_out += demand_flow_unload[dem_id][id_1];
        // Transshipment
        for (size_t id_2 = 0; id_2 < num_nodes_in_port; id_2++) {
          if (id_2 == id_1) continue;
          flow_out += flow_transhipment[dem_id][port_id][id_1][id_2];
          flow_in += flow_transhipment[dem_id][port_id][id_2][id_1];
        }
        model_.addConstr(flow_in == flow_out);
      }
      std::cout << "\n";
    }
  }
  // Capacity constraints
  for (size_t node_id = 0 ; node_id < num_nodes_ ; node_id++) {
    const auto rotation_id = rotation_ids_[node_id];
    const auto capacity = rotations_[rotation_id].getClass().m_capacity;
    GRBLinExpr lhs = 0;
    for (size_t d = 0; d < num_demand; d++)
      lhs += flow_to_next_in_rotation[d][node_id];
    model_.addConstr(lhs <= capacity);
  }

  model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
  model_.update();
  model_.optimize();

  // Save solution
  demand_satisfied_.resize(num_demand, 0.0);
  cargo_operations_.resize(num_nodes_);

  // Demand satisfied
  for (auto it_dem = demand_map.begin(); it_dem != demand_map.end();
       ++it_dem) {
    const size_t dem_id = std::distance(demand_map.begin(), it_dem);
    demand_satisfied_[dem_id] = demand_flow[dem_id].get(GRB_DoubleAttr_X);
  }

  // cargo operations
  for (auto iter = nodes_in_port_.begin(); iter != nodes_in_port_.end();
       ++iter) {
    const size_t port_id = std::distance(nodes_in_port_.begin(), iter);
    auto port_name = iter->first;
    const size_t num_nodes_in_port = iter->second.size();
    for (size_t i = 0; i < num_nodes_in_port; ++i) {
      const auto node_id = iter->second[i];
      for (auto it_dem = demand_map.begin(); it_dem != demand_map.end();
           ++it_dem) {
        const size_t dem_id = std::distance(demand_map.begin(), it_dem);
        if (demand_is_feasible[dem_id] == false) continue;

        // Flow to next port in rotation
        const auto flow_next = flow_to_next_in_rotation[dem_id][node_id].get
            (GRB_DoubleAttr_X);
        if (flow_next > 0) {
          auto tup = std::make_tuple(dem_id, CargoOperation::Move, flow_next);
          cargo_operations_[node_id].push_back(tup);
        }

        // Load/Unload
        auto dem_origin = ports.find(it_dem->first.first)->second;
        if (port_name == dem_origin) {
          const auto load = demand_flow_load[dem_id][i].get(GRB_DoubleAttr_X);
          if (load > 0) {
            auto tup = std::make_tuple(dem_id, CargoOperation::Load, load);
            cargo_operations_[node_id].push_back(tup);
          }
        }
        auto dem_dest = ports.find(it_dem->first.second)->second;
        if (port_name == dem_dest) {
          const auto load = demand_flow_unload[dem_id][i].get(GRB_DoubleAttr_X);
          if (load > 0) {
            auto tup = std::make_tuple(dem_id, CargoOperation::Unload, load);
            cargo_operations_[node_id].push_back(tup);
          }
        }
        // Transhipment
        for (size_t j = 0; j < num_nodes_in_port; ++j) {
          if (i == j) continue;
          const auto transported_leave =
              flow_transhipment[dem_id][port_id][i][j].get
              (GRB_DoubleAttr_X);
          const auto transported_take =
              flow_transhipment[dem_id][port_id][j][i].get
                  (GRB_DoubleAttr_X);
          if (transported_leave > 0) {
            auto tup = std::make_tuple(dem_id,
                                       CargoOperation::Transshipment_leave,
                                       transported_leave);
            cargo_operations_[node_id].push_back(tup);
          }
          if (transported_take > 0) {
            auto tup = std::make_tuple(dem_id,
                                       CargoOperation::Transshipment_take,
                                       transported_take);
            cargo_operations_[node_id].push_back(tup);
          }
        }
      }
    }
  }

  // OUTPUT
  if (0) {
    std::cout << "SOLUTION:\n";
    for (auto it_dem = demand_map.begin(); it_dem != demand_map.end();
         ++it_dem) {
      const size_t dem_id = std::distance(demand_map.begin(), it_dem);
      auto origin = ports.find(it_dem->first.first)->second;
      auto dest = ports.find(it_dem->first.second)->second;
      std::cout << "Demand id=" << dem_id
                << " [" << origin << "-" << dest << "]\n";
      const auto transported = demand_flow[dem_id].get(GRB_DoubleAttr_X);
      std::cout << "\tTransported " << transported
                << " of " << std::get<0>(it_dem->second) << "\n";
      if (transported > 0) {
        auto origin_it = nodes_in_port_.find(origin);
        const auto num_port_load_nodes = origin_it->second.size();
        for (size_t i = 0 ; i < num_port_load_nodes ; i++) {
          const auto node_id = origin_it->second[i];
          const auto load = demand_flow_load[dem_id][i].get(GRB_DoubleAttr_X);
          if (load >= 1.0) {
            std::cout << "Loaded " << load << " at node " << node_id << "\n";
          }
        }
      }
    }
    for (size_t node_id = 0 ; node_id < num_nodes_ ; node_id++) {
      std::cout << "Arc from " << node_id << " (" << node_ids_[node_id]
      << ") to " << next_in_rotation_[node_id] << " ("
      << node_ids_[next_in_rotation_[node_id]] << ")\n";
      for (auto it_dem = demand_map.begin(); it_dem != demand_map.end();
           ++it_dem) {
        const size_t dem_id = std::distance(demand_map.begin(), it_dem);
        auto origin = ports.find(it_dem->first.first)->second;
        auto dest = ports.find(it_dem->first.second)->second;
        const auto transported = flow_to_next_in_rotation[dem_id][node_id].get
            (GRB_DoubleAttr_X);
        if (transported >= 1)
          std::cout << "\tTransported " << transported << " for demand=" << dem_id
                    << " (" << origin << "-" << dest << ")\n";
      }
    }
    std::cout << "Transhipments:\n";
    for (auto iter = nodes_in_port_.begin(); iter != nodes_in_port_.end();
         ++iter) {
      const size_t port_id = std::distance(nodes_in_port_.begin(), iter);
      auto port_name = iter->first;
      const size_t num_nodes_in_port = iter->second.size();
      for (size_t i = 0 ; i < num_nodes_in_port ; ++i) {
        for (size_t j = 0; j < num_nodes_in_port; ++j) {
          if (i == j) continue;
          for (auto it_dem = demand_map.begin(); it_dem != demand_map.end();
               ++it_dem) {
            const size_t dem_id = std::distance(demand_map.begin(), it_dem);
            if (demand_is_feasible[dem_id] == false) continue;
            const auto
                transported = flow_transhipment[dem_id][port_id][i][j].get
                (GRB_DoubleAttr_X);
            if (transported >= 1) {
              auto origin = ports.find(it_dem->first.first)->second;
              auto dest = ports.find(it_dem->first.second)->second;
              std::cout << "Port " << port_name << ": " << transported
                        << " from demand " << dem_id << " (" << origin << "-"
                        << dest
                        << ")\n";
            }
          }
        }
      }
    }
  }
}

void bm_solution::display_port_operations() {
  double total_profit = 0;
  double total_moved_containers = 0;
  double total_demand = 0;
  double total_cost_load = 0;
  double total_cost_unload = 0;
  double total_cost_transhipment = 0;
  for (size_t i = 0 ; i < num_nodes_ ; i++) {
    std::cout << "Node " << i << ":\t" << node_ids_[i] << std::endl;
    double total_cargo = 0;
    const auto &port_id = data_.UNLOCODE_to_vertex(node_UNLOCODE_[i]);
    const auto port_load_cost = data_.get_instance_graph().m_vertices[port_id]
        .m_property.localMoveCost;
    const auto port_trans_cost = data_.get_instance_graph().m_vertices[port_id]
        .m_property.transhipmentCost;
    for (auto op : cargo_operations_[i]) {
      auto it_dem = data_.get_demands_by_id().begin();
      std::advance(it_dem, std::get<0>(op));
      auto dem_origin = data_.get_ports().find(it_dem->first.first)->second;
      auto dem_dest = data_.get_ports().find(it_dem->first.second)->second;
      std::cout << "\t";
      switch (std::get<1>(op)) {
        case CargoOperation::Load:
          std::cout <<  "Load ";
          total_cost_load += std::get<2>(op) * port_load_cost;
          break;
        case CargoOperation::Unload:
          std::cout << "Unload ";
          total_cost_unload += std::get<2>(op) * port_load_cost;
          total_profit += std::get<2>(op) *  std::get<1>(it_dem->second);
          total_demand += std::get<2>(op);
          break;
        case CargoOperation::Transshipment_leave:
          std::cout << "Leaving for transhipment ";
          total_cost_transhipment += std::get<2>(op) * port_trans_cost * 0.5;
          break;
        case CargoOperation::Transshipment_take:
          std::cout << "Taking for transhipment ";
          total_cost_transhipment += std::get<2>(op) * port_trans_cost * 0.5;
          break;
        case CargoOperation::Move:
          std::cout << "Moving ";
          total_cargo += std::get<2>(op);
          total_moved_containers += std::get<2>(op);
          break;
      }
      std::cout << std::get<2>(op) << " containers of demand "
//      << std::get<0>(op) << std::endl;
      << dem_origin << "-" << dem_dest << "\n";
    }
    const auto rotation_id = rotation_ids_[i];
    const auto capacity = rotations_[rotation_id].getClass().m_capacity;
    std::cout << "\t\tTOTAL CARGO TO NEXT STOP: " << total_cargo
    << " of " << capacity << std::endl;
  }
}

void bm_solution::display_solution_objective() {
  double call_cost = 0;
  double bunker_cost = 0;
  double vessel_cost = 0;
  double canal_cost = 0;
  for (auto &rot : rotations_) {
    call_cost += rot.get_port_call_cost();
    bunker_cost += rot.get_bunker_cost()/rot.getNumV();
    vessel_cost += rot.get_vessel_running_cost();
    canal_cost += rot.get_canal_cost();
  }
  double revenue = 0;
  // Demand
  auto it_dem = data_.get_demands_by_id().begin();
  for (int i = 0; i < demand_satisfied_.size(); i++) {
    const auto profit = std::get<1>(it_dem->second);
    revenue += profit * demand_satisfied_[i];
    it_dem++;
  }
  double load_unload_cost = 0;
  double transshipment_cost = 0;
  // Handling cost
  for (int i = 0; i < num_nodes_; i++) {
    auto port_name = node_UNLOCODE_[i];
    const auto port_id = data_.UNLOCODE_to_vertex(port_name);
    const auto port_load_cost = data_.get_instance_graph().m_vertices[port_id]
        .m_property.localMoveCost;
    const auto port_trans_cost = data_.get_instance_graph().m_vertices[port_id]
        .m_property.transhipmentCost;
    for (auto &op : cargo_operations_[i]) {
      switch (std::get<1>(op)) {
        case CargoOperation::Load:
        case CargoOperation::Unload:
          load_unload_cost += port_load_cost * std::get<2>(op);
          break;
        case CargoOperation::Transshipment_take:
        case CargoOperation::Transshipment_leave:
          transshipment_cost += 0.5 * port_trans_cost * std::get<2>(op);
          break;
        case CargoOperation::Move:
          break;
      }
    }
  }
  std::cout << "\nTOTAL COSTS:"
            << "\nVessel Class cost: " << vessel_cost
            << "\nBunker cost: " << bunker_cost
            << "\nPort Call Cost: " << call_cost
            << "\nCanal costs: " << canal_cost
            << "\nRevenue: " << revenue
            << "\nHandling cost: " << load_unload_cost + transshipment_cost
            << "\n\tLoad/Unload cost: " << load_unload_cost
            << "\n\tTransshipment cost: " << transshipment_cost
            << "\nTOTAL SUM: " << revenue - vessel_cost - bunker_cost -
            call_cost - canal_cost - load_unload_cost - transshipment_cost
  << std::endl;
}
