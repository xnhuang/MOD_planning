mutable struct FleetController
    customer_graph
    customer_graph_id :: Int
    vehicle_graph
    vehicle_graph_id :: Int
    cluster_center_trip

    routing_cost :: Float64
    link_uid :: String
    cluster_index :: Int
    cluster_centroid_link::String
    map

    routing_policy_assign
    routing_policy_rebalance
    routing_policy_rebalance_idle
    time_step :: Int

    max_clique_size
    w_wait
    balance_weight
    discount_factor :: Float64
    idle_balance
end
function FleetController( routing_cost::Float64,
                 cluster_index,
                 cluster_centroid_link::String,
                 map,
                 routing_policy_assign,
                 routing_policy_rebalance,
                 routing_policy_rebalance_idle,
                 time_step,
                 max_clique_size,
                 w_wait,
                 balance_weight,
                 discount_factor,
                 idle_balance
               )
  controller = FleetController([], [], [], [], [], routing_cost, cluster_index[:,1],
                                   cluster_centroid_link, map, routing_policy_assign,
                                   routing_policy_rebalance, routing_policy_rebalance_idle,
                                   time_step, max_clique_size, w_wait, balance_weight,
                                   discount_factor, idle_balance
                                  )
  if idle_balance == 1
    computer_cluster_center_trip(controller)
  end
  controller
end

function trip_matching(controller::FleetController, vehicle_list, customer_list, current_time, demand_distribution)
  (customer_list_out,
   vehicle_list_out, ~,
   customer_adj_mat_cell,
   customer_id_list,
   customer_vehicle_connection_mat_cell,
   vehicle_id_list) = trip_matching_pass_adj_mat(vehicle_list,
                                                 customer_list,
                                                 controller,
                                                 current_time,
                                                 demand_distribution
                                                )
  controller.customer_graph = customer_adj_mat_cell
  controller.customer_graph_id = customer_id_list
  controller.vehicle_graph = customer_vehicle_connection_mat_cell
  controller.vehicle_graph_id = vehicle_id_list
  return (customer_list_out, vehicle_list_out, controller)
end

function trip_matching_rebalance(controller::FleetController, vehicle_list::MxArray, customer_list::MxArray, current_time::Int)
    put_variable(Session, :vehicle_list, vehicle_list)
    put_variable(Session, :customer_list, customer_list)
    put_variable(Session, :current_time, current_time)
    mxcontroller = mxarray(controller)
    put_variable(Session, :mxcontroller, mxcontroller)
    eval_string(Session, "[customer_list_out,vehicle_list_out] = trip_matching_rebalance(vehicle_list,...
                                                               customer_list,...
                                                               mxcontroller.routing_cost{mxcontroller.routing_policy_rebalance}, ...
                                                               mxcontroller.link_uid,...
                                                               mxcontroller.map,...
                                                               mxcontroller.w_wait,...
                                                               current_time,...
                                                               mxcontroller.routing_policy_rebalance);")


  (customer_list_out,vehicle_list_out) =  trip_matching_rebalance(vehicle_list,
                                                                  customer_list,
                                                                  controller.routing_cost[controller.routing_policy_rebalance],
                                                                  controller.link_uid,
                                                                  controller.map,
                                                                  controller.w_wait,
                                                                  current_time,
                                                                  controller.routing_policy_rebalance)
  return (customer_list_out, vehicle_list_out,controller)
end

function trip_matching_rebalance_idle(controller::FleetController, vehicle_list, customer_list, current_time, demand_distribution)
  vehicle_list_out = trip_matching_rebalance_idle(vehicle_list,
                                                  customer_list,
                                                  controller.routing_cost[controller.routing_policy_rebalance_idle],
                                                  controller.link_uid,
                                                  controller.map,
                                                  controller.w_wait,
                                                  current_time,
                                                  controller.cluster_index,
                                                  controller.cluster_centroid_link,
                                                  demand_distribution,
                                                  controller.balance_weight,
                                                  controller.time_step,
                                                  controller.cluster_center_trip,
                                                  controller.discount_factor,
                                                  controller.routing_policy_rebalance_idle)
  return vehicle_list, controller
end


function compute_cluster_center_trip(controller::FleetController)
  controller.cluster_center_trip = [length(controller.cluster_centroid_link), length(controller.cluster_centroid_link)]
  for origin_id = 1:length(controller.cluster_centroid_link)
    origin = controller.cluster_centroid_link[origin_id]
    virtual_vehicle = Vehicle(origin_id, 4, origin, [], [])
    for destination_id = 1:length(controller.cluster_centroid_link)
      destination = controller.cluster_centroid_link(destination_id)
      if destination != origin
        virtual_customer = Customer(destination_id, destination, destination, Inf, Inf, 1)
        controller.cluster_center_trip[origin_id, destination_id] = Trip(virtual_vehicle,
                                                                         [virtual_customer],
                                                                         0, [tt_mat, fc_mat],
                                                                         controller.link_uid,
                                                                         controller.w_wait, routing_policy)
        controller.cluster_center_trip[origin_id, destination_id] = controller.cluster_center_trip[origin_id, destination_id].reconstruct_route(tt_mat,controller.link_uid,controller.map)
      end
    end
  end
end

function fleet_plan(controller::FleetController, vehicle_list::MxArray, customer_list::MxArray, current_time::Int, demand_distribution)
   (customer_list_out_match,vehicle_list_out_match,controller) = trip_matching(controller, vehicle_list, customer_list, current_time, demand_distribution);
   (customer_list_out, vehicle_list_out, controller] = trip_matching_rebalance(controller, vehicle_list_out_match, customer_list_out_match, current_time);
   if controller.idle_balance == 1
     (vehicle_list_out, controller) = trip_matching_rebalance_idle(controller, vehicle_list_out, customer_list_out, current_time, demand_distribution);
  end
end

function update_map(controller:FleetController, map)
  controller.map = map
end
