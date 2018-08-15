mutable struct FleetController
    customer_graph
    customer_graph_id :: Int
    vehicle_graph
    vehicle_graph_id :: Int
    cluster_center_trip

    routing_cost :: Float64
    link_uid :: String
    cluster_index :: Int
    cluster_centroid_link
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
