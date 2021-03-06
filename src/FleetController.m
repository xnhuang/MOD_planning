classdef FleetController
    %FLEET_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % private variables for efficiency
        customer_graph
        customer_graph_id
        vehicle_graph
        vehicle_graph_id
        cluster_center_trip
        
        % model parameters
        routing_cost
        link_uid
        cluster_index
        cluster_centroid_link
        map
        
        routing_policy_assign
        routing_policy_rebalance
        routing_policy_rebalance_idle
        time_step
        
        max_clique_size
        w_wait
        balance_weight
        discount_factor
        idle_balance
    end
    
    methods
        function obj = FleetController(...
            routing_cost,...
            cluster_index,...
            cluster_centroid_link,...
            map,...
            routing_policy_assign,...
            routing_policy_rebalance,...
            routing_policy_rebalance_idle,...
            time_step,...
            max_clique_size,...
            w_wait,...
            balance_weight,...
            discount_factor,...
            idle_balance)
            % constructor of FleetController class
            % initialize controller parameters and internal variables
            
            obj.customer_graph = {};
            obj.customer_graph_id = [];
            obj.vehicle_graph = {};
            obj.vehicle_graph_id = [];
            obj.cluster_center_trip = {};
            obj.routing_cost = routing_cost;
            obj.link_uid = cluster_index(:,1);
            obj.cluster_index = cluster_index(:,2);
            obj.cluster_centroid_link = cluster_centroid_link;
            obj.map = map;

            obj.routing_policy_assign = routing_policy_assign;
            obj.routing_policy_rebalance = routing_policy_rebalance;
            obj.routing_policy_rebalance_idle = routing_policy_rebalance_idle;
            obj.time_step = time_step;

            obj.max_clique_size = max_clique_size;
            obj.w_wait = w_wait;
            obj.balance_weight = balance_weight;
            obj.discount_factor = discount_factor;
            obj.idle_balance = idle_balance;
            
            if obj.idle_balance == 1
                obj = obj.compute_cluster_center_trip();
            end
        end
        
        
        function [customer_list_out,vehicle_list_out,obj] = trip_matching(...
                obj,...
                vehicle_list, ...
                customer_list, ...
                current_time, ...
                demand_distribution)
            
            [customer_list_out,...
            vehicle_list_out,...
            ~,...
            customer_adj_mat_cell,...
            customer_id_list, ...
            customer_vehicle_connection_mat_cell, ...
            vehicle_id_list]...
                = trip_matching_pass_adj_mat(...
                    vehicle_list, ...
                    customer_list, ...
                    obj.routing_cost{obj.routing_policy_assign}, ...
                    obj.link_uid, ...
                    obj.map, ...
                    obj.w_wait, ...
                    obj.max_clique_size, ...
                    current_time, ...
                    obj.cluster_index, ...
                    demand_distribution, ...
                    obj.balance_weight, ...
                    obj.time_step, ...
                    obj.discount_factor, ...
                    obj.customer_graph, ...
                    obj.customer_graph_id, ...
                    obj.vehicle_graph, ...
                    obj.vehicle_graph_id ,...
                    obj.routing_policy_assign);
            obj.customer_graph = customer_adj_mat_cell;
            obj.customer_graph_id = customer_id_list;
            obj.vehicle_graph = customer_vehicle_connection_mat_cell;
            obj.vehicle_graph_id = vehicle_id_list;
        end
        
        
        function [customer_list_out,vehicle_list_out,obj] = trip_matching_rebalance(...
                obj,...
                vehicle_list, ...
                customer_list, ...
                current_time)
            
                [customer_list_out,vehicle_list_out] ...
                    = trip_matching_rebalance(...
                        vehicle_list,...
                        customer_list,...
                        obj.routing_cost{obj.routing_policy_rebalance}, ...
                        obj.link_uid,...
                        obj.map,...
                        obj.w_wait,...
                        current_time,...
                        obj.routing_policy_rebalance);
        end
        
        
        function [vehicle_list_out,obj] = trip_matching_rebalance_idle(...
                obj,...
                vehicle_list, ...
                customer_list, ...
                current_time, ...
                demand_distribution)
            
                vehicle_list_out ...
                    = trip_matching_rebalance_idle(...
                        vehicle_list,...
                        customer_list,...
                        obj.routing_cost{obj.routing_policy_rebalance_idle},...
                        obj.link_uid,...
                        obj.map,...
                        obj.w_wait,...
                        current_time,...
                        obj.cluster_index,...
                        obj.cluster_centroid_link,...
                        demand_distribution,...
                        obj.balance_weight,...
                        obj.time_step,...
                        obj.cluster_center_trip,...
                        obj.discount_factor,...
                        obj.routing_policy_rebalance_idle);
        end
        
        
        function obj = compute_cluster_center_trip(obj)
            obj.cluster_center_trip = cell(length(obj.cluster_centroid_link),length(obj.cluster_centroid_link));
            for origin_id = 1:length(obj.cluster_centroid_link)
                origin = obj.cluster_centroid_link(origin_id);
                virtual_vehicle = Vehicle(origin_id,4,origin,[],[]);
                for destination_id = 1:length(obj.cluster_centroid_link)
                    destination = obj.cluster_centroid_link(destination_id);
                    if destination~=origin
                        virtual_customer = Customer(destination_id,destination,destination,inf,inf,1);
                        obj.cluster_center_trip{origin_id,destination_id}=Trip(virtual_vehicle,{virtual_customer},0,{tt_mat,fc_mat},obj.link_uid,obj.w_wait,routing_policy);
                        obj.cluster_center_trip{origin_id,destination_id}=obj.cluster_center_trip{origin_id,destination_id}.reconstruct_route(tt_mat,obj.link_uid,obj.map);
                    end
                end
            end
        end
        
        
        function [customer_list_out,vehicle_list_out,obj] = fleet_plan(...
                obj,...
                vehicle_list, ...
                customer_list, ...
                current_time, ...
                demand_distribution)
            [customer_list_out_match,vehicle_list_out_match,obj] = trip_matching(...
                obj,...
                vehicle_list, ...
                customer_list, ...
                current_time, ...
                demand_distribution);
            [customer_list_out,vehicle_list_out,obj] = trip_matching_rebalance(...
                obj,...
                vehicle_list_out_match, ...
                customer_list_out_match, ...
                current_time);
            if obj.idle_balance == 1
                [vehicle_list_out,obj] = trip_matching_rebalance_idle(...
                    obj,...
                    vehicle_list_out, ...
                    customer_list_out, ...
                    current_time, ...
                    demand_distribution);
            end
        end
        
        
        function obj = update_map(obj,map)
            obj.map = map;
        end
    end
end

