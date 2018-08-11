classdef Trip
    properties
        vehicle_id
        customer_id_list
        wait_time_list
        delay_time_list
        customer_location_sequence
        customer_onboard_sequence
        customer_location_time
        link_sequence
        vehicle_trajectory
        departure_time
        total_wait_time
        total_delay_time
        violate_wait_time
        violate_delay_time
        total_fuel
        routing_policy
        % 1: fastest routing
        % 2: eco routing
    end
    methods
        function obj = Trip(vehicle,od_set,departure_time,routing_cost_mat,link_uid,w_wait,routing_policy)
            obj.vehicle_id = vehicle.vehicle_id;
            obj.departure_time = departure_time;
            route_result = tsp_pd_t(vehicle,od_set,routing_cost_mat,link_uid,w_wait,routing_policy);
            obj.wait_time_list = route_result.wait_time;
            obj.delay_time_list = route_result.delay_time;
            obj.total_wait_time = sum(route_result.wait_time);
            obj.total_delay_time = sum(route_result.delay_time);
            obj.total_fuel = sum(route_result.fuel_trajectory);
            obj.violate_wait_time = route_result.wait_time_violation_status;
            obj.violate_delay_time = route_result.delay_time_violation_status;
            obj.customer_location_sequence = route_result.location_trajectory;
            obj.customer_location_time = route_result.accu_travel_time;
            obj.customer_onboard_sequence = route_result.onboard_num;
            obj.customer_id_list = cellfun(@(x) x.customer_id,od_set);
            if length(obj.customer_id_list)>length(unique(obj.customer_id_list))
                fprintf('repeated customer\n')
            end
            obj.routing_policy = routing_policy;
        end
        function obj = reconstruct_route(obj,routing_cost_mat,link_uid,map)
            % recover vehicle trajectory from customer location trajectory
            origin_list = obj.customer_location_sequence(1:end-1);
            destination_list = obj.customer_location_sequence(2:end);
            departure_time_list = obj.customer_location_time(1:end-1)+obj.departure_time;
            link_sequence_cell = cell(length(origin_list),1);
            for section_id = 1:length(origin_list)
                section_origin = origin_list(section_id);
                section_destination = destination_list(section_id);
                section_departure_time = departure_time_list(section_id);
                route = recover_route(section_origin,section_destination,routing_cost_mat,map,link_uid,obj.routing_policy);
                route(:,2) = route(:,2)+section_departure_time;
                link_sequence_cell{section_id} = route;
            end
            link_sequence_cell = vertcat(link_sequence_cell{:});
            time_step = link_sequence_cell(:,2);
            [time_uniq,uniq_id] = unique(time_step);
            link_sequence_uniq_time = link_sequence_cell(uniq_id,:);
            time_grid = (obj.departure_time:1:max(time_uniq))';
            if length(time_uniq)==1
                obj.link_sequence = [time_uniq link_sequence_uniq_time(:,1)];
                obj.vehicle_trajectory = [time_grid link_sequence_uniq_time(:,1) obj.customer_onboard_sequence(end)];
            else
                link_trajectory = interp1(time_uniq,link_sequence_uniq_time(:,1),time_grid,'nearest');
                [uniq_loc_time,uniq_loc_time_id] = unique(obj.customer_location_time);
                onboard_trajectory = interp1(uniq_loc_time,obj.customer_onboard_sequence(uniq_loc_time_id),time_grid-time_grid(1),'previous');
                onboard_trajectory(end) = 0;
                obj.link_sequence = [time_uniq link_sequence_uniq_time(:,1)];
                obj.vehicle_trajectory = [time_grid link_trajectory onboard_trajectory];
            end
            if length(obj.customer_id_list)>length(unique(obj.customer_id_list))
                fprintf('repeated customer\n')
            end
        end
    end
end

function route = recover_route(origin,destination,routing_cost_mat,map,link_uid,routing_policy)
% return [link_sequence|cumsum_time|cumsum_fuel]
    cost_mat = routing_cost_mat{routing_policy};
    cost2go_state = cost_mat(link_uid==origin,link_uid==destination);
    current_state = origin;
    link_sequence = current_state;
    cost_sequence = 0;
    time_sequence = 0;
    fuel_sequence = 0;
    while cost2go_state~=0
        feasible_state = map.get_adj_e_next(current_state);
        feasible_state = feasible_state(ismember(feasible_state,link_uid));
        cost2go_feasible_state = zeros(length(feasible_state),2);
        tt_feasible_state = zeros(length(feasible_state),2);
        fc_feasible_state = zeros(length(feasible_state),2);
        for feasible_id = 1:length(feasible_state)
            next_link = feasible_state(feasible_id);
            cost2go_feasible_state(feasible_id,1) = cost_mat(link_uid==current_state,link_uid==next_link);
            cost2go_feasible_state(feasible_id,2) = cost_mat(link_uid==next_link,link_uid==destination);
            tt_feasible_state(feasible_id,1) = routing_cost_mat{1}(link_uid==current_state,link_uid==next_link);
            tt_feasible_state(feasible_id,2) = routing_cost_mat{1}(link_uid==next_link,link_uid==destination);
            fc_feasible_state(feasible_id,1) = routing_cost_mat{2}(link_uid==current_state,link_uid==next_link);
            fc_feasible_state(feasible_id,2) = routing_cost_mat{2}(link_uid==next_link,link_uid==destination);
        end
        [~,min_cost_id] = min(sum(cost2go_feasible_state,2));
        next_state = feasible_state(min_cost_id);
        cost2go_state = cost2go_feasible_state(min_cost_id,2);
        travel_cost = cost2go_feasible_state(min_cost_id,1);
        time_cost = tt_feasible_state(min_cost_id,1);
        fuel_cost = fc_feasible_state(min_cost_id,1);
        current_state = next_state;
        link_sequence = [link_sequence;current_state];
        cost_sequence = [cost_sequence;travel_cost];
        time_sequence = [time_sequence;time_cost];
        fuel_sequence = [fuel_sequence;fuel_cost];
    end
    time_sequence_accu = cumsum(time_sequence);
    route = [link_sequence,time_sequence_accu,fuel_sequence];
end