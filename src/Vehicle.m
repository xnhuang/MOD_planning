classdef Vehicle
    properties
        vehicle_id
        capacity
        location
        onboard
        trip
        on_link_time
        link_travel_time
        current_location_list
        pickup_location
        dropoff_location
        rebalance
        departed
        trajectory
        in_use
        distance_traveled
    end
    methods
        function obj = Vehicle(vehicle_id,capacity,location,onboard,trip)
            obj.vehicle_id = vehicle_id;
            obj.capacity = capacity;
            obj.location = location;
            obj.onboard = onboard;
            obj.trip = trip;
            obj.rebalance = 0;
            obj.on_link_time = 0;
            obj.departed = 0;
            obj.in_use = 0;
            obj.trajectory = [location];
            obj.distance_traveled = 0;
        end
        function [obj,map] = depart_vehicle(obj,map)
            link_location = map.link_list{map.link_uid==obj.location};
            obj.link_travel_time = link_location.link_length/max(link_location.current_speed,1);
            if isempty(obj.distance_traveled)
                obj.distance_traveled = 0;
            end
            obj.distance_traveled = obj.distance_traveled+link_location.link_length/2;
            obj.departed = 1;
            if obj.on_link_time == 0
                map.link_list{map.link_uid==obj.location}.current_vehicle_count = map.link_list{map.link_uid==obj.location}.current_vehicle_count+1;
                %                 fprintf('send vehicle on link')
                %                 map.link_list{map.link_uid==obj.location}.current_vehicle_count
            end
        end
        function obj = park_vehicle(obj)
            obj.on_link_time = 0;
            obj.trip = [];
            obj.onboard = {};
            obj.rebalance = 0;
            obj.pickup_location = [];
            obj.dropoff_location = [];
            obj.departed = 0;
            obj.in_use = 0;
        end
        function obj = assign_trip(obj,trip,customer_list)
            %             if ~isempty(obj.trip)
            %                 fprintf('reassign trip\n')
            %             end
            obj.trip = trip;
            obj.current_location_list = 1;
            customer_id_list = trip.customer_id_list;
            customer_id_total_list = cellfun(@(x) x.customer_id,customer_list);
            [~,customer_id_list_index] = ismember(customer_id_list,customer_id_total_list);
            customer_list_vehicle = customer_list(customer_id_list_index(customer_id_list_index>0));
%             customer_list_vehicle = [obj.onboard;customer_list_vehicle];
            obj.trip.customer_id_list = cellfun(@(x) x.customer_id,customer_list_vehicle);
            if ~isequal(sort(customer_id_list),sort(obj.trip.customer_id_list))
                fprintf('error')
            end
            obj.pickup_location = cellfun(@(x) x.origin,customer_list_vehicle);
            obj.dropoff_location = cellfun(@(x) x.destination,customer_list_vehicle);
        end
        function [obj,customer_list,map,customer_delivered] = update_state(obj,time_step,customer_list,map)
            customer_delivered = {};
            if ~isempty(obj.trip)
                obj.on_link_time = obj.on_link_time+time_step;
                if obj.current_location_list<size(obj.trip.link_sequence,1)
                    if obj.on_link_time>=obj.link_travel_time
                        %                         fprintf('move to new link')
                        %                         map.link_list{map.link_uid==obj.location}.current_vehicle_count
                        map.link_list{map.link_uid==obj.location}.current_vehicle_count = map.link_list{map.link_uid==obj.location}.current_vehicle_count-1;
                        if map.link_list{map.link_uid==obj.location}.current_vehicle_count<0
                            fprintf('negative count')
                        end
                        
                        partial_time_step = 0;
                        while obj.on_link_time>=obj.link_travel_time && obj.current_location_list<size(obj.trip.link_sequence,1)
                            if ismember(obj.location,obj.trip.customer_location_sequence)
                                % arrive location for pickup/dropoff
                                if ismember(obj.location,obj.pickup_location)
                                    customer_id_total_list = cellfun(@(x) x.customer_id,customer_list);
                                    customer_id = obj.trip.customer_id_list(obj.pickup_location==obj.location);
                                    [~,customer_id_pickup] = ismember(customer_id,customer_id_total_list);
                                    customer_id_pickup = customer_id_pickup(customer_id_pickup>0);
                                    if ~isempty(customer_id_pickup)
                                        customer_list_pickup = customer_list(customer_id_pickup);
                                        for customer_pick_ip = 1:size(customer_list_pickup,1)
                                            %                                             customer_list_pickup{customer_pick_ip}.max_delay_time = customer_list_pickup{customer_pick_ip}.max_delay_time-customer_list_pickup{customer_pick_ip}.max_wait_time;
                                            customer_list_pickup{customer_pick_ip}.max_wait_time = 0;
                                            customer_list_pickup{customer_pick_ip}.pickup_time = customer_list_pickup{customer_pick_ip}.in_pool_time+partial_time_step;
                                        end
                                        obj.onboard=[obj.onboard;customer_list_pickup];
                                        customer_id_keep = setdiff(customer_id_total_list,customer_id);
                                        [~,customer_id_keep_index] = ismember(customer_id_keep,customer_id_total_list);
                                        customer_list = customer_list(customer_id_keep_index);
                                    end
                                else
                                    if ~isempty(obj.onboard)
                                        customer_id_onboard = cellfun(@(x) x.customer_id,obj.onboard);
                                        customer_dropoff_loc_list = cellfun(@(x) x.destination,obj.onboard);
                                        customer_id_drop = customer_id_onboard(ismember(customer_dropoff_loc_list,obj.location));
                                        customer_id_keep = setdiff(customer_id_onboard,customer_id_drop);
                                        [~,customer_id_keep_index] = ismember(customer_id_onboard,customer_id_keep);
                                        [~,customer_id_drop_index] = ismember(customer_id_onboard,customer_id_drop);
                                        customer_delivered = obj.onboard(customer_id_drop_index(customer_id_drop_index>0));
                                        obj.onboard = obj.onboard(customer_id_keep_index(customer_id_keep_index>0));
                                        for deliver_index = 1:numel(customer_delivered)
                                            customer_delivered{deliver_index}.delivery_time = customer_delivered{deliver_index}.in_pool_time+partial_time_step;
                                        end
                                    end
                                end
                            end
                            link_location = map.link_list{map.link_uid==obj.location};
                            obj.distance_traveled = obj.distance_traveled+link_location.link_length/2;
                            obj.current_location_list = obj.current_location_list+1;
                            obj.on_link_time = obj.on_link_time-obj.link_travel_time;
                            partial_time_step = partial_time_step+obj.link_travel_time;
                            
                            obj.location = obj.trip.link_sequence(obj.current_location_list,2);
                            link_location = map.link_list{map.link_uid==obj.location};
                            obj.link_travel_time = link_location.link_length/max(link_location.current_speed,1);
                            obj.distance_traveled = obj.distance_traveled+link_location.link_length/2;
                            obj.trajectory = [obj.trajectory;obj.location];
                        end
                        for customer_onboard_id = 1:length(obj.onboard)
                            obj.onboard{customer_onboard_id}.origin = obj.location;
                            obj.onboard{customer_onboard_id}.max_delay_time = obj.onboard{customer_onboard_id}.max_delay_time-time_step;
                            obj.onboard{customer_onboard_id}.max_delay_time(obj.onboard{customer_onboard_id}.max_delay_time<=0) = 0;
                            obj.onboard{customer_onboard_id}.in_pool_time = obj.onboard{customer_onboard_id}.in_pool_time+time_step;
                        end
                        map.link_list{map.link_uid==obj.location}.current_vehicle_count = map.link_list{map.link_uid==obj.location}.current_vehicle_count+1;
                        
                        if obj.current_location_list==size(obj.trip.link_sequence,1)
                            %customer_id = cellfun(@(x) x.customer_id,obj.onboard);
                            % drop off customer if onboard
                            if ~isempty(obj.onboard)
                                customer_id_onboard = cellfun(@(x) x.customer_id,obj.onboard);
                                customer_dropoff_loc_list = cellfun(@(x) x.destination,obj.onboard);
                                customer_id_drop = customer_id_onboard(ismember(customer_dropoff_loc_list,obj.location));
                                customer_id_keep = setdiff(customer_id_onboard,customer_id_drop);
                                [~,customer_id_keep_index] = ismember(customer_id_onboard,customer_id_keep);
                                [~,customer_id_drop_index] = ismember(customer_id_onboard,customer_id_drop);
                                customer_delivered = obj.onboard(customer_id_drop_index(customer_id_drop_index>0));
                                obj.onboard = obj.onboard(customer_id_keep_index(customer_id_keep_index>0));
                                for deliver_index = 1:numel(customer_delivered)
                                    customer_delivered{deliver_index}.delivery_time = customer_delivered{deliver_index}.in_pool_time;
                                end
                            end
                            obj = obj.park_vehicle();
                            map.link_list{map.link_uid==obj.location}.current_vehicle_count = map.link_list{map.link_uid==obj.location}.current_vehicle_count-1;
                        end
                    end
                else
                    % arrive at destination
                    if ~isempty(obj.onboard)
                        customer_id_onboard = cellfun(@(x) x.customer_id,obj.onboard);
                        customer_dropoff_loc_list = cellfun(@(x) x.destination,obj.onboard);
                        customer_id_drop = customer_id_onboard(ismember(customer_dropoff_loc_list,obj.location));
                        customer_id_keep = setdiff(customer_id_onboard,customer_id_drop);
                        [~,customer_id_keep_index] = ismember(customer_id_onboard,customer_id_keep);
                        [~,customer_id_drop_index] = ismember(customer_id_onboard,customer_id_drop);
                        customer_delivered = obj.onboard(customer_id_drop_index(customer_id_drop_index>0));
                        obj.onboard = obj.onboard(customer_id_keep_index(customer_id_keep_index>0));
                        for deliver_index = 1:numel(customer_delivered)
                            customer_delivered{deliver_index}.delivery_time = customer_delivered{deliver_index}.in_pool_time;
                        end
                    end
                    obj = obj.park_vehicle();
                    %                     map.link_list{map.link_uid==obj.location}.current_vehicle_count = map.link_list{map.link_uid==obj.location}.current_vehicle_count-1;
                end
            end
        end
    end
end