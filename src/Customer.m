classdef Customer
    properties
        customer_id
        origin
        destination
        max_wait_time
        max_delay_time
%         current_wait_time
%         current_delay_time
        rebalance
        ignore
		assign
        in_pool_time
        pickup_time
        delivery_time
        virtual
        enter_time
        max_delivery_time
        max_pickup_time
        fastest_travel_time
    end
    methods
        function obj = Customer(customer_id,enter_time,origin,destination,max_wait_time,max_delay_time,virtual,fastest_travel_time)
%         function obj = Customer(customer_id,origin,destination,max_wait_time,max_delay_time,current_wait_time,current_delay_time)
            obj.customer_id = customer_id;   
            obj.enter_time = enter_time;
            obj.origin = origin;
            obj.destination = destination;
            obj.max_wait_time = max_wait_time;
            obj.max_delay_time = max_delay_time;
%             obj.current_wait_time = current_wait_time;
%             obj.current_delay_time = current_delay_time;
            obj.rebalance = 0;
            obj.ignore = 0;
			obj.assign = 0;
            obj.in_pool_time = 0;
            obj.pickup_time = -1;
            obj.delivery_time = -1;
            obj.virtual = virtual;
            obj.max_delivery_time = obj.enter_time+fastest_travel_time+max_delay_time;
            obj.max_pickup_time = obj.enter_time+max_wait_time;
            obj.fastest_travel_time = fastest_travel_time;
        end
        function obj = update_constraint(obj,max_wait_time,max_delay_time)
            % assign to vehicle, update max wait time and delay time
            % accordingly (or not?)
            obj.max_wait_time = max_wait_time;
            obj.max_delay_time = max_delay_time;
            obj.max_delivery_time = obj.enter_time+obj.fastest_travel_time+obj.max_delay_time;
            obj.max_pickup_time = obj.enter_time+obj.max_wait_time;
        end
        function obj = time_update(obj,current_time)
            % not picked up by vehicle, update during wait
            obj.max_wait_time = obj.max_pickup_time - current_time;
            obj.max_delay_time = obj.max_delivery_time - obj.fastest_travel_time - current_time;
        end
        function obj = location_update(obj,location,tt_mat,link_uid)
            % picked up by vehicle and moving
            fastest_travel_time_curr = tt_mat(link_uid==location,link_uid==obj.destination);
            max_delivery_time_curr = current_time+fastest_travel_time_curr;
            obj.max_wait_time = 0;
            obj.max_delay_time = obj.max_delivery_time - max_delivery_time_curr;
            obj.origin = location;
        end
    end
end