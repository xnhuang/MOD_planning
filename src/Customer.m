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
    end
    methods
        function obj = Customer(customer_id,origin,destination,max_wait_time,max_delay_time,virtual)
%         function obj = Customer(customer_id,origin,destination,max_wait_time,max_delay_time,current_wait_time,current_delay_time)
            obj.customer_id = customer_id;   
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
        end
    end
end