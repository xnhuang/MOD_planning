classdef DemandManager
    %DEMANDMANAGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        customer_list
        time_grid
        current_time
        time_step
        customer_id_offset
        
    end
    
    methods
        function obj = DemandManager(time_grid,current_time,time_step,customer_list)
            %DEMANDMANAGER Construct an instance of this class
            %   Detailed explanation goes here
            obj.time_grid = time_grid;
            obj.time_step = time_step;
            obj.current_time = current_time;
            obj.customer_list = customer_list;
            obj.customer_id_offset = 0;
        end
        
        function obj = read_new_customer(obj,customer_list_new)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            for customer_id = 1:numel(customer_list_new)
                customer_list_new.enter_time = obj.current_time;
            end
            obj.customer_list = [obj.customer_list;customer_list_new];
        end
        
        function obj = generate_new_customer(obj,trips,location_list,demand_sample_ratio,tt_mat,link_uid)
            time_lb = obj.time_grid(obj.time_grid==obj.current_time)-obj.time_step;
            time_hb = obj.time_grid(obj.time_grid==obj.current_time);

            trip_time_fil = trips(trips.start>=time_lb&trips.start<time_hb,:);
            origin_list = trip_time_fil.origin;
            destination_list = trip_time_fil.destination;
            [~,origin_list_id] = ismember(origin_list,location_list.location);
            [~,destination_list_id] = ismember(destination_list,location_list.location);
            origin_link_list = 2*location_list.link(origin_list_id)+location_list.dir(origin_list_id);
            destination_link_list = 2*location_list.link(destination_list_id)+location_list.dir(destination_list_id);

            customer_list_new = cell(round(size(origin_list,1)*demand_sample_ratio),1);
            customer_id_list = 1:length(origin_list);
            customer_id_list_sample = datasample(customer_id_list,round(size(origin_list,1)*demand_sample_ratio),'Replace',false);
            parfor customer_id = 1:round(size(origin_list,1)*demand_sample_ratio)
                origin = origin_link_list(customer_id_list_sample(customer_id));
                destination = destination_link_list(customer_id_list_sample(customer_id));
                fastest_tt = tt_mat(link_uid==origin,link_uid==destination);
                customer_list_new{customer_id} = Customer(customer_id+obj.customer_id_offset,obj.current_time,origin,destination,max_wait_time,max_delay_time,fastest_tt);
            end
            obj.customer_id_offset = max(cellfun(@(x) x.customer_id,customer_list_new));
            obj.customer_list = [obj.customer_list;customer_list_new];
        end
        
        function obj = update_customer_state(obj)
            obj.current_time = obj.current_time+obj.time_step;
            for customer_id = 1:numel(obj.customer_list)
                obj.customer_list{customer_id} = obj.customer_list{customer_id}.in_pool_time+obj.time_step;
                obj.customer_list{customer_id} = obj.customer_list{customer_id}.in_pool_time+obj.time_step;
            end
        end
    end
end

