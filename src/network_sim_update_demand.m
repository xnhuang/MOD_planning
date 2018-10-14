function [vehicle_list_out,customer_list_out,customer_delivered] = network_sim_update_demand(vehicle_list, customer_list,time_step)
%NETWORK_SIM_UPDATE_DEMAND Summary of this function goes here
%   Detailed explanation goes here
    vehicle_active_list = vehicle_list(cellfun(@(x) (isa(x.trip,'Trip')),vehicle_list));
    vehicle_inactive_list = vehicle_list(cellfun(@(x) (~isa(x.trip,'Trip')),vehicle_list));
    customer_list_out = customer_list;
    customer_delivered = {};
    
    for vehicle_id = 1:size(vehicle_active_list,1)
        [vehicle_active_list{vehicle_id},customer_list_out,customer_delivered_veh]=vehicle_active_list{vehicle_id}.update_state_customer(time_step,customer_list_out);
        customer_delivered = [customer_delivered;customer_delivered_veh];
    end
    parfor customer_id = 1:numel(customer_list_out)
        customer_list_out{customer_id}.in_pool_time = customer_list_out{customer_id}.in_pool_time+time_step;
        customer_list_out{customer_id}.max_wait_time = customer_list_out{customer_id}.max_wait_time-time_step;
%         customer_list_out{customer_id}.max_delay_time = customer_list_out{customer_id}.max_delay_time-time_step;
        customer_list_out{customer_id}.max_wait_time(customer_list_out{customer_id}.max_wait_time<=0)=0;
%         customer_list_out{customer_id}.max_delay_time(customer_list_out{customer_id}.max_delay_time<=0)=0;
    end
    
    vehicle_list_out = [vehicle_active_list;vehicle_inactive_list];
end

