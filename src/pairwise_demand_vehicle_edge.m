function [can_combine,wait_time] = pairwise_demand_vehicle_edge(od,vehicle,tt_matrix,link_uid,current_time)
tt_vo = tt_matrix(link_uid==vehicle.location,link_uid==od.origin);
time_offset = od.enter_time-current_time;
time_offset(time_offset<=0)=0;
wait_time = tt_vo-time_offset;
can_combine_veh = wait_time<=od.max_wait_time & tt_vo<=od.max_delay_time;
can_combine_onboard = 1;
if ~isempty(vehicle.onboard)
    can_combine_onboard = zeros(size(size(vehicle.onboard,1),1));
    for onboard_customer_id = 1:size(vehicle.onboard,1)
        can_combine_onboard(onboard_customer_id) = pairwise_demand_edge(vehicle.onboard{onboard_customer_id},od,tt_matrix,link_uid,current_time);
    end
end
can_combine = all(can_combine_onboard) & can_combine_veh;