function [can_combine,wait_time] = pairwise_demand_vehicle_edge(od,vehicle,tt_matrix,link_uid)
tt_vo = tt_matrix(link_uid==vehicle.location,link_uid==od.origin);

wait_time = tt_vo;
can_combine_veh = wait_time<=od.max_wait_time & tt_vo<=od.max_delay_time;
can_combine_onboard = 1;
if ~isempty(vehicle.onboard)
    can_combine_onboard = zeros(size(size(vehicle.onboard,1),1));
    for onboard_customer_id = 1:size(vehicle.onboard,1)
        can_combine_onboard(onboard_customer_id) = pairwise_demand_edge(vehicle.onboard{onboard_customer_id},od,tt_matrix,link_uid);
    end
end
can_combine = all(can_combine_onboard) & can_combine_veh;