function [vehicle_list_out,map] = network_sim(vehicle_list, map, sim_length, time_step, current_time)
% calculate vehicle count on each link for each time step and update
% vehicle location
vehicle_active_list = vehicle_list(cellfun(@(x) (isa(x.trip,'Trip')),vehicle_list));
vehicle_inactive_list = vehicle_list(cellfun(@(x) (~isa(x.trip,'Trip')),vehicle_list));
% map_out = map;
fprintf('start_simulation\n')
tic
for vehicle_id = 1:size(vehicle_active_list,1)
    if vehicle_active_list{vehicle_id}.departed == 0
        [vehicle_active_list{vehicle_id},map] = vehicle_active_list{vehicle_id}.depart_vehicle(map,current_time);
    end
end
toc
tic
for time = 1:time_step:sim_length*time_step
    for vehicle_id = 1:size(vehicle_active_list,1)
        [vehicle_active_list{vehicle_id},map]=vehicle_active_list{vehicle_id}.update_state(time_step,map,current_time);
    end
    
    link_list = map.link_list;
    parfor link_id = 1:length(map.link_uid)
        link_list{link_id} = link_list{link_id}.update_link_status_sim();
    end
    map.link_list = link_list;
    toc
end
vehicle_list_out = [vehicle_active_list;vehicle_inactive_list];