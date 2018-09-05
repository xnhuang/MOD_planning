function vehicle_list_out = trip_matching_rebalance_idle(vehicle_list,customer_list,routing_cost,link_uid,map,w_wait,current_time,cluster_index,cluster_centroid,demand_distribution,balance_weight,time_step,routing_policy,rebalance_radius)
if isempty(customer_list)
    vehicle_list_out = vehicle_list;
    return
end
customer_list_onboard = cellfun(@(x) x.onboard,vehicle_list,'uniformoutput',false);
customer_list_onboard = customer_list_onboard(cellfun(@(x) ~isempty(x),customer_list_onboard));
customer_list_onboard = vertcat(customer_list_onboard{:});
customer_list_aug = [customer_list;customer_list_onboard];
space_available = sum(cellfun(@(x) x.capacity,vehicle_list))-numel(customer_list_aug);
space_available(space_available<1) = 2;

vehicle_list_rebalance = vehicle_list(cellfun(@(x) x.rebalance,vehicle_list)==1,:);
vehicle_list = vehicle_list(cellfun(@(x) x.rebalance,vehicle_list)==0,:);
vehicle_list_out = vehicle_list;
vehicle_list_id = cellfun(@(x) x.vehicle_id,vehicle_list_out);
%% assign ignored customers to idling vehicles (rebalancing)
% assigned_trip = cellfun(@(x) x.trip,vehicle_list,'uniformoutput',false);
% trip_not_empty = cellfun(@(x) ~isempty(x),assigned_trip);
trip_not_empty = (cellfun(@(x) isa(x.trip,'Trip'),vehicle_list));
vehicle_in_use = (cellfun(@(x) x.in_use,vehicle_list));
trip_not_empty = trip_not_empty & vehicle_in_use;

% assigned_trip = assigned_trip(trip_not_empty);
assigned_vehicle = vehicle_list(trip_not_empty);
% assigned_customer_id = cellfun(@(x) x.customer_id_list,assigned_trip,'uniformoutput',false);
% assigned_customer_id = vertcat(assigned_customer_id{:});
% total_customer_id_list = cellfun(@(x) x.customer_id,customer_list);
total_vehicle_id = cellfun(@(x) x.vehicle_id,vehicle_list);
assigned_vehicle_id = cellfun(@(x) x.vehicle_id,assigned_vehicle);

idling_vehicle_list = vehicle_list(~ismember(total_vehicle_id,assigned_vehicle_id));
if isempty(idling_vehicle_list)
    vehicle_list_out = [vehicle_list_out;vehicle_list_rebalance];
    return
end

cluster_size = length(unique(cluster_index));
customer_virtual_list = cell(cluster_size,1);
for cluster_id = 1:cluster_size
    cluster_centroid_curr = cluster_centroid(cluster_id);
    customer_virtual_list{cluster_id} = Customer(cluster_id,cluster_centroid_curr,cluster_centroid_curr,inf,inf,1);
end

trip_mat = cell(size(idling_vehicle_list,1),1);
tic
origin_distribution = sum(demand_distribution,1);
for vehicle_id = 1:size(idling_vehicle_list,1)
    vehicle = idling_vehicle_list{vehicle_id};
    vehicle_location = vehicle.location;
    origin_region = cluster_index(link_uid==vehicle_location);
    tt_loc = routing_cost{1}(link_uid==vehicle_location,:);
    feasible_dest = link_uid(tt_loc<time_step*rebalance_radius);
    [~,destination_id] = ismember(feasible_dest,link_uid);
    cluster_dest = cluster_index(destination_id);
    feasible_dest = feasible_dest(cluster_dest~=origin_region);
    cluster_dest = cluster_dest(cluster_dest~=origin_region);
    [~,cluster_dest_id] = ismember(cluster_dest,1:length(cluster_centroid));
    cluster_dest_centroid = cluster_centroid(cluster_dest_id);
    cluster_center_dist = zeros(length(cluster_dest),1);
    
    for cluster_dest_id = 1:length(cluster_dest)
        cluster_center_dist(cluster_dest_id) = routing_cost{1}(link_uid==feasible_dest(cluster_dest_id),link_uid==cluster_dest_centroid(cluster_dest_id));
    end
    cluster_dest_uniq = unique(cluster_dest);
    trip_cost = zeros(length(cluster_dest_uniq),1);
    cluster_dest_link = zeros(length(cluster_dest_uniq),1);
    unnormal_kl_vehicle = 1e-20*ones(length(cluster_dest_uniq),1);
    for cluster_dest_id = 1:length(cluster_dest_uniq)
        cluster_center_dist_cluster = cluster_center_dist(cluster_dest==cluster_dest_uniq(cluster_dest_id));
        feasible_dest_cluster = feasible_dest(cluster_dest==cluster_dest_uniq(cluster_dest_id));
        [~,min_dist_id] = min(cluster_center_dist_cluster);
        cluster_dest_link(cluster_dest_id) = feasible_dest_cluster(min_dist_id);
        trip_cost(cluster_dest_id) = routing_cost{2}(link_uid==vehicle_location,link_uid==cluster_dest_link(cluster_dest_id));
        demand_pdf = origin_distribution(cluster_dest_uniq(cluster_dest_id));
        unnormal_kl_vehicle(cluster_dest_id) = (demand_pdf).*(log(vehicle.capacity)-log(space_available));
    end
    cluster_dest_link = [cluster_dest_link;vehicle_location];
    trip_cost = [trip_cost;0];
    trip_vehicle_id = vehicle_id*ones(length(trip_cost),1);
    origin_list = vehicle_location*ones(length(trip_cost),1);
    demand_pdf = origin_distribution(origin_region);
    unnormal_kl_vehicle = [unnormal_kl_vehicle;(demand_pdf).*(log(vehicle.capacity)-log(space_available))];

    trip_mat{vehicle_id} = {cluster_dest_link origin_list trip_cost unnormal_kl_vehicle trip_vehicle_id};
end

toc
% trip_list = reshape(trip_mat',[],1);
trip_list = vertcat(trip_mat{:});
dest_list = trip_list(:,1);
origin_list = trip_list(:,2);
fc_cost_list = trip_list(:,3);
kl_cost_list = trip_list(:,4);
veh_id_list = trip_list(:,5);
dest_list = vertcat(dest_list{:});
origin_list = vertcat(origin_list{:});
fc_cost_list = vertcat(fc_cost_list{:});
kl_cost_list = vertcat(kl_cost_list{:});
veh_id_list = vertcat(veh_id_list{:});

trip_vehicle_indicator_mat = zeros(size(idling_vehicle_list,1),length(veh_id_list));

for trip_id = 1:length(veh_id_list)
    trip_vehicle_indicator_mat(veh_id_list(trip_id),trip_id) = 1;
end
% toc
% tic
b_vehicle = ones(size(idling_vehicle_list,1),1);

variableType = 'B';
variableTypeCell = cell(1,length(fc_cost_list));
[variableTypeCell{:}] = deal(variableType);
variableTypeChar = char(variableTypeCell{:});

clear model;
model.A = sparse([...
    trip_vehicle_indicator_mat]);
model.rhs = [b_vehicle];
model.obj = [
    (1-balance_weight)*fc_cost_list - balance_weight*10000*kl_cost_list;
    ];
model.modelsense = 'min';
model.vtype = variableTypeChar;
params.outputflag = 0;
model.sense = [
    repmat('=', length(b_vehicle), 1)];

result = gurobi(model, params);
% toc
trip_assignment = result.x(1:size(fc_cost_list,1));
destination_list_out = dest_list(trip_assignment==1);
origin_list_out = origin_list(trip_assignment==1);
vehicle_id_list_out = veh_id_list(trip_assignment==1);
fc_trip_out = fc_cost_list(trip_assignment==1);
for trip_id = 1:size(vehicle_id_list_out,1)
    if fc_trip_out(trip_id)>0
        trip = Trip(idling_vehicle_list{vehicle_id_list_out(trip_id)},{Customer(trip_id,destination_list_out(trip_id),destination_list_out(trip_id),inf,inf,1)},current_time,routing_cost,link_uid,w_wait,routing_policy);
        trip = trip.reconstruct_route(routing_cost,link_uid,map);
        trip_list_out{trip_id} = trip;
        vehicle_list_out{vehicle_list_id==trip.vehicle_id} = vehicle_list_out{vehicle_list_id==trip.vehicle_id}.assign_empty_trip(trip);
    end
end
vehicle_list_out = [vehicle_list_out;vehicle_list_rebalance];
