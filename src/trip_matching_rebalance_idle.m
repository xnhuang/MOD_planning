function vehicle_list_out = trip_matching_rebalance_idle(vehicle_list,customer_list,tt_mat,fc_mat,link_uid,map,w_wait,current_time,cluster_index,cluster_centroid,demand_distribution,balance_weight,time_step,cluster_center_trip,discount_factor,routing_policy)
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
% customer_ignored_list = customer_list(~ismember(total_customer_id_list,assigned_customer_id));
customer_wait_time_mat = -1*ones(size(customer_virtual_list,1),size(idling_vehicle_list,1));
trip_vehicle_mat = zeros(size(customer_virtual_list,1),size(idling_vehicle_list,1));
trip_customer_mat = zeros(size(customer_virtual_list,1),size(idling_vehicle_list,1));
% trip_mat = cell(size(customer_virtual_list,1),size(idling_vehicle_list,1));
% tic
for customer_id = 1:size(customer_virtual_list,1)
    customer = customer_virtual_list{customer_id};
%     customer_list_out{customer_list_id==customer.customer_id}.rebalance = 1;
    for vehicle_id = 1:size(idling_vehicle_list,1)
        vehicle = idling_vehicle_list{vehicle_id};
        origin_region = cluster_index(link_uid==vehicle.location);
        destination_region = cluster_index(link_uid==customer.destination);
        if origin_region ~= destination_region
%         trip = Trip(vehicle,{customer},current_time,tt_mat,link_uid,w_wait);
%         trip_mat{customer_id,vehicle_id} = trip;
            [~,customer_wait_time] = pairwise_demand_vehicle_edge(customer,vehicle,tt_mat,link_uid);
            customer_wait_time_mat(customer_id,vehicle_id) = customer_wait_time;
    %         customer_wait_time_mat(customer_id,vehicle_id) = trip.wait_time_list(1);
            trip_vehicle_mat(customer_id,vehicle_id) = vehicle.vehicle_id;
            trip_customer_mat(customer_id,vehicle_id) = customer.customer_id;
%         destination_list_time = trip.vehicle_trajectory(time_step:end,1);
%         destination_list_time = destination_list_time - destination_list_time(1)+1;
        end
    end
end
% toc
trip_vehicle = reshape(trip_vehicle_mat',[],1);
trip_customer = reshape(trip_customer_mat',[],1);
customer_wait_time = reshape(customer_wait_time_mat',[],1);
trip_vehicle = trip_vehicle(customer_wait_time>0);
trip_customer = trip_customer(customer_wait_time>0);
customer_wait_time = customer_wait_time(customer_wait_time>0);

% trip_list = reshape(trip_mat',[],1);

idling_vehicle_id = cellfun(@(x) x.vehicle_id, idling_vehicle_list);
ignored_customer_id = cellfun(@(x) x.customer_id, customer_virtual_list);
trip_vehicle_indicator_mat = zeros(size(idling_vehicle_list,1),length(customer_wait_time));
trip_customer_indicator_mat = zeros(size(customer_virtual_list,1),length(customer_wait_time));
unnormal_kl_vehicle = 1e-20*ones(size(customer_wait_time,1),1);
unnormal_kl_empty_vehicle = 1e-20*ones(size(idling_vehicle_list,1),1);
% tic
for vehicle_id = 1:size(idling_vehicle_list,1)
    vehicle = idling_vehicle_list{idling_vehicle_id==idling_vehicle_id(vehicle_id)};
    origin_region = cluster_index(link_uid==vehicle.location);
    demand_pdf_empty_vehicle = demand_distribution(:,origin_region);
%     unnormal_kl_empty_vehicle(vehicle_list_id==trip.vehicle_id) =
%     mean(demand_pdf_empty_vehicle)*log(vehicle.capacity);
    unnormal_kl_empty_vehicle(idling_vehicle_id==vehicle_id) = mean(demand_pdf_empty_vehicle)*(log(vehicle.capacity)-log(space_available));
end
% for trip_id = 1:size(customer_wait_time,1)
%     trip_vehicle_indicator_mat(idling_vehicle_id == trip_vehicle(trip_id),trip_id) = 1;
%     trip_customer_indicator_mat(ignored_customer_id == trip_customer(trip_id),trip_id) = 1;
% end
% toc
% tic
for trip_id = 1:size(customer_wait_time,1)
    trip_vehicle_indicator_mat(idling_vehicle_id == trip_vehicle(trip_id),trip_id) = 1;
    trip_customer_indicator_mat(ignored_customer_id == trip_customer(trip_id),trip_id) = 1;
    
%     trip = trip_list{trip_id};
    vehicle = idling_vehicle_list{idling_vehicle_id==trip_vehicle(trip_id)};
    customer = customer_virtual_list{ignored_customer_id==trip_customer(trip_id)};
    origin_region = cluster_index(link_uid==vehicle.location);
    destination_region = cluster_index(link_uid==customer.destination);
    trip = cluster_center_trip{origin_region,destination_region};
%     demand_pdf_empty_vehicle = demand_distribution(:,origin_region);
%     unnormal_kl_empty_vehicle(vehicle_list_id==trip.vehicle_id) =
%     mean(demand_pdf_empty_vehicle)*log(vehicle.capacity);
%     unnormal_kl_empty_vehicle(idling_vehicle_id==trip.vehicle_id) = mean(demand_pdf_empty_vehicle)*(log(vehicle.capacity)-log(space_available));
%     destination_list = trip.customer_location_sequence(2:end);
%     destination_list_time = trip.customer_location_time(2:end);
%     destination_list = destination_list(destination_list_time>0);
    if time_step>=length(trip.vehicle_trajectory)
        destination_list = trip.vehicle_trajectory(end,2);
        [~,destination_id] = ismember(destination_list,link_uid);
        destination_region = cluster_index(destination_id);
        demand_pdf = demand_distribution(:,destination_region);
    %     unnormal_kl_vehicle(trip_id) = mean(demand_pdf)*log(available_space_vehicle);
        unnormal_kl_vehicle(trip_id) = mean(demand_pdf)*(log(vehicle.capacity)-log(space_available));
        if isnan(unnormal_kl_vehicle(trip_id))
            fprintf('error')
        end
    else
        destination_list = trip.vehicle_trajectory(time_step:end,2);
        destination_list_time = trip.vehicle_trajectory(time_step:end,1);
        destination_list_time = destination_list_time - destination_list_time(1)+1;
        destination_list_time_step = floor(destination_list_time/time_step);
        [~,destination_id] = ismember(destination_list,link_uid);
        destination_region = cluster_index(destination_id);
        demand_pdf = demand_distribution(:,destination_region);
        demand_pdf = mean(demand_pdf,1)';
        discount_factor_vec = discount_factor.^(destination_list_time_step);
        unnormal_kl_vehicle(trip_id) = sum(demand_pdf.*(log(vehicle.capacity)-log(space_available)).*discount_factor_vec)/sum(discount_factor_vec);
        if isnan(unnormal_kl_vehicle(trip_id))
            fprintf('error')
        end
    end
end
% toc
% tic
b_vehicle = ones(size(idling_vehicle_list,1),1);
b_customer = ones(size(customer_virtual_list,1),1);

variableType = 'B';
variableTypeCell = cell(1,length(customer_wait_time)+size(idling_vehicle_list,1));
[variableTypeCell{:}] = deal(variableType);
variableTypeChar = char(variableTypeCell{:});

clear model;
model.A = sparse([...
    trip_vehicle_indicator_mat eye(size(idling_vehicle_list,1));
    trip_customer_indicator_mat zeros(size(customer_virtual_list,1),size(idling_vehicle_list,1))]);
model.rhs = [b_vehicle;b_customer];
model.obj = [
    (1-balance_weight)*customer_wait_time - balance_weight*10000*unnormal_kl_vehicle;...
    -balance_weight*10000*unnormal_kl_empty_vehicle
    ];
model.modelsense = 'min';
model.vtype = variableTypeChar;
params.outputflag = 0;
model.sense = [
    repmat('=', length(b_vehicle), 1);
    repmat('<', length(b_customer), 1)];

result = gurobi(model, params);
% toc
trip_assignment = result.x(1:size(customer_wait_time,1));
vehicle_assignment = trip_vehicle(trip_assignment>0);
customer_assignment = trip_customer(trip_assignment>0);
trip_list_out = cell(sum(trip_assignment),1);
for trip_id = 1:size(vehicle_assignment,1)
    vehicle_id = vehicle_assignment(trip_id);
    customer_id = customer_assignment(trip_id);
    vehicle = vehicle_list{idling_vehicle_id==vehicle_id};
    customer = customer_virtual_list(ignored_customer_id==customer_id);
    trip = Trip(vehicle,customer,current_time,{tt_mat,fc_mat},link_uid,w_wait,routing_policy);
    trip = trip.reconstruct_route({tt_mat,fc_mat},link_uid,map);
    trip_list_out{trip_id} = trip;
    vehicle_list_out{vehicle_list_id==vehicle_id} = vehicle_list_out{vehicle_list_id==vehicle_id}.assign_trip(trip,customer_virtual_list);
end
% assigned_trip_out = [assigned_trip;trip_list_out];
vehicle_list_out = [vehicle_list_out;vehicle_list_rebalance];
