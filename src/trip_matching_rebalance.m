function [customer_list_out,vehicle_list_out] = trip_matching_rebalance(vehicle_list,customer_list,routing_cost,link_uid,map,w_wait,current_time,routing_policy)
if isempty(customer_list)
    customer_list_out = customer_list;
    vehicle_list_out = vehicle_list;
    return
end
tt_mat = routing_cost{1};
fc_mat = routing_cost{2};
vehicle_list_rebalance = vehicle_list(cellfun(@(x) x.rebalance,vehicle_list)==1,:);
vehicle_list = vehicle_list(cellfun(@(x) x.rebalance,vehicle_list)==0,:);
customer_list_rebalance = customer_list(cellfun(@(x) x.rebalance,customer_list)==1,:);
customer_list = customer_list(cellfun(@(x) x.rebalance,customer_list)==0,:);

customer_list_out = customer_list;
vehicle_list_out = vehicle_list;
vehicle_list_id = cellfun(@(x) x.vehicle_id,vehicle_list_out);
customer_list_id = cellfun(@(x) x.customer_id,customer_list_out);
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
customer_ignored_list = customer_list(cellfun(@(x) x.ignore,customer_list)==1,:);
% customer_ignored_list = customer_list(~ismember(total_customer_id_list,assigned_customer_id));
customer_wait_time_mat = zeros(size(customer_ignored_list,1),size(idling_vehicle_list,1));
customer_inpool_time_mat = zeros(size(customer_ignored_list,1),size(idling_vehicle_list,1));

trip_vehicle_mat = zeros(size(customer_ignored_list,1),size(idling_vehicle_list,1));
trip_customer_time_mat = zeros(size(customer_ignored_list,1),size(idling_vehicle_list,1));
for customer_id = 1:size(customer_ignored_list,1)
    customer = customer_ignored_list{customer_id};
%     customer_list_out{customer_list_id==customer.customer_id}.rebalance = 1;
    for vehicle_id = 1:size(idling_vehicle_list,1)
        vehicle = idling_vehicle_list{vehicle_id};
        [~,customer_wait_time] = pairwise_demand_vehicle_edge(customer,vehicle,tt_mat,link_uid);
        customer_wait_time_mat(customer_id,vehicle_id) = customer_wait_time;
        customer_inpool_time_mat(customer_id,vehicle_id) = customer.in_pool_time;
        trip_vehicle_mat(customer_id,vehicle_id) = vehicle.vehicle_id;
        trip_customer_time_mat(customer_id,vehicle_id) = customer.customer_id;
    end
end
trip_vehicle = reshape(trip_vehicle_mat',[],1);
trip_customer = reshape(trip_customer_time_mat',[],1);
idling_vehicle_id = cellfun(@(x) x.vehicle_id, idling_vehicle_list);
ignored_customer_id = cellfun(@(x) x.customer_id, customer_ignored_list);
customer_wait_time = reshape(customer_wait_time_mat',[],1);
customer_inpool_time = reshape(customer_inpool_time_mat',[],1);
trip_vehicle_indicator_mat = zeros(size(idling_vehicle_list,1),length(customer_wait_time));
trip_customer_indicator_mat = zeros(size(customer_ignored_list,1),length(customer_wait_time));
for trip_id = 1:length(customer_wait_time)
    trip_vehicle_indicator_mat(idling_vehicle_id == trip_vehicle(trip_id),trip_id) = 1;
    trip_customer_indicator_mat(ignored_customer_id == trip_customer(trip_id),trip_id) = 1;
end
b_vehicle = ones(size(idling_vehicle_list,1),1);
b_customer = ones(size(customer_ignored_list,1),1);

variableType = 'B';
variableTypeCell = cell(1,length(customer_wait_time));
[variableTypeCell{:}] = deal(variableType);
variableTypeChar = char(variableTypeCell{:});

clear model;
model.A = sparse([...
    trip_vehicle_indicator_mat;
    trip_customer_indicator_mat]);
model.rhs = [b_vehicle;b_customer];
model.obj = [customer_wait_time.*((customer_inpool_time+1).^2)];
model.modelsense = 'min';
model.vtype = variableTypeChar;
params.outputflag = 0;

if size(customer_ignored_list,1)>size(idling_vehicle_list,1)
    % vehicle less than customer, all vehicle assigned
    model.sense = [repmat('=', length(b_vehicle), 1);
        repmat('<', length(b_customer), 1)];
else
    % vehicle more than customer, all customer matched
    model.sense = [repmat('<', length(b_vehicle), 1);
        repmat('=', length(b_customer), 1)];
end
result = gurobi(model, params);
customer_assignment = trip_customer(result.x>0);
vehicle_assignment = trip_vehicle(result.x>0);

trip_list_out = cell(size(vehicle_assignment,1),1);
for trip_id = 1:size(vehicle_assignment,1)
    vehicle_id = vehicle_assignment(trip_id);
    customer_id = customer_assignment(trip_id);
    vehicle = idling_vehicle_list{idling_vehicle_id==vehicle_id};
    customer = customer_ignored_list{ignored_customer_id==customer_id};
    trip =  Trip(vehicle,{customer},current_time,{tt_mat,fc_mat},link_uid,w_wait,routing_policy);
    trip = trip.reconstruct_route({tt_mat,fc_mat},link_uid,map);
    trip_list_out{trip_id} = trip;
    customer_list_out{customer_list_id==customer.customer_id}.rebalance = 1;
    customer_list_out{customer_list_id==customer.customer_id}.max_wait_time ...
        = trip.wait_time_list(1);
%     customer_list_out{customer_list_id==customer.customer_id}.max_delay_time ...
%         = customer_list_out{customer_list_id==customer.customer_id}.max_delay_time - trip.wait_time_list(1);
%     
    vehicle_list_out{vehicle_list_id==vehicle_id} = vehicle_list_out{vehicle_list_id==vehicle_id}.assign_trip(trip,customer_list);
    vehicle_list_out{vehicle_list_id==vehicle_id}.rebalance = 1;
    vehicle_list_out{vehicle_list_id==vehicle_id}.in_use = 1;
end
% assigned_trip_out = [assigned_trip;trip_list_out];
vehicle_list_out = [vehicle_list_out;vehicle_list_rebalance];
customer_list_out = [customer_list_out;customer_list_rebalance];