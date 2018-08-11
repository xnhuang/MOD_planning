function [customer_list_out,...
    vehicle_list_out,...
    vehicle_trip_cons_fil,...
    customer_adj_mat_cell,...
    customer_id_list, ...
    customer_vehicle_connection_mat_cell, ...
    vehicle_id_list]...
    = trip_matching_pass_adj_mat(...
    vehicle_list, ...
    customer_list, ...
    routing_cost, ...
    link_uid, ...
    map, ...
    w_wait, ...
    max_clique_size, ...
    current_time, ...
    cluster_index, ...
    demand_distribution, ...
    balance_weight, ...
    time_step, ...
    discount_factor, ...
    customer_graph_last, ...
    customer_graph_id_last, ...
    vehicle_graph_last, ...
    vehicle_graph_id_last ,...
    routing_policy)

tt_mat = routing_cost{1};
fc_mat = routing_cost{2};

customer_list_out = customer_list;
vehicle_list_out = vehicle_list;
vehicle_list_rebalance = vehicle_list(cellfun(@(x) x.rebalance,vehicle_list)==1,:);
vehicle_list = vehicle_list(cellfun(@(x) x.rebalance,vehicle_list)==0,:);
customer_list_rebalance = customer_list(cellfun(@(x) x.rebalance,customer_list)==1,:);
customer_list = customer_list(cellfun(@(x) x.rebalance,customer_list)==0,:);
customer_list_ignore = customer_list(cellfun(@(x) x.ignore,customer_list)==1,:);
customer_list = customer_list(cellfun(@(x) x.ignore,customer_list)==0,:);
if size(customer_list,1)==0
    return
end
% customer_list_length = size(customer_list,1)
customer_list_onboard = cellfun(@(x) x.onboard,vehicle_list,'uniformoutput',false);
customer_list_onboard = customer_list_onboard(cellfun(@(x) ~isempty(x),customer_list_onboard));
customer_list_onboard = vertcat(customer_list_onboard{:});
customer_list_virtual = 0;
if ~isempty(customer_list_onboard)
    customer_list_virtual = cellfun(@(x) x.virtual,customer_list_onboard);
    customer_list_onboard = customer_list_onboard(customer_list_virtual==0);
end

customer_list_aug = [customer_list;customer_list_onboard];
customer_id_aug = cellfun(@(x) x.customer_id,customer_list_aug);
customer_id_list = cellfun(@(x) x.customer_id,customer_list);
% customer_id_list = sort(customer_id_list);
[customer_is_remain,customer_id_remain] = ismember(customer_graph_id_last,customer_id_list);
customer_id_remain = customer_id_remain(customer_id_remain>0);
customer_graph_id_last = customer_graph_id_last(customer_is_remain);
customer_graph_last = customer_graph_last(customer_is_remain);
customer_graph_last = cellfun(@(x) x(customer_is_remain),customer_graph_last,'uniformoutput',false);
% [~,customer_id_remain_loc] = ismember(customer_graph_id_last,customer_id_list);
customer_set_id = 1:length(customer_id_list);
customer_list_new_set_id = setdiff(customer_set_id,customer_id_remain);
customer_list_new = customer_list(customer_list_new_set_id);

vehicle_trip_nonempty = (cellfun(@(x) isa(x.trip,'Trip'),vehicle_list));
vehicle_in_use = (cellfun(@(x) x.in_use,vehicle_list));
vehicle_trip_nonempty = vehicle_trip_nonempty & vehicle_in_use;
vehicle_id_idling = cellfun(@(x) x.vehicle_id,vehicle_list(~vehicle_trip_nonempty));

vehicle_id_list = cellfun(@(x) x.vehicle_id,vehicle_list);
% vehicle_id_list = sort(vehicle_id_list);
[vehicle_is_remain,vehicle_id_remain] = ismember(vehicle_graph_id_last,vehicle_id_list);
vehicle_graph_id_last = vehicle_graph_id_last(vehicle_id_remain>0);
% vehicle_id_remain = vehicle_id_remain(vehicle_id_remain>0);
vehicle_graph_last = vehicle_graph_last(customer_id_remain);
vehicle_graph_last = cellfun(@(x) x(vehicle_is_remain),vehicle_graph_last,'uniformoutput',false);

[vehicle_is_idle,vehicle_id_remain_idle] = ismember(vehicle_graph_id_last,vehicle_id_idling);
vehicle_id_remain_idle = vehicle_id_remain_idle(vehicle_id_remain_idle>0);
% vehicle_graph_last_idling = vehicle_graph_last(vehicle_id_remain_idle);
vehicle_graph_last_idling = cellfun(@(x) x(vehicle_is_idle),vehicle_graph_last,'uniformoutput',false);

if ~isempty(customer_list_onboard)
    customer_id_onboard = cellfun(@(x) x.customer_id,customer_list_onboard);
end
vehicle_list_nonidle = vehicle_list(vehicle_trip_nonempty);
vehicle_trip = cellfun(@(x) x.trip,vehicle_list(vehicle_trip_nonempty),'uniformoutput',false);
customer_id_assigned_trip = cellfun(@(x) x.customer_id_list,vehicle_trip,'uniformoutput',false);
customer_id_assigned_trip = vertcat(customer_id_assigned_trip{:});
space_available = sum(cellfun(@(x) x.capacity,vehicle_list))-numel(customer_list_aug);
space_available(space_available<1) = 2;
w_balance = balance_weight;
%% get adjacency matrix for customer graph
fleet_size = size(vehicle_list,1);
customer_adj_mat_cell = cell(size(customer_list,1),1);
customer_vehicle_connection_mat_cell = cell(size(customer_list,1),1);
fprintf('construct pairwise connection\n')

tic
for customer_id_ego = 1:size(customer_list,1)
    customer_ego = customer_list{customer_id_ego};
    customer_adj_mat_row = zeros(1,size(customer_list,1));
    customer_vehicle_connection_mat_row = zeros(1,fleet_size);
    if customer_id_ego<size(customer_list,1)
        if ismember(customer_ego.customer_id,customer_graph_id_last)
            
            customer_adj_mat_row(customer_id_remain) = customer_graph_last{customer_graph_id_last==customer_ego.customer_id};
            for customer_id = 1:numel(customer_list_new)
                customer = customer_list_new{customer_id};
                if pairwise_demand_edge(customer,customer_ego,tt_mat,link_uid)
                    customer_adj_mat_row(customer_id_list==customer.customer_id) = 1;
                end
            end
        else
            for customer_id = 1:size(customer_list,1)
                if customer_id ~= customer_id_ego
                    customer = customer_list{customer_id};
                    if pairwise_demand_edge(customer,customer_ego,tt_mat,link_uid)
                        customer_adj_mat_row(customer_id_list==customer.customer_id) = 1;
                    end
                end
            end
        end
    end
    if ismember(customer_ego.customer_id,customer_graph_id_last)
        
        customer_vehicle_connection_mat_row(vehicle_id_remain_idle) = vehicle_graph_last_idling{customer_graph_id_last==customer_ego.customer_id};
        for vehicle_id = 1:numel(vehicle_list_nonidle)
            vehicle = vehicle_list_nonidle{vehicle_id};
            if pairwise_demand_vehicle_edge(customer_ego,vehicle,tt_mat,link_uid)
                customer_vehicle_connection_mat_row(vehicle_id_list==vehicle.vehicle_id) = 1;
            end
        end
    else
        for vehicle_id = 1:fleet_size
            vehicle = vehicle_list{vehicle_id};
            if pairwise_demand_vehicle_edge(customer_ego,vehicle,tt_mat,link_uid)
                customer_vehicle_connection_mat_row(vehicle_id_list==vehicle.vehicle_id) = 1;
            end
        end
    end
    
    customer_adj_mat_cell{customer_id_ego} = customer_adj_mat_row;
    customer_vehicle_connection_mat_cell{customer_id_ego} = customer_vehicle_connection_mat_row;
    %     customer_id_ego/length(origin_list)
end
customer_adj_mat = vertcat(customer_adj_mat_cell{:});
customer_vehicle_connection_mat = vertcat(customer_vehicle_connection_mat_cell{:});
toc
customer_adj_mat = customer_adj_mat+customer_adj_mat';
%% find cliques in matching
% use graph necessary condition to find potential trips
max_customer_clique = maximalCliques(customer_adj_mat>0);
vehicle_trip_set = cell(fleet_size,size(max_customer_clique,2));
fprintf('find all cliques\n')
tic
parfor mc_id = 1:size(max_customer_clique,2)
    max_clique = max_customer_clique(:,mc_id);
    customer_mc = find(max_clique>0);
    customer_vehicle_mc = customer_vehicle_connection_mat(customer_mc,:);
    %     tic
    for vehicle_id = 1:fleet_size
        vehicle = vehicle_list{vehicle_id};
        vehicle_onboard = vehicle.onboard;
        vehicle_max_clique = customer_mc(customer_vehicle_mc(:,vehicle_id)>0);
        %         vehicle_max_clique = vehicle_max_clique(~ismember(vehicle_max_clique,vehicle_onboard));
        if ~isempty(vehicle_max_clique)
            max_clique_size_trip = max_clique_size-size(vehicle_onboard,1);
            total_vehicle_clique_size = 2^length(vehicle_max_clique);
            clique_bi = de2bi(1:total_vehicle_clique_size-1);
            clique_bi = clique_bi(sum(clique_bi,2)<=max_clique_size_trip,:);
            vehicle_trip_mat = repmat(vehicle_max_clique',size(clique_bi,1),1).*clique_bi;
            vehicle_trip_mat = sort(vehicle_trip_mat,2);
            
            if size(vehicle_trip_mat,2)>max_clique_size_trip
                vehicle_trip_mat = vehicle_trip_mat(:,end-max_clique_size_trip+1:end);
            else
                vehicle_trip_mat_make_up = zeros(size(vehicle_trip_mat,1),max_clique_size_trip-size(vehicle_trip_mat,2));
                vehicle_trip_mat = [vehicle_trip_mat_make_up,vehicle_trip_mat];
            end
            %             vehicle_trip_mat = [vehicle_trip_mat repmat(vehicle_onboard',size(vehicle_trip_mat,1),1)];
            vehicle_trip_set{vehicle_id,mc_id} = vehicle_trip_mat;
            %         else
            %             vehicle_trip_mat_make_up = zeros(1,max_clique_size-size(vehicle_onboard,1));
            %             vehicle_trip_mat = [vehicle_trip_mat_make_up,vehicle_onboard'];
            %             vehicle_trip_set{vehicle_id,mc_id} = vehicle_trip_mat;
        end
        
    end
    %     toc
    %     mc_id
end
toc
vehicle_trip = cell(fleet_size,1);
trip_exist_cell = cell(fleet_size,1);
fprintf('get trip schedule\n')
tic
for vehicle_id = 1:fleet_size
    trip_list = vehicle_trip_set(vehicle_id,:);
    trip_list_empty = cellfun(@(x) size(x,2),trip_list);
    trip_list = trip_list(trip_list_empty>0);
    trip_list = vertcat(trip_list{:});
    
    trip_list = unique(trip_list,'rows');
    trip_list_sum = sum(trip_list,2);
    trip_list = trip_list(trip_list_sum>0,:);
    customer_id = unique(vertcat(trip_list));
    customer_id = customer_id(customer_id>0);
    trip_list_mat = zeros(size(trip_list,1),length(customer_id));
    for trip_id = 1:size(trip_list,1)
        trip_customer = trip_list(trip_id,:);
        trip_customer = trip_customer(trip_customer>0);
        [~,customer_index] = ismember(trip_customer,customer_id);
        trip_list_mat(trip_id,customer_index) = 1;
    end
    vehicle = vehicle_list{vehicle_id};
    trip_set = cell(size(trip_list,1),1);
    trip_list_mark = zeros(size(trip_list,1),1);
    for trip_id = 1:size(trip_list,1)
        if trip_list_mark(trip_id)==0
            od_id = trip_list(trip_id,:);
            od_set = customer_list(od_id(od_id>0));
            available_space = max_clique_size;
            if ~isempty(vehicle.onboard)
                available_space = max_clique_size - size(vehicle.onboard,1);
                if available_space>0 & size(od_set,1)>available_space
                    od_set = datasample(od_set,available_space,'Replace',false);
                end
                od_set = [od_set;vehicle.onboard];
            end
            od_customer_id = cellfun(@(x) x.customer_id,od_set);
            [~,uniq_od_id] = unique(od_customer_id);
            od_set = od_set(uniq_od_id);
            if available_space>0
                
                trip = Trip(vehicle,od_set,current_time,{tt_mat,fc_mat},link_uid,w_wait,routing_policy);
                trip_set{trip_id} = trip;
                trip_list_mark(trip_id) = 1;
                if trip.violate_delay_time || trip.violate_wait_time
                    % find superset of infeasible trips
                    trip_set_check = trip_list_mat-repmat(trip_list_mat(trip_id,:),size(trip_list_mat,1),1);
                    trip_list_mark(all(trip_set_check>=0,2))=1; % mark supset of infeasible trip
                end
            end
        end
        if sum(trip_list_mark) == length(trip_list_mark)
            break;
        end
        %         length(trip_list_mark(trip_list_mark==0))/length(trip_list_mark)
    end
    trip_set_empty = cellfun(@isempty,trip_set);
    trip_set = trip_set(~trip_set_empty);
    trip_exist_cell{vehicle_id} = zeros(size(trip_set,1),1);
    current_trip = vehicle.trip;
    vehicle_trip{vehicle_id} = trip_set;
    if isa(current_trip,'Trip')
        trip_customer_list = current_trip.customer_id_list;
        %         trip_customer_list = trip_customer_list(ismember(trip_customer_list,customer_id_aug));
        %         if ~isempty(trip_customer_list)
        %             append current trip plan
        [~,od_id] = ismember(trip_customer_list,customer_id_aug);
        od_set = customer_list_aug(od_id(od_id>0));
        %         if ~isempty(vehicle.onboard)
        %             od_set = [vehicle.onboard;od_set];
        %         end
        od_customer_id = cellfun(@(x) x.customer_id,od_set);
        [~,uniq_od_id] = unique(od_customer_id);
        od_set = od_set(uniq_od_id);
        if ~isempty(od_set)
            trip = Trip(vehicle,od_set,current_time,{tt_mat,fc_mat},link_uid,w_wait,routing_policy);
            vehicle_trip{vehicle_id} = [{trip};trip_set];
            trip_exist_cell{vehicle_id} = [1;trip_exist_cell{vehicle_id}];
        end
        %         end
    end
end
toc

%% ILP to match vehicles and trips
% process trip set to get customer-trip and vehicle-trip relation
fprintf('solve ILP for trip vehicle assignment\n')
tic
vehicle_trip = vertcat(vehicle_trip{:});
trip_exist = vertcat(trip_exist_cell{:});
trip_exist = trip_exist(cellfun(@(x) ~isempty(x),vehicle_trip));
vehicle_trip = vehicle_trip(cellfun(@(x) ~isempty(x),vehicle_trip));
trip_constraint_status = cellfun(@(x) x.violate_wait_time|x.violate_delay_time,vehicle_trip);
vehicle_trip_cons_fil = vehicle_trip(trip_constraint_status==0 | trip_exist==1);

trip_vehicle_mat = zeros(size(vehicle_list,1),size(vehicle_trip_cons_fil,1));

trip_demand_mat = zeros(size(customer_list_aug,1),size(vehicle_trip_cons_fil,1));
trip_demand_wait_time_mat = zeros(size(customer_list_aug,1),size(vehicle_trip_cons_fil,1));
trip_demand_delay_time_mat = zeros(size(customer_list_aug,1),size(vehicle_trip_cons_fil,1));
customer_list_id = cellfun(@(x) x.customer_id,customer_list_aug);
vehicle_list_id = cellfun(@(x) x.vehicle_id,vehicle_list);
unnormal_kl_vehicle = 1e-20*ones(size(vehicle_trip_cons_fil,1),1);
unnormal_kl_empty_vehicle = 1e-20*ones(fleet_size,1);

for trip_id = 1:size(vehicle_trip_cons_fil,1)
    vehicle_trip_cons_fil{trip_id}=vehicle_trip_cons_fil{trip_id}.reconstruct_route({tt_mat,fc_mat},link_uid,map);
    trip = vehicle_trip_cons_fil{trip_id};
    trip_vehicle_mat(vehicle_list_id==trip.vehicle_id,trip_id) = 1;
    vehicle = vehicle_list{vehicle_list_id==trip.vehicle_id};
    available_space_vehicle = vehicle.capacity - length(trip.customer_id_list);
    origin_region = cluster_index(link_uid==trip.customer_location_sequence(1));
    demand_pdf_empty_vehicle = demand_distribution(:,origin_region);
    %     unnormal_kl_empty_vehicle(vehicle_list_id==trip.vehicle_id) =
    %     mean(demand_pdf_empty_vehicle)*log(vehicle.capacity);
    unnormal_kl_empty_vehicle(vehicle_list_id==trip.vehicle_id) = mean(demand_pdf_empty_vehicle)*(log(vehicle.capacity)-log(space_available));
    %     destination_list = trip.customer_location_sequence(2:end);
    %     destination_list_time = trip.customer_location_time(2:end);
    %     destination_list = destination_list(destination_list_time>0);
    if time_step>=length(trip.vehicle_trajectory)
        destination_list = trip.vehicle_trajectory(end,2);
        [~,destination_id] = ismember(destination_list,link_uid);
        destination_region = cluster_index(destination_id);
        demand_pdf = demand_distribution(destination_region,origin_region);
        %     unnormal_kl_vehicle(trip_id) = mean(demand_pdf)*log(available_space_vehicle);
        unnormal_kl_vehicle(trip_id) = mean(demand_pdf)*(log(vehicle.capacity)-log(space_available));
        if isnan(unnormal_kl_vehicle(trip_id))
            fprintf('error')
        end
        
    else
        destination_list = trip.vehicle_trajectory(time_step:end,2);
        destination_list_occupy = trip.vehicle_trajectory(time_step:end,3);
        destination_list_time = trip.vehicle_trajectory(time_step:end,1);
        destination_list_time = destination_list_time - destination_list_time(1)+1;
        destination_list_time_step = floor(destination_list_time/time_step);
        discount_factor_vec = discount_factor.^(destination_list_time_step);
        [~,destination_id] = ismember(destination_list,link_uid);
        destination_region = cluster_index(destination_id);
        demand_pdf = demand_distribution(destination_region,origin_region);
        %     unnormal_kl_vehicle(trip_id) = mean(demand_pdf)*log(available_space_vehicle);
        available_space_vehicle = vehicle.capacity - destination_list_occupy;
        available_space_vehicle(available_space_vehicle==0) = 0.01;
        unnormal_kl_vehicle(trip_id) = sum(demand_pdf.*(log(available_space_vehicle)-log(space_available)).*discount_factor_vec)/sum(discount_factor_vec);
        if isnan(unnormal_kl_vehicle(trip_id))
            fprintf('error')
        end
    end
    for customer_id = 1:length(trip.customer_id_list)
        customer_id_curr = trip.customer_id_list(customer_id);
        if ismember(customer_id_curr,customer_list_id)
            customer = customer_list_aug{customer_list_id==customer_id_curr};
            %         else
            %             onboard_customer_id = cellfun(@(x) x.customer_id,vehicle.onboard);
            %             customer = vehicle.onboard{onboard_customer_id==customer_id_curr};
            %         end
            trip_demand_mat(customer_list_id==customer.customer_id,trip_id) = 1;
            trip_demand_wait_time_mat(customer_list_id==customer.customer_id,trip_id) = trip.wait_time_list(trip.customer_id_list==customer.customer_id);
            trip_demand_delay_time_mat(customer_list_id==customer.customer_id,trip_id) = trip.delay_time_list(trip.customer_id_list==customer.customer_id);
        end
    end
end
trip_total_wait = cellfun(@(x) x.total_wait_time,vehicle_trip_cons_fil);
trip_total_delay = cellfun(@(x) x.total_delay_time,vehicle_trip_cons_fil);
trip_total_fuel = cellfun(@(x) x.total_fuel,vehicle_trip_cons_fil);

b_vehicle_const = ones(fleet_size,1);
b_demand_const = ones(size(customer_list_aug,1),1);
b_demand_complement = ones(size(customer_list_aug,1),1);
b_vehicle_complement = ones(size(vehicle_list,1),1);

complement_demand_inequ_v = zeros(fleet_size,size(customer_list_aug,1));
complement_demand_inequ_c = zeros(size(customer_list_aug,1));
complement_demand_equ = eye(size(customer_list_aug,1));
complement_vehicle_equ = eye(size(vehicle_list,1));

variableType = 'B';
variableTypeCell = cell(1,size(trip_vehicle_mat,2)+size(trip_demand_mat,1)+size(trip_vehicle_mat,1));
[variableTypeCell{:}] = deal(variableType);
variableTypeChar = char(variableTypeCell{:});
in_pool_time = cellfun(@(x) x.in_pool_time,customer_list_aug);
complement_demand_pickup_cons = [];
if ~isempty(customer_id_assigned_trip)
    [~,assigned_trip_customer] = ismember(customer_id_assigned_trip,customer_id_aug);
    complement_demand_pickup_cons = complement_demand_equ(assigned_trip_customer(assigned_trip_customer>0),:);
end
clear model;
%
model.A = sparse([...
    %     trip_vehicle_mat complement_demand_inequ_v zeros(fleet_size,fleet_size);
    %     trip_demand_mat complement_demand_inequ_c zeros(size(customer_list_aug,1),fleet_size);
    trip_demand_mat complement_demand_equ zeros(size(customer_list_aug,1),fleet_size);
    trip_vehicle_mat zeros(fleet_size,size(customer_list_aug,1)) complement_vehicle_equ;
    zeros(size(complement_demand_pickup_cons,1),size(trip_demand_mat,2)) complement_demand_pickup_cons zeros(size(complement_demand_pickup_cons,1),fleet_size)]);
model.rhs = [...
    %     b_vehicle_const;
    %     b_demand_const;
    b_demand_complement;
    b_vehicle_complement;
    zeros(size(complement_demand_pickup_cons,1),1)];
%
%
model.sense = [...
    %     repmat('<', size(trip_vehicle_mat,1)+size(trip_demand_mat,1), 1); ...
    repmat('=', size(trip_demand_mat,1), 1);
    repmat('=', size(trip_vehicle_mat,1), 1);
    repmat('=', size(complement_demand_pickup_cons,1), 1)];

% model.obj = [...
%     (1-w_balance)*(0.9*trip_total_delay+0.1*trip_total_wait)-w_balance*10000*(unnormal_kl_vehicle);
%     10000000*(in_pool_time+1);
%     -w_balance*10000*(unnormal_kl_empty_vehicle);
%     ];
model.obj = [...
    (1-w_balance)*(trip_total_fuel)-w_balance*10000*(unnormal_kl_vehicle);
    10000000*(in_pool_time+1);
    -w_balance*10000*(unnormal_kl_empty_vehicle);
    ];
model.modelsense = 'min';
model.vtype = variableTypeChar;
params.outputflag = 0;
result = gurobi(model, params);
toc
trip_assignment = result.x(1:size(trip_vehicle_mat,2));
demand_ignore_id = result.x(size(trip_vehicle_mat,2)+1:size(trip_vehicle_mat,2)+size(customer_list_aug,1));
ignored_customer_id = customer_id_aug(demand_ignore_id>0);

assigned_trip = vehicle_trip_cons_fil(trip_assignment==1);
% rider_number = cellfun(@(x) size(x.od_set,1),assigned_trip);
% ignored_demand = sum(demand_ignore_id);

for ignore_id = 1:length(ignored_customer_id)
    customer_list_aug{customer_id_aug==ignored_customer_id(ignore_id)}.ignore = 1;
end

customer_list_out = customer_list_aug;
vehicle_list_out = vehicle_list;
vehicle_list_id = cellfun(@(x) x.vehicle_id,vehicle_list_out);
customer_list_id = cellfun(@(x) x.customer_id,customer_list_out);
veh_assigned = cellfun(@(x) x.vehicle_id,assigned_trip);
% for trip_id = 1:size(assigned_trip,1)
% %     if length(unique(assigned_trip{trip_id}.customer_location_sequence))>1
% %         assigned_trip{trip_id}=assigned_trip{trip_id}.reconstruct_route(tt_mat,link_uid,map);
%         vehicle_list_out{vehicle_list_id==assigned_trip{trip_id}.vehicle_id} = vehicle_list_out{vehicle_list_id==assigned_trip{trip_id}.vehicle_id}.assign_trip(assigned_trip{trip_id},customer_list_aug);
%         vehicle_list_out{vehicle_list_id==assigned_trip{trip_id}.vehicle_id}.in_use = 1;
%         for customer_trip_id = 1:length(assigned_trip{trip_id}.customer_id_list)
%             customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.max_wait_time ...
%                 =assigned_trip{trip_id}.wait_time_list(customer_trip_id);
% 			customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.assign = 1;
%             %         if assigned_trip{trip_id}.delay_time_list(customer_trip_id)~=0
%             %             customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.max_delay_time ...
%             %                 =customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.max_delay_time-customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.max_wait_time;
%             %         end
%         end
% %     end
% end
for vehicle_id = 1:size(vehicle_list_out,1)
    if ismember(vehicle_list_id(vehicle_id),veh_assigned)
        vehicle_list_out{vehicle_list_id==vehicle_list_id(vehicle_id)} = vehicle_list_out{vehicle_list_id==vehicle_list_id(vehicle_id)}.assign_trip(assigned_trip{veh_assigned==vehicle_list_id(vehicle_id)},customer_list_aug);
        vehicle_list_out{vehicle_list_id==vehicle_list_id(vehicle_id)}.in_use = 1;
        for customer_trip_id = 1:length(assigned_trip{veh_assigned==vehicle_list_id(vehicle_id)}.customer_id_list)
            customer_list_out{customer_list_id==assigned_trip{veh_assigned==vehicle_list_id(vehicle_id)}.customer_id_list(customer_trip_id)}.max_wait_time ...
                =assigned_trip{veh_assigned==vehicle_list_id(vehicle_id)}.wait_time_list(customer_trip_id);
            customer_list_out{customer_list_id==assigned_trip{veh_assigned==vehicle_list_id(vehicle_id)}.customer_id_list(customer_trip_id)}.assign = 1;
            %         if assigned_trip{trip_id}.delay_time_list(customer_trip_id)~=0
            %             customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.max_delay_time ...
            %                 =customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.max_delay_time-customer_list_out{customer_list_id==assigned_trip{trip_id}.customer_id_list(customer_trip_id)}.max_wait_time;
            %         end
        end
    else
        vehicle_list_out{vehicle_list_id==vehicle_list_id(vehicle_id)} = vehicle_list_out{vehicle_list_id==vehicle_list_id(vehicle_id)}.park_vehicle();
    end
end
customer_list_out = customer_list_out(1:size(customer_list,1));
vehicle_list_out = [vehicle_list_out;vehicle_list_rebalance];
customer_list_out = [customer_list_out;customer_list_rebalance;customer_list_ignore];

vehicle_trip_nonempty = (cellfun(@(x) isa(x.trip,'Trip'),vehicle_list_out));
vehicle_in_use = (cellfun(@(x) x.in_use,vehicle_list_out));
vehicle_trip_nonempty = vehicle_trip_nonempty & vehicle_in_use;
vehicle_trip = cellfun(@(x) x.trip,vehicle_list_out(vehicle_trip_nonempty),'uniformoutput',false);
customer_id_assigned_trip = cellfun(@(x) x.customer_id_list,vehicle_trip,'uniformoutput',false);
customer_id_assigned_trip = vertcat(customer_id_assigned_trip{:});
if length(customer_id_assigned_trip) - length(unique(customer_id_assigned_trip))>0
    fprintf('error\n')
end
if length(veh_assigned) - length(unique(veh_assigned))>0
    fprintf('error\n')
end