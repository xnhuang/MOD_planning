clear,close all;clc;
save_results = 0;
load_test_case = 1;
idle_balance = 0;
current_path = [pwd,'\'];
addpath('src')

traffic_state_path = strrep(current_path,'MOD_planning','polaris_result_17_18');
load([traffic_state_path,'polaris_17_18_trip_location_associate_link.mat'])
partition_path = strrep(current_path,'RegionRideShare\MOD_planning','TrafficModel\network_partition\demand_prior');
% partition_path = strrep(current_path,'RegionRideShare\Mora2017_implementation','TrafficModel\network_partition');
all2alltt_path = strrep(current_path,'\MOD_planning','\offline_travel_time');
cluster_number = 30;
% load([partition_path,'Link_Center_kmeans_ClusterNum_',num2str(cluster_number),'.mat']);
load([partition_path,'EMGMM_combineOD_ClusterNum_',num2str(cluster_number),'_kmeans_start_mds_20_sym_min_metricstress.mat']);
test_case_path = strrep(current_path,'MOD_planning','test_case');

% graph_db_name = 'annarbor-Supply.sqlite';
% network_db_path = strrep(current_path,'RegionRideShare\Mora2017_implementation','Data\Network');
% map = parse_network(graph_db_name,network_db_path);
%
% load('gmr_spd_lim_ind_p00005_idle_fil_resample_test.mat');
% fc_mdl = struct('fc_mdl','fc_mdl_parameter');
% [fc_mdl(:).std_input_param]                  = deal(std_input_param);
% [fc_mdl(:).gmm_cell]                         = deal(layer1_unsup_gmm_cell);
% [fc_mdl(:).spd_limit_list]                   = deal(spd_limit_list);
% result_db_name = 'polaris_17_18_linkMOE_mean.mat';
% map = parse_linkMOE(map,'model_parameter',fc_mdl,'traffic_cost','mat_file_name',result_db_name);
load([traffic_state_path,'polaris_17_18_linkMOE_mean_HCM_process.mat']);
load([all2alltt_path,'all2all_eco_time_fc.mat'])   % all2all travel time from polaris simulation historical cost
eco_cost = {tt_mat,fc_mat};
load([all2alltt_path,'all2all_fast_time_fc.mat'])   % all2all travel time from polaris simulation historical cost
time_cost = {tt_mat,fc_mat};
routing_cost = {time_cost,eco_cost};
% filter demand according to link list
location_list_link = 2*location_list.link+location_list.dir;
location_list = location_list(ismember(location_list_link,link_uid),:);
trips = trips(ismember(trips.origin,location_list.location)&ismember(trips.destination,location_list.location),:);

[~,destination_all_id] = ismember(trips.destination,location_list.location);
destination_all_list = 2*location_list.link(destination_all_id)+location_list.dir(destination_all_id);
[~,origin_all_id] = ismember(trips.origin,location_list.location);
origin_all_list = 2*location_list.link(origin_all_id)+location_list.dir(origin_all_id);

[state_matrixX,state_matrixY] = meshgrid([1:cluster_size],[1:cluster_size]);
state_matrix = state_matrixX*100+state_matrixY;
state_list = reshape(state_matrix,[],1);
% characterize demand distribution with partition assuming static demand
% distribution
[~,origin_index] = ismember(origin_all_list,link_uid);
[~,destination_index] = ismember(destination_all_list,link_uid);
origin_demand_region = cluster_index(origin_index);
destination_demand_region = cluster_index(destination_index);
demand_region = 100*origin_demand_region+destination_demand_region;
demand_region_count = histc(demand_region,state_list);
demand_region_normal = demand_region_count/sum(demand_region_count);
demand_state_matrix = reshape(demand_region_normal,cluster_size,cluster_size);
demand_region_matrix = reshape(state_list,cluster_size,cluster_size);

%% initialize fleet and demand
discount_factor = 1;
balance_weight = 0;
balance_weight_idle = 0.99;
vehicle_capacity = 4;
fleet_size = 900;
max_clique_size = vehicle_capacity;
max_wait_time = 3*60;
max_delay_time = 6*60;
w_wait = 0.5;
demand_sample_ratio = 0.2;
duration = 30;
time_step = 30;
time_step_size = 1;

rebalance_radius = 2;
% routing policy: 1:fastest; 2:eco
routing_policy_assign = 1;
routing_policy_rebalance = 1;
routing_policy_rebalance_idle = 1;
test_case_name = ['test_demand_ratio_',num2str(demand_sample_ratio),...
    '_fleet_',num2str(fleet_size),...
    '_capacity_',num2str(vehicle_capacity),...
    '_duration_',num2str(duration),...
    '_waittime_',num2str(max_wait_time),...
    '_timestep_',num2str(time_step),'.mat'];
% define vehicle and customer structure
% vehicle_location_initial = destination_all_list(randi([1,length(destination_all_list)],fleet_size,1));
% duration = 30;
if load_test_case == 0
    vehicle_location_initial = origin_all_list(randi([1,length(origin_all_list)],fleet_size,1));
    vehicle_list = cell(fleet_size,1);
    for vehicle_total_id = 1:fleet_size
        capacity = vehicle_capacity;
        location = vehicle_location_initial(vehicle_total_id);
        onboard = [];
        trip = [];
        vehicle_list{vehicle_total_id} = Vehicle(vehicle_total_id,capacity,location,onboard,trip);
    end
else
    if exist([test_case_path,test_case_name],'file')~=0
        load([test_case_path,test_case_name]);
    else
        error('test case not exist');
    end
end
vehicle_list = [];
customer_id_offset = 0;
customer_list = {};
customer_list_ignore = {};
current_time = 17*3600+30*time_step_size;
time_grid_demand_generation = current_time:time_step:current_time+duration*60;
time_grid = current_time:time_step*time_step_size:current_time+duration*60;
link_num_count = zeros(length(time_grid)+1,length(link_uid));
link_speed = zeros(length(time_grid)+1,length(link_uid));
vehicle_location = zeros(length(time_grid)+1,fleet_size);
vehicle_departed = zeros(length(time_grid)+1,fleet_size);
customer_log = cell(length(time_grid),1);
vehicle_log = cell(length(time_grid),1);
customer_delivered_log = cell(length(time_grid),1);
ignored_customer_id_log = cell(length(time_grid),1);
rebalance_customer_id_log = cell(length(time_grid),1);

vehicle_planned_customer_log = cell(length(time_grid),fleet_size);
vehicle_onboard_customer_log = cell(length(time_grid),fleet_size);
feasible_trip_log = cell(length(time_grid),1);
link_list_log = cell(length(time_grid),1);

vehicle_rebalance_log = zeros(length(time_grid),fleet_size);
vehicle_id_log = zeros(length(time_grid),fleet_size);
vehicle_total_distance = zeros(length(time_grid),fleet_size);

vehicle_onboard_num = zeros(length(time_grid)+1,fleet_size);
vehicle_onboard_num(1,:) = 0;
vehicle_assigned_num = zeros(length(time_grid)+1,fleet_size);
vehicle_assigned_num(1,:) = 0;
vehicle_onlinktime = zeros(length(time_grid)+1,fleet_size);
vehicle_onlinktime(1,:) = 0;
map_link_uid = cellfun(@(x) x.link_uid,map.link_list);
[~,link_uid_index] = ismember(link_uid,map_link_uid);
link_uid_link = map.link_list(link_uid_index);
link_num_count(1,:) = cellfun(@(x) x.current_vehicle_count,link_uid_link);
link_speed(1,:) = cellfun(@(x) x.current_speed,link_uid_link);
% vehicle_total_id = cellfun(@(x) x.vehicle_id,vehicle_list);
% vehicle_location(1,vehicle_total_id) = cellfun(@(x) x.location,vehicle_list);
total_customer = 0;

fleet_controller = FleetController(...
            routing_cost,...
            [link_uid,cluster_index],...
            cluster_centroid_link,...
            map,...
            routing_policy_assign,...
            routing_policy_rebalance,...
            routing_policy_rebalance_idle,...
            time_step,...
            max_clique_size,...
            w_wait,...
            balance_weight,...
            balance_weight_idle,...
            discount_factor,...
            idle_balance,...
            rebalance_radius);
        
for time_id = 1:length(time_grid)
    current_time = time_grid(time_id);
    if load_test_case == 0
        time_lb = time_grid(time_id)-time_step;
        time_hb = time_grid(time_id);
        
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
            current_wait_time = inf;
            current_delay_time = inf;
            customer_list_new{customer_id} = Customer(customer_id+customer_id_offset,origin,destination,max_wait_time,max_delay_time);
        end
        customer_id_offset = max(cellfun(@(x) x.customer_id,customer_list_new));
    else
        
        customer_list_new = customer_list_log(time_grid_demand_generation>=current_time&time_grid_demand_generation<current_time+time_step*time_step_size);
        customer_list_new = vertcat(customer_list_new{:});
        for read_customer_id = 1:numel(customer_list_new)
            customer_list_new{read_customer_id}.max_wait_time = customer_list_new{read_customer_id}.max_wait_time-(current_time-customer_list_new{read_customer_id}.enter_time);
            customer_list_new{read_customer_id}.max_delay_time = customer_list_new{read_customer_id}.max_delay_time-(current_time-customer_list_new{read_customer_id}.enter_time);
            customer_list_new{read_customer_id}.max_wait_time(customer_list_new{read_customer_id}.max_wait_time<=0) = 0;
            customer_list_new{read_customer_id}.max_delay_time(customer_list_new{read_customer_id}.max_delay_time<=0) = 0;
        end
    end
    customer_list = [customer_list;customer_list_new];
    [vehicle_list,fleet_controller] = fleet_controller.fleet_plan_no_share(...
            customer_list, ...
            customer_list_new,...
            vehicle_list, ...
            current_time);
    [vehicle_list,map] = network_sim(vehicle_list, map, time_step_size, time_step, current_time);
    [vehicle_list,customer_list,customer_delivered] = network_sim_update_demand(vehicle_list, customer_list,time_step);
    fleet_controller = fleet_controller.update_map(map);
    %   general fleet state
    total_customer = total_customer+size(customer_list_new,1);
    onboard = cellfun(@(x) size(x.onboard,1),vehicle_list);
    total_onboard = sum(onboard);
    total_onboard_log(time_id) = total_onboard;
    delivered = total_customer-total_onboard;
    delivered_log(time_id) = delivered;
    new_demand = size(customer_list_new,1);
    new_demand_id = cellfun(@(x) x.customer_id,customer_list_new);
    customer_pool_size = size(customer_list(cellfun(@(x) x.ignore==0&x.rebalance==0,customer_list)),1);
    idle_vehicle_num = sum(cellfun(@(x) x.in_use==0,vehicle_list));
    idle_balance_vehicle_num = sum(cellfun(@(x) isa(x.trip,'Trip')&x.in_use==0,vehicle_list));
    fprintf('=============================================================\n');
    fprintf('simulation time = %d\n',time_id);
    fprintf('customer pool size = %d\n',customer_pool_size);
    fprintf('new demand number = %d\n',new_demand);
    fprintf('idling fleet size = %d\n',idle_vehicle_num);
    fprintf('onboard customer number = %d\n',total_onboard);
    fprintf('delivered customer number = %d\n',delivered);
    fprintf('ignore demand number = %d\n',sum(cellfun(@(x) x.ignore,customer_list))-sum(cellfun(@(x) x.rebalance,customer_list)));
    fprintf('rebalance demand number = %d\n',sum(cellfun(@(x) x.rebalance,customer_list)));
    fprintf('rebalancing fleet size = %d\n',sum(cellfun(@(x) x.rebalance,vehicle_list)));
    fprintf('idling rebalancing fleet size = %d\n',idle_balance_vehicle_num);
    fprintf('vehicle total travel dist[m] = %d\n',sum(cellfun(@(x) x.distance_traveled,vehicle_list)));
    fprintf('=============================================================\n');
    vehicle_trip_nonempty = (cellfun(@(x) isa(x.trip,'Trip'),vehicle_list));
    vehicle_in_use = (cellfun(@(x) x.in_use,vehicle_list));
    vehicle_trip_nonempty = vehicle_trip_nonempty & vehicle_in_use;
    vehicle_trip = cellfun(@(x) x.trip,vehicle_list(vehicle_trip_nonempty),'uniformoutput',false);
    customer_id_assigned_trip = cellfun(@(x) x.customer_id_list,vehicle_trip,'uniformoutput',false);
    customer_id_assigned_trip = vertcat(customer_id_assigned_trip{:});
    fprintf('customer assignment gap rebalance = %d\n',length(customer_id_assigned_trip) - length(unique(customer_id_assigned_trip)));

    %   assign trip
%     [customer_list,vehicle_list,feasible_trip_log{time_id},customer_graph_last,customer_graph_id_last,vehicle_graph_last,vehicle_graph_id_last] = trip_matching_pass_adj_mat(vehicle_list,customer_list,eco_cost,link_uid,map,w_wait,max_clique_size,current_time,cluster_index,demand_state_matrix,balance_weight,time_step,discount_factor,customer_graph_last,customer_graph_id_last,vehicle_graph_last,vehicle_graph_id_last,routing_policy);
        
%     [customer_list,vehicle_list] = trip_matching_rebalance(vehicle_list,customer_list,time_cost,link_uid,map,w_wait,current_time,routing_policy_rebalance);
%     if idle_balance == 1
%         fprintf('empty vehicle rebalance\n');
%         tic
%         vehicle_list = trip_matching_rebalance_idle(vehicle_list,customer_list,tt_mat,link_uid,map,w_wait,current_time,cluster_index,cluster_centroid_link,demand_state_matrix,0.9,time_step,cluster_center_trip,0.1);
%         toc
%     end
    

            
    vehicle_trip_nonempty = (cellfun(@(x) isa(x.trip,'Trip'),vehicle_list));
    vehicle_in_use = (cellfun(@(x) x.in_use,vehicle_list));
    vehicle_trip_nonempty = vehicle_trip_nonempty & vehicle_in_use;
    vehicle_trip = cellfun(@(x) x.trip,vehicle_list(vehicle_trip_nonempty),'uniformoutput',false);
    customer_id_assigned_trip = cellfun(@(x) x.customer_id_list,vehicle_trip,'uniformoutput',false);
    customer_id_assigned_trip = vertcat(customer_id_assigned_trip{:});
    
    fprintf('customer assignment gap assignment = %d\n',length(customer_id_assigned_trip) - length(unique(customer_id_assigned_trip))); 
    
    
    link_list_log{time_id} = map.link_list;
    customer_delivered_log{time_id} = customer_delivered;
    customer_list_id = cellfun(@(x) x.customer_id,customer_list);
    ignored = cellfun(@(x) x.ignore,customer_list);
    vehicle_trip_nonempty = (cellfun(@(x) isa(x.trip,'Trip'),vehicle_list));
    vehicle_trip = cellfun(@(x) x.trip,vehicle_list(vehicle_trip_nonempty),'uniformoutput',false);
    customer_trip = cellfun(@(x) x.customer_id_list,vehicle_trip,'uniformoutput',false);
    customer_trip = vertcat(customer_trip{:});
    ignored_num = sum(ignored);
    on_time_satisfied_demand = length(new_demand_id(ismember(new_demand_id,customer_trip)));
    rebalance = cellfun(@(x) x.rebalance,customer_list);
    miss_demand = sum(ignored)-sum(rebalance);
    
    rebalance_customer_id_log{time_id} = customer_list_id(rebalance==1);
    ignored_customer_id_log{time_id} = customer_list_id(ignored==1);
    link_uid_link = map.link_list(link_uid_index);
    link_num_count(time_id+1,:) = cellfun(@(x) x.current_vehicle_count,link_uid_link);
    link_speed(time_id+1,:) = cellfun(@(x) x.current_speed,link_uid_link);
    vehicle_total_id = cellfun(@(x) x.vehicle_id,vehicle_list);
    vehicle_location(time_id+1,vehicle_total_id) = cellfun(@(x) x.location,vehicle_list);
    vehicle_departed(time_id+1,vehicle_total_id) = cellfun(@(x) x.departed,vehicle_list);
    vehicle_onboard_num(time_id+1,vehicle_total_id) = cellfun(@(x) size(x.onboard,1),vehicle_list);
    vehicle_assigned_num(time_id+1,vehicle_total_id) = 0;
    vehicle_trip_nonempty = (cellfun(@(x) isa(x.trip,'Trip'),vehicle_list));
    vehicle_id_nonempty = vehicle_total_id(vehicle_trip_nonempty>0);
    assigned_num = cellfun(@(x) length(x.trip.customer_id_list),vehicle_list(vehicle_trip_nonempty>0));
    vehicle_assigned_num(time_id+1,vehicle_id_nonempty) = assigned_num;
    vehicle_onlinktime(time_id+1,vehicle_total_id) = cellfun(@(x) x.on_link_time,vehicle_list);
    on_time_satisfied_demand_log(time_id) = on_time_satisfied_demand;
    vehicle_log{time_id} = vehicle_list;
    customer_log{time_id} = customer_list;
    for vehicle_id = 1:length(vehicle_total_id)
        vehicle = vehicle_list{vehicle_total_id==vehicle_id};
        if isa(vehicle.trip,'Trip')
            vehicle_planned_customer_log{time_id,vehicle_id} = sort(vehicle.trip.customer_id_list);
        end
        if ~isempty(vehicle.onboard) && isa(vehicle.onboard{1},'Customer')
            vehicle_onboard_customer_log{time_id,vehicle_id} = sort(cellfun(@(x) x.customer_id,vehicle.onboard));
        end
        vehicle_rebalance_log(time_id,vehicle_id) = vehicle.rebalance;
        vehicle_id_log(time_id,vehicle_id) = vehicle.vehicle_id;
        
    end
end

save_name = ['demandRatio_',num2str(demand_sample_ratio),...
    '_fleet_',num2str(fleet_size),...
    '_duration_',num2str(duration),...
    '_timestep_',num2str(time_step*time_step_size),...
    '_waittime_',num2str(max_wait_time),...
    '_balanceWeight_',num2str(balance_weight),...
    '_clusterNumber_',num2str(cluster_number),...
    '_discountFactor_',num2str(discount_factor),...
    '_idleBalance_',num2str(idle_balance),'_noshare_time.mat'];

save_path = 'eco_cost_result\';

if save_results
    if ~exist(save_path,'dir')
        mkdir(save_path);
    end
    save([save_path,save_name],...
        'vehicle_rebalance_log',...
        'vehicle_onboard_customer_log',...
        'vehicle_planned_customer_log',...
        'on_time_satisfied_demand_log',...
        'vehicle_onboard_num',...
        'vehicle_assigned_num',...
        'vehicle_location',...
        'vehicle_departed',...
        'vehicle_onlinktime',...
        'total_onboard_log',...
        'delivered_log',...
        'time_grid',...
        'vehicle_log',...
        'customer_log',...
        'rebalance_customer_id_log',...
        'ignored_customer_id_log',...
        'customer_delivered_log',...
        'feasible_trip_log');
end