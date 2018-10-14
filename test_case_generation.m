clear,close all;clc;
addpath('src');
load('C:\Users\xnhuang\OneDrive - umich.edu\MultihopRideShare\RegionRideShare\Mora2017_implementation\polaris_17_18_trip_location_associate_link.mat')
load('C:\Users\xnhuang\OneDrive - umich.edu\MultihopRideShare\RegionRideShare\Mora2017_implementation\polaris_17_18_linkMOE_mean.mat')
save_results = 1;

current_path = [pwd,'\'];
partition_path = strrep(current_path,'RegionRideShare\MOD_planning','TrafficModel\network_partition');
load([partition_path,'partition_gmm_maxIter10000.mat']);

graph_db_name = 'annarbor-Supply.sqlite';
network_db_path = strrep(current_path,'RegionRideShare\MOD_planning','Data\Network');
map = parse_network(graph_db_name,network_db_path);

load('gmr_spd_lim_ind_p00005_idle_fil_resample_test.mat');
fc_mdl = struct('fc_mdl','fc_mdl_parameter');
[fc_mdl(:).std_input_param]                  = deal(std_input_param);
[fc_mdl(:).gmm_cell]                         = deal(layer1_unsup_gmm_cell);
[fc_mdl(:).spd_limit_list]                   = deal(spd_limit_list);
result_db_name = 'C:\Users\xnhuang\OneDrive - umich.edu\MultihopRideShare\RegionRideShare\Mora2017_implementation\polaris_17_18_linkMOE_mean.mat';
map = parse_linkMOE(map,'model_parameter',fc_mdl,'traffic_cost','mat_file_name',result_db_name);

load('..\offline_travel_time\all2all_tt_filter.mat')   % all2all travel time from polaris simulation historical cost
% filter demand according to link list
location_list_link = 2*location_list.link+location_list.dir;
location_list = location_list(ismember(location_list_link,link_uid),:);
trips = trips(ismember(trips.origin,location_list.location)&ismember(trips.destination,location_list.location),:);

[~,destination_all_id] = ismember(trips.destination,location_list.location);
destination_all_list = 2*location_list.link(destination_all_id)+location_list.dir(destination_all_id);
[~,origin_all_id] = ismember(trips.origin,location_list.location);
origin_all_list = 2*location_list.link(origin_all_id)+location_list.dir(origin_all_id);

[state_matrixX,state_matrixY] = meshgrid([1:cluster_size],[1:cluster_size]);
state_matrix = state_matrixX*10+state_matrixY;
state_list = reshape(state_matrix,[],1);
% characterize demand distribution with partition assuming static demand
% distribution
[~,origin_index] = ismember(origin_all_list,link_uid);
[~,destination_index] = ismember(destination_all_list,link_uid);
origin_demand_region = cluster_index(origin_index);
destination_demand_region = cluster_index(destination_index);
demand_region = 10*origin_demand_region+destination_demand_region;
demand_region_count = histc(demand_region,state_list);
demand_region_normal = demand_region_count/sum(demand_region_count);
demand_state_matrix = reshape(demand_region_normal,cluster_size,cluster_size);

%% initialize fleet and demand
vehicle_capacity = 4;
fleet_size = 900;
max_clique_size = vehicle_capacity;
max_wait_time = 3*60;
max_delay_time = 6*60;
w_wait = 0.5;
demand_sample_ratio = 0.2;
% define vehicle and customer structure
% vehicle_location_initial = destination_all_list(randi([1,length(destination_all_list)],fleet_size,1));
vehicle_location_initial = origin_all_list(randi([1,length(origin_all_list)],fleet_size,1));
vehicle_list = cell(fleet_size,1);
for vehicle_total_id = 1:fleet_size
    capacity = vehicle_capacity;
    location = vehicle_location_initial(vehicle_total_id);
    onboard = [];
    trip = [];
    vehicle_list{vehicle_total_id} = Vehicle(vehicle_total_id,capacity,location,onboard,trip);
end
current_time = 17*3600+30;
duration = 30;
time_step = 30;
time_grid = current_time:time_step:current_time+duration*60;
customer_list_log = cell(length(time_grid),1);
customer_id_offset = 0;
for time_id = 1:length(time_grid)
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
    origin_sample = origin_link_list(customer_id_list_sample);
    destination_sample = destination_link_list(customer_id_list_sample);
    enter_time_sample = trip_time_fil.start(customer_id_list_sample);
    [~,origin_index] = ismember(origin_sample,link_uid);
    [~,destination_index] = ismember(destination_sample,link_uid);
    tt_index = sub2ind(size(tt_mat),origin_index,destination_index);
    fastest_tt_sample = tt_mat(tt_index);

    parfor customer_id = 1:length(customer_id_list_sample)
        origin = origin_sample(customer_id);
        destination = destination_sample(customer_id);
        enter_time = enter_time_sample(customer_id);
        fastest_tt = fastest_tt_sample(customer_id);
        current_wait_time = inf;
        current_delay_time = inf;
        max_wait_time_cust = max_wait_time-(time_grid(time_id)-enter_time);
        max_delay_time_cust = max_delay_time-(time_grid(time_id)-enter_time);
        customer_list_new{customer_id} = Customer(customer_id+customer_id_offset,enter_time,origin,destination,max_wait_time_cust,max_delay_time_cust,0,fastest_tt);
    end
    customer_id_offset = max(cellfun(@(x) x.customer_id,customer_list_new));
    customer_list_log{time_id} = customer_list_new;
end

save_name = ['test_demand_ratio_',num2str(demand_sample_ratio),...
    '_fleet_',num2str(fleet_size),...
    '_capacity_',num2str(vehicle_capacity),...
    '_duration_',num2str(duration),...
    '_waittime_',num2str(max_wait_time),...
    '_timestep_',num2str(time_step),'.mat'];
save_path = '..\test_case\';
save([save_path,save_name],...
    'vehicle_list',...
    'customer_list_log');
