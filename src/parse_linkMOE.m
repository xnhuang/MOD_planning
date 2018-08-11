function route_map = parse_linkMOE(route_map,varargin)

fc_model_index = find(cellfun(@(x) strcmp(x,'model_parameter'),varargin));
assert(~isempty(fc_model_index),'need fuel consumption model for eco routing and path evaluation')

model_parameter = varargin{fc_model_index+1};
std_input_param = model_parameter.std_input_param;
gmm_cell = model_parameter.gmm_cell;
spd_limit_list = model_parameter.spd_limit_list;

traffic_cost_index = find(cellfun(@(x) strcmp(x,'traffic_cost'),varargin));
if ~isempty(traffic_cost_index)
    db_name_index = find(cellfun(@(x) strcmp(x,'db_name'),varargin));
    if ~isempty(db_name_index)
        db_name = varargin{db_name_index+1};
        folder_path_index = find(cellfun(@(x) strcmp(x,'folder_path'),varargin));
        folder_path = varargin{folder_path_index+1};

        driver = 'org.sqlite.JDBC';
        query_linkMOE = 'SELECT link_uid,link_type,start_time,end_time,link_travel_time,link_travel_time_standard_deviation,link_speed_ratio FROM LinkMOE';
        databaseurl = ['jdbc:sqlite:',folder_path,db_name];
        conn = database(db_name,[],[],driver,databaseurl);
        datatype = 'table';
        setdbprefs('DataReturnFormat',datatype);
        curs_linkMOE = exec(conn,query_linkMOE);
        curs_linkMOE = fetch(curs_linkMOE);
        link_MOE_table = curs_linkMOE.data;

        route_map.link_speed = link_MOE_table.Average_speed;
    else
        file_name_index = find(cellfun(@(x) strcmp(x,'mat_file_name'),varargin));
        file_name = varargin{file_name_index+1};
        load(file_name);
%         route_map.link_speed = route_map.link_speed_limit;
        historical_link_id = link_spd_historical(:,1);
        link_id_undir = route_map.link_uid;
        for link_id = 1:length(historical_link_id)
            record_link_uid = historical_link_id(link_id);
            link_speed_record = link_spd_historical(link_id,2);
%             link_count_record = mean_vehicle_count(link_id);
            link_density_record = mean_density(link_id);
            if ~isempty(link_id_undir(link_id_undir==record_link_uid))
                route_map.link_list{link_id_undir==record_link_uid}=route_map.link_list{link_id_undir==record_link_uid}.update_link_status(link_speed_record,link_density_record);
            end
        end
    end
else
    route_map.link_speed = route_map.link_speed_limit;
end

% augment state-action pair with fuel consumption cost
current_link = mat2cell(route_map.link_uid,ones(length(route_map.link_uid),1));
current_link_aug = cellfun(@(x) repmat(x,length(route_map.get_adj_e(x)),1),current_link,'UniformOutput',false);

% current_link_aug = cellfun(@(x,y) repmat(x,length(y),1),current_link,route_map.adj_e','UniformOutput',false);
adj_link = cellfun(@(x) x.adj_upstream_uid_list,route_map.link_list,'uniformoutput',false);

adj_link = vertcat(adj_link{:});
current_link = vertcat(current_link_aug{:});
[~,member_index_current] = ismember(current_link,route_map.link_uid);
[~,member_index_adj] = ismember(adj_link,route_map.link_uid);

current_link_spd = cellfun(@(x) x.current_speed,route_map.link_list(member_index_current));
% route_map.link_list{member_index_current}.current_speed;
adj_link_spd = cellfun(@(x) x.current_speed,route_map.link_list(member_index_adj));
spd_limit = cellfun(@(x) x.speed_limit,route_map.link_list(member_index_adj));
link_length = cellfun(@(x) x.link_length,route_map.link_list(member_index_adj));
link_grade = cellfun(@(x) x.link_grade,route_map.link_list(member_index_adj));

% adj_link_spd = route_map.link_speed(member_index_adj);
% spd_limit = route_map.link_speed_limit(member_index_adj);
% link_length = route_map.link_length(member_index_adj);
% link_grade = route_map.grade(member_index_adj);
spd_chg2 = current_link_spd.^2 - adj_link_spd.^2;
fc_input = [adj_link_spd, spd_chg2, link_grade, link_length];
fc_cost = fc_estimation(fc_input,spd_limit,std_input_param, gmm_cell,spd_limit_list);
route_map.link_uid_adj_aug = current_link;
route_map.link_uid_adj = adj_link;
route_map.fuel_cost = fc_cost;
