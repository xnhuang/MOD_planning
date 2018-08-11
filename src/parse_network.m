function g = parse_network(db_name,folder_path)

load([folder_path,'link_feature_aaid_uniq_add_lane_num.mat']);
load([folder_path,'isolated_links.mat']);
driver = 'org.sqlite.JDBC';
query_link = 'SELECT * FROM Link';
query_node = 'SELECT * FROM Node';
databaseurl = ['jdbc:sqlite:',folder_path,db_name];
conn = database(db_name,[],[],driver,databaseurl);
datatype = 'table';
setdbprefs('DataReturnFormat',datatype);
curs_link = exec(conn,query_link);
curs_link = fetch(curs_link);
link_table = curs_link.Data;

curs_node = exec(conn,query_node);
curs_node = fetch(curs_node);
node_table = curs_node.Data;

node_list = node_table.node;
connection_list_direction_0 = [link_table.node_a, link_table.node_b];
connection_list_direction_1 = fliplr(connection_list_direction_0);
[~,cover_data_index] = ismember(link_property_table.Link,link_table.link);
elevation_change_no_dir = zeros(length(link_table.link),1);
elevation_change_no_dir(cover_data_index) = link_property_table.ElevationChange;
grade_no_dir = elevation_change_no_dir./link_table.length;

link_uid_list = [2*link_table.link; 2*link_table.link+1];
connection_list = [connection_list_direction_0;connection_list_direction_1];
length_list = [link_table.length;link_table.length];
link_speed_limit = [link_table.speed_ab;link_table.speed_ba];
lane_number = [link_table.lanes_ab;link_table.lanes_ba];
link_type = [link_table.type;link_table.type];
grade = [grade_no_dir;-grade_no_dir];

% link_uid_list = link_uid_list(lane_number>0 & link_speed_limit>11);
% connection_list = connection_list(lane_number>0 & link_speed_limit>11,:);
% length_list = length_list(lane_number>0 & link_speed_limit>11);
% link_speed_limit_list = link_speed_limit(lane_number>0 & link_speed_limit>11);
% link_type_list = link_type(lane_number>0 & link_speed_limit>11);
% lane_number_list = lane_number(lane_number>0 & link_speed_limit>11);
% grade_list = grade(lane_number>0 & link_speed_limit>11);
link_speed_limit(link_speed_limit<=mph2mps(25)) = mph2mps(25);
link_uid_list = link_uid_list(lane_number>0);
connection_list = connection_list(lane_number>0,:);
length_list = length_list(lane_number>0);
link_speed_limit_list = link_speed_limit(lane_number>0);
link_type_list = link_type(lane_number>0);
lane_number_list = lane_number(lane_number>0);
grade_list = grade(lane_number>0);

connection_list = connection_list(~isolated_links,:);
link_uid_list = link_uid_list(~isolated_links,:);
length_list = length_list(~isolated_links,:);
link_speed_limit_list = link_speed_limit_list(~isolated_links,:);
link_type_list = link_type_list(~isolated_links,:);
lane_number_list = lane_number_list(~isolated_links,:);
grade_list = grade_list(~isolated_links,:);

g = route_map(node_list',connection_list,link_uid_list,length_list,link_speed_limit_list,lane_number_list,link_type_list,grade_list);

utm_zone = ['17 T'];
utm_zone_list = repmat(utm_zone,length(node_table.x),1);
[lat,lon] = utm2deg(node_table.x,node_table.y,utm_zone_list);
g.node_location = [lat,lon];

[~,node_a_id] = ismember(connection_list(:,1),node_list);
[~,node_b_id] = ismember(connection_list(:,2),node_list);

link_node_a_loc = g.node_location(node_a_id,:);
link_node_b_loc = g.node_location(node_b_id,:);
g.link_center_location = (link_node_a_loc+link_node_b_loc)/2;