classdef Link    
    properties
        link_uid;
        speed_limit;
        speed_limit_mph;
        lane_number;
        link_length;
        link_type;
        link_grade;
        adj_upstream_uid_list;
        adj_downstream_uid_list;
        current_speed;
        current_vehicle_count;
        current_density;
        model_sim_fail;
    end
    
    methods
        function obj = Link(link_uid,adj_upstream_uid_list,adj_downstream_uid_list)
            obj.link_uid = link_uid;
            obj.adj_upstream_uid_list = adj_upstream_uid_list;
            obj.adj_downstream_uid_list = adj_downstream_uid_list;
            obj.model_sim_fail=0;
        end
        function obj = assign_link_attribute(obj,speed_limit,lane_number,link_length,link_type,link_grade)
            obj.speed_limit = mph2mps(round(mps2mph(speed_limit)));
            obj.speed_limit_mph = round(mps2mph(obj.speed_limit));
            obj.lane_number = max(lane_number,1);
            obj.link_length = link_length;
            obj.link_type = link_type;
            obj.link_grade = link_grade;
            obj.current_speed = speed_limit;
            obj.current_density = 0;
            obj.current_vehicle_count = 0;
        end
        function obj = update_link_status(obj, current_speed, current_density)
            obj.current_speed = current_speed;
            obj.current_vehicle_count = current_density*obj.link_length*obj.lane_number;
            obj.current_density = current_density;
        end
        
        function obj = update_link_spd(obj, current_density)
            obj.current_vehicle_count = current_density*obj.link_length*obj.lane_number;
            obj = obj.update_link_status_sim();
        end
        
        function obj = update_link_status_sim(obj)
            % update link travel speed based on fundamental diagram
            % parameters from HCM, urban approximated with midsection,
            % ignore delay due to heavy vehicle, traffic control, left and
            % right turn, use through-movement capacity as capacity for
            % urban link
            % input: current_vehicle_cout
            % output: current_density, current_speed
            obj.current_density = obj.current_vehicle_count/obj.lane_number/obj.link_length; % vehicle density, veh/ln/m
            density_mi = mi2m(obj.current_density);
            freeway_type_collection = {'EXPRESSWAY','COLLECTOR','RAMP'};
            highway_type_collection = {'MAJOR','MINOR'};
            urban_type_collection = {'LCOAL','MAJOR','MINOR'};
            iteration_limit = 20;
            current_speed_ini = obj.speed_limit_mph;
            ff_spd = 0;
            if ismember(obj.link_type,freeway_type_collection) && obj.speed_limit_mph>=55
                % freeway
                break_point_list = [1000:200:1800];
                capacity_list = [2400,2400,2350,2300,2250];
                coeff_list = [0.00001107,0.00001160,0.00001418,0.00001816,0.00002469];
                speed_limit_list = (75:-5:55);
                bffs = obj.speed_limit_mph+5;
                break_point = interp1(speed_limit_list,break_point_list,bffs);
                coeff = interp1(speed_limit_list,coeff_list,bffs);
                capacity = interp1(speed_limit_list,capacity_list,bffs);
                critical_density_mi = break_point/bffs;
                obj.current_speed = current_speed_ini;
%                 current_speed_last = obj.current_speed;
                for iteration_id = 1:iteration_limit
                    flow_rate = obj.current_speed.*density_mi;
                    flow_rate(flow_rate>=capacity) = capacity;
                    obj.current_speed = bffs-coeff*(flow_rate-break_point).^2;
                    obj.current_speed(density_mi<=critical_density_mi) = bffs;
%                     current_speed_cell{iteration_id} = current_speed;
%                     flow_rate_cell{iteration_id} = flow_rate;
%                     spd_change(iteration_id) = norm(current_speed-current_speed_last);
%                     current_speed_last = obj.current_speed;
                end
                %     flow_rate(density_mi<=critical_density_mi) = m2mi(speed_limit_mph*density_mi(density_mi<=critical_density_mi));
                ff_spd = bffs;
            elseif (ismember(obj.link_type,highway_type_collection) && obj.speed_limit_mph>=45) ...
                    || (ismember(obj.link_type,freeway_type_collection) && obj.speed_limit_mph<55)
                % highway
                divider_list = [800:-100:500];
                coeff_list = [5,3.78,3.49,2.78];
                speed_limit_list = (60:-5:45);
                capacity_list = [2200,2100,2000,1900];
                bffs = obj.speed_limit_mph;
                if bffs <50
                    bffs = bffs+7;
                else
                    bffs = bffs+5;
                end
                divider = interp1(speed_limit_list,divider_list,bffs);
                coeff = interp1(speed_limit_list,coeff_list,bffs);
                capacity = interp1(speed_limit_list,capacity_list,bffs);
                critical_density_mi = 1400/bffs;
                obj.current_speed = current_speed_ini;
%                 current_speed_last = current_speed;
                for iteration_id = 1:iteration_limit
                    flow_rate = obj.current_speed.*density_mi;
                    flow_rate(flow_rate>=capacity) = capacity;
                    obj.current_speed = bffs-coeff*((flow_rate-1400)/divider).^1.31;
                    obj.current_speed(density_mi<=critical_density_mi) = bffs;
%                     current_speed_cell{iteration_id} = current_speed;
%                     flow_rate_cell{iteration_id} = flow_rate;
%                     spd_change(iteration_id) = norm(current_speed-current_speed_last);
%                     current_speed_last = current_speed;
                end
                ff_spd = bffs;
            else
                % urban
                speed_limit_list = 25:5:55;
                speed_constant_list = [37.4,39.7,42.1,44.4,46.8,49.1,51.5];
                capacity = 1800;
                fcs = -0.9;
                fa = 0;
                speed_constant = interp1(speed_limit_list,speed_constant_list,obj.speed_limit_mph);
                ffspeed = speed_constant+fcs+fa;
                obj.current_speed = current_speed_ini;
%                 current_speed_last = obj.current_speed;
                for iteration_id = 1:iteration_limit
                    flow_rate = obj.current_speed.*density_mi;
                    flow_rate(flow_rate>=capacity) = capacity;
                    fv = 2./(1+(1-flow_rate/52.8/ffspeed).^0.21);
                    obj.current_speed = ffspeed./fv;
                    %         current_speed(density_mi<=critical_density_mi) = speed_limit_mph;
%                     current_speed_cell{iteration_id} = current_speed;
%                     flow_rate_cell{iteration_id} = flow_rate;
%                     spd_change(iteration_id) = norm(current_speed-current_speed_last);
%                     current_speed_last = current_speed;
                end
                %     current_speed = mph2mps(ffspeed/fv);
                ff_spd = ffspeed;
            end
            density_mi = flow_rate/obj.current_speed;
            obj.current_density = m2mi(density_mi);
%             if obj.speed_limit_mph < 40
%                 ffspd_polaris = obj.speed_limit_mph + 5;
%             elseif (obj.speed_limit_mph >= 40 && obj.speed_limit_mph < 50) 
%                 ffspd_polaris = obj.speed_limit_mph + 7;
%             else
%                 ffspd_polaris = obj.speed_limit_mph + 5;
%             end
%             obj.current_speed = obj.current_speed/ff_spd*ffspd_polaris;
            obj.current_speed = mph2mps(obj.current_speed);
            obj.model_sim_fail=0;
            if isnan(obj.current_speed)
                obj.current_speed = obj.speed_limit;
                obj.model_sim_fail=1;
            end
        end
    end
end

