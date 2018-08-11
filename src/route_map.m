classdef route_map
    properties
        node
        node_location
        link
        link_uid
        adj_v
        adj_e_head
        adj_e_tail
        link_center_location
        fuel_cost
        link_uid_adj_aug
        link_uid_adj
        link_list
    end
    
    methods
        function obj = route_map(a,b,link_uid,link_length,speed_limit,lane_number,link_type,grade)
                %GRAPH graph class constructor
                % g = graph: consturct an empty (directed) graph
                % g = graph(g0): construct a copy of graph g0
                % g = graph(v,e,link_uid): construct a graph
                % where
                %   a:  a row vector of nodes; eg., [0,1,...,n]
                %       note that node names start with 0; i.e., v(1) = 0!
                %   b:  a column vector of edges; eg., [[0,1];[0,2];[1,3];...]
                %   link_uid:  a column vector of link_id
            if nargin > 8
                error('Error: Number of arguments > 8');
            elseif nargin == 0  % enmpty graph
                obj.node = [];
                obj.link = [];
                obj.link_uid = [];
                obj.adj_v = cell(0,0);
                obj.adj_e_head = cell(0,0);
                obj.adj_e_tail = cell(0,0);
            elseif nargin == 1 && isa(a,'graph') % copy of graph a
                obj = a;
            elseif nargin ==  8
                if isnumeric(a) && size(a,1) == 1
                    if isnumeric(b) && size(b,2) == 2
                        n = size(a,2);
                        m = size(b,1);
                        obj.node = a;
                        obj.link = b;
                        obj.link_uid = link_uid;
                        obj.adj_v = cell(1,n);
                        obj.adj_v(:) = {[]};
                        obj.adj_e_head = cell(1,n);
                        obj.adj_e_head(:) = {[]};
                        obj.adj_e_tail = cell(1,n);
                        obj.adj_e_tail(:) = {[]};
                        obj.link_list = cell(m,1);
                        
                        for j = 1:m
                            edge = obj.link(j,:);
                            if ismember(edge(1),obj.node) && ismember(edge(2),obj.node)   % edge validity check
                                obj.adj_v{obj.node==edge(2)} = [obj.adj_v{obj.node==edge(2)},edge(1)];
                                adj_e = obj.link_uid(obj.link(:,2)==edge(1) & obj.link(:,1)~=edge(2));
                                adj_e_next = obj.link_uid(obj.link(:,1)==edge(2) & obj.link(:,2)~=edge(1));
%                                 obj.adj_e{j} = obj.link_uid(obj.link(:,2)==edge(1));

                                edge_head_id = find(obj.node==edge(1));
                                edge_tail_id = find(obj.node==edge(2));
                                for i=1:length(edge_head_id)
                                    obj.adj_e_head{edge_head_id(i)} = [obj.adj_e_head{edge_head_id(i)},obj.link_uid(j)];
                                end
                                for i=1:length(edge_tail_id)
                                    obj.adj_e_tail{edge_tail_id(i)} = [obj.adj_e_tail{edge_tail_id(i)},obj.link_uid(j)];
                                end
                            else
                                error('Error: Edges contain non-existing nodes.');
                            end
                            obj.link_list{j} = Link(obj.link_uid(j),adj_e,adj_e_next);
                            obj.link_list{j} = obj.link_list{j}.assign_link_attribute(speed_limit(j),lane_number(j),link_length(j),link_type{j}, grade(j));
                        end
                    else
                        error('Error: Edge set must be in the form [[0,1];[0,2];...]');
                    end
                else
                    error('Effor: Node set must be row vector [0,1,...,n]');
                end
            end
        end
        %% accessor
        function v = get_v(obj)
            v=obj.node;
        end
        function e = get_e(obj)
            e=obj.link;
        end
        function e = get_edge(obj,v1,v2)
            % get edge from v1 to v2
            if ismember([v1,v2],obj.link,'rows')
                [~,index] = ismember([v1,v2],obj.link,'rows');
                e=obj.link_uid(index);
            else
                error('Effor: Invalid link');
            end
        end
        function uid = get_link_uid(obj)
            uid=obj.link_uid;
        end
        function v = get_adj_v(obj,v)
            v=obj.adj_v{obj.node==v};
        end
        function e = get_adj_e(obj,link_uid)
            e=obj.link_list{obj.link_uid==link_uid}.adj_upstream_uid_list;
        end
        function e = get_adj_e_next(obj,link_uid)
            e=obj.link_list{obj.link_uid==link_uid}.adj_downstream_uid_list;
        end
        function e = get_adj_e_head(obj,v)
            e=obj.adj_e_head{obj.node==v};
        end
        function e = get_adj_e_tail(obj,v)
            e=obj.adj_e_tail{obj.node==v};
        end
    end
end