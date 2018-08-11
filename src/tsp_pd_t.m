function result = tsp_pd_t(vehicle,od_set,routing_cost_mat,link_uid,w_wait,routing_policy)
% member for structure vehicle: location, capacity, id
% od_set: cell array of od structure
% member for structure od: origin, destination, max_wait_time,
% max_delay_time, current_wait_time, current_delay_time, id
% input to function:
% 
tt_mat = routing_cost_mat{1};
fc_mat = routing_cost_mat{2};

veh_capacity = vehicle.capacity;
od_num = size(od_set,1);
% w_wait = 0.1;
% For debug, generate/load od set and routing map
% generate_data = 1;
% if generate_data == 1
%     od_set = cell(od_num,1);
%     link_uid = 0:4;
%     tt_mat = rand(length(link_uid),length(link_uid))*10;
%     tt_mat = tt_mat-diag(diag(tt_mat));
%     tt_mat = (tt_mat+tt_mat')/2;
%     for od_id = 1:od_num
%         od_set{od_id}.origin = randi([1,length(link_uid)-1]);
%         destination_set = link_uid(link_uid~=od_set{od_id}.origin&link_uid~=0);
%         od_set{od_id}.destination = destination_set(randi([1,length(destination_set)]));
%         od_set{od_id}.max_wait_time = 2;
%         od_set{od_id}.max_delay_time = 5;
%     end
% else
%     load('map_od_set4_2od_repeat_start_location.mat')
%     
%     link_uid = 0:size(tt_mat,1)-1;
% end
% state definition: [[indicator for visited origins][indicator for visited destinations][current location]]
% get feasible indicator state vectors:
% not violating vehicle capacity constraint
% visit origin before visit destination
state_bi_list = (0:2^(2*od_num)-1)';
state_vector = de2bi(state_bi_list);
state_vector_origin = state_vector(:,1:od_num);
state_vector_destination = state_vector(:,od_num+1:2*od_num);
state_vector_check = all(all(state_vector_origin >= state_vector_destination,2) ...
    & sum(state_vector_origin-state_vector_destination,2)<=veh_capacity,2);
state_vector = state_vector(state_vector_check,:);
% state_list_de = state_bi_list(state_vector_check);
% state_vector_origin = state_vector(:,1:od_num);
% state_vector_destination = state_vector(:,od_num+1:2*od_num);
% current location state:
% cannot at origin if already visited destination
% at vehicle location initially
% append to end of indicator state
state_aug = cell(2*od_num+1,1);
state_aug{1} = zeros(1,2*od_num+1);
location_list = [vehicle.location;cellfun(@(x) x.origin,od_set);cellfun(@(x) x.destination,od_set)];

for current_location_id = 1:2*od_num
    current_location_state = state_vector(state_vector(:,current_location_id)==1,:);
    if current_location_id<=od_num
        current_location_state = current_location_state(current_location_state(:,current_location_id+od_num)==0,:);
    end
    current_location = current_location_id*ones(size(current_location_state,1),1);
    state_aug{current_location_id+1} = [current_location_state,current_location];
end
state_aug_list = vertcat(state_aug{:});
state_aug_de = bi2de(state_aug_list(:,1:end-1));
location_aug = state_aug_list(:,end);
cost2go_constatint = cell(size(state_aug_list,1),1);
% state_time_constratint = cell(size(state_aug_list,1),1);
action_list = (1:od_num*2)';
action_vector = eye(2*od_num);
feasible_next_state_set = cell(size(state_aug_list,1),2);

for state_id = 1:size(state_aug_list,1)
    state = state_aug_list(state_id,1:end-1);
    current_location = state_aug_list(state_id,end);
%     state_origin = state(1:od_num);
%     state_destination = state(od_num+2:2*od_num);
    A_state = zeros(length(action_list),size(state_aug_list,1));
    b_state = zeros(length(action_list),1);
    feasible_id = zeros(length(action_list),1);
%     A_time_constraint = zeros(length(action_list),size(state_aug_list,1));
%     b_time_limit = zeros(length(action_list),1);
    feasible_next_state_id = zeros(length(action_list),2);
    for action_id = 1:length(action_list)
        next_location = action_id;
        action = action_vector(action_id,:);
        state_next = state + action;
        
        % check if next state is valid
        % TODO: fix for same location
        % transition cost: weighted sum of
        % total waiting time and total delay time
        if(all(state_next<=1))
            state_next_de = bi2de(state_next);
            next_state_id = find(state_aug_de==state_next_de & location_aug == next_location);
            if ~isempty(next_state_id)
                fuel_cost = fc_mat(link_uid==location_list(current_location+1),link_uid==location_list(next_location+1));
                time_cost = tt_mat(link_uid==location_list(current_location+1),link_uid==location_list(next_location+1));
                A_state(action_id,state_id) = 1;
                A_state(action_id,next_state_id) = -1;
                if routing_policy == 1
                    wait_cost = time_cost*(od_num - sum(state_next(1:od_num)));
                    delay_cost = time_cost*(od_num - sum(state_next(od_num+1:2*od_num)));
                    transition_cost = (1-w_wait)*delay_cost+w_wait*wait_cost;
                else
                    transition_cost = fuel_cost;
                end
                b_state(action_id) = transition_cost;
%                 A_time_constraint(action_id,1) = 1;
%                 A_time_constraint(action_id,next_state_id) = -1;
                feasible_next_state_id(action_id,1) = next_state_id;
                feasible_next_state_id(action_id,2) = transition_cost;
                feasible_id(action_id) = 1;
%                 if next_location <= od_num
%                     od_set_origin = cellfun(@(x) x.origin,od_set);
%                     od_set_select = od_set(od_set_origin==location_list(next_location+1));
%                     max_wait_time_set = cellfun(@(x) x.max_wait_time,od_set_select);
%                     b_time_limit(action_id) = min(max_wait_time_set);
%                 else
%                     min_travel_time = tt_mat(link_uid==location_list(next_location-od_num+1),link_uid==location_list(next_location+1));
%                     od_set_destination = cellfun(@(x) x.destination,od_set);
%                     od_set_origin = cellfun(@(x) x.origin,od_set);
%                     od_set_select = od_set(od_set_destination==location_list(next_location+1) & od_set_origin==location_list(next_location-od_num+1));
%                     max_delay_time_set = cellfun(@(x) x.max_delay_time,od_set_select);
%                     b_time_limit(action_id) = min_travel_time+min(max_delay_time_set);
%                 end
            end
        end
    end
    A_state = A_state(feasible_id>0,:);
%     b_time_limit = b_time_limit(b_state>0);
%     A_time_constraint = A_time_constraint(b_state>0,:);
    feasible_next_state_id = feasible_next_state_id(feasible_id>0,:);
    b_state = b_state(feasible_id>0);
    
    cost2go_constatint{state_id,1} = A_state;
    cost2go_constatint{state_id,2} = b_state;
    
%     state_time_constratint{state_id,1} = A_time_constraint;
%     state_time_constratint{state_id,2} = b_time_limit;
    
    feasible_next_state_set{state_id} = feasible_next_state_id;
%     state_id/size(state_aug_list,1)
end

cost2go_constatint_mat = cost2go_constatint(:,1);
cost2go_constatint_mat = vertcat(cost2go_constatint_mat{:});
cost2go_constatint_val = cost2go_constatint(:,2);
cost2go_constatint_val = vertcat(cost2go_constatint_val{:});
% state_time_constratint_mat = state_time_constratint(:,1);
% state_time_constratint_mat = vertcat(state_time_constratint_mat{:});
% state_time_constratint_val = state_time_constratint(:,2);
% state_time_constratint_val = vertcat(state_time_constratint_val{:});

state_sum = sum(state_aug_list(:,1:end-1),2);
terminal_index_list = find(state_sum==2*od_num);

terminal_cost_mat = zeros(length(terminal_index_list),size(state_aug_list,1));
terminal_cost_val = zeros(length(terminal_index_list),1);
for terminal_id = 1:length(terminal_index_list)
    terminal_cost_mat(terminal_id,terminal_index_list(terminal_id)) = 1;
end

f = ones(size(state_aug_list,1),1);
% tic
clear model;
model.A = sparse([cost2go_constatint_mat;
%     state_time_constratint_mat;
    terminal_cost_mat]);
model.rhs = [cost2go_constatint_val;
%     state_time_constratint_val;
    terminal_cost_val];
model.sense = [repmat('<', size(cost2go_constatint_mat,1), 1);
%     repmat('<', size(state_time_constratint_mat,1), 1);
    repmat('=', size(terminal_cost_mat,1), 1)];
model.obj = f;
model.modelsense = 'max';
params.outputflag = 0;
result = gurobi(model, params);
% toc
% recover trajectory
if isfield(result,'x')
    cost2go = result.x;
    cost2go_state = cost2go(1);
    
    current_state = 1;
    state_id_sequence = current_state;
    state_sequence = state_aug_list(current_state,:);
    location_sequence = location_list(1);
    cost_time_sequence = 0;
    cost_fuel_sequence = 0;
    while sum(state_aug_list(current_state,1:end-1))<2*od_num
        feasible_state = feasible_next_state_set{current_state};
%         feasible_state = feasible_state(feasible_state>0);
        feasible_state_cost = cost2go(feasible_state(:,1))+feasible_state(:,2);
        [cost2go_state,min_cost_id] = min(feasible_state_cost);
        next_state = feasible_state(min_cost_id);
        travel_time = tt_mat(link_uid==location_list(state_aug_list(current_state,end)+1),link_uid==location_list(state_aug_list(next_state,end)+1));
        fuel_consumption = fc_mat(link_uid==location_list(state_aug_list(current_state,end)+1),link_uid==location_list(state_aug_list(next_state,end)+1));
        current_state = next_state;
        state_id_sequence = [state_id_sequence;current_state];
        state_sequence = [state_sequence;state_aug_list(current_state,:)];
        location_sequence = [location_sequence;location_list(state_aug_list(current_state,end)+1)];
        cost_time_sequence = [cost_time_sequence;travel_time];
        cost_fuel_sequence = [cost_fuel_sequence;fuel_consumption];
    end
    onboard_num = sum(state_sequence(:,1:od_num),2) - sum(state_sequence(:,od_num+1:2*od_num),2);
    accu_cost = cumsum(cost_time_sequence);
%   check for wait time and delay constraint
    wait_time = zeros(1,od_num);
    delay_time = zeros(1,od_num);
    wait_time_status = zeros(1,od_num);
    delay_time_status = zeros(1,od_num);
    for od_id = 1:od_num
        origin_status = [0;diff(state_sequence(:,od_id))];
        destination_status = [0;diff(state_sequence(:,od_id+od_num))];
        wait_time(od_id) = accu_cost(origin_status==1);
        delay_time(od_id) = accu_cost(destination_status==1)-tt_mat(link_uid==od_set{od_id}.origin,link_uid==od_set{od_id}.destination);
        wait_time_status(od_id) = wait_time(od_id)>od_set{od_id}.max_wait_time;
        delay_time_status(od_id) = delay_time(od_id)>od_set{od_id}.max_delay_time;
    end
    trip_wait_time_status = any(wait_time_status);
    trip_delay_time_status = any(delay_time_status);
    result.onboard_num = onboard_num;
    result.wait_time_violation_status = 0;
    result.delay_time_violation_status = 0;
    result.wait_time = wait_time;
    result.delay_time = delay_time;
    result.accu_travel_time = accu_cost;
    result.travel_time_trajectory = cost_time_sequence;
    result.fuel_trajectory = cost_fuel_sequence;
    result.location_trajectory = location_sequence;
    result.feasible_soluition = 1;
    if trip_wait_time_status == 1
        result.wait_time_violation_status = 1;
    end
    if trip_delay_time_status == 1
        result.delay_time_violation_status = 1;
    end
%     Y = mdscale(tt_mat,2);
%     figure;hold on;
%     scatter(Y(1,1),Y(1,2),100,'filled','r')
%     c = linspace(1,10,od_num);
%     for i = 1:od_num
%         scatter(Y(link_uid==location_list(i+1),1),Y(link_uid==location_list(i+1),2),100,c(i),'d','filled')
%         scatter(Y(link_uid==location_list(i+1+od_num),1),Y(link_uid==location_list(i+1+od_num),2),100,c(i),'s','filled')
%     end
%     for link_id = 1:length(location_sequence)-1
%         plot([Y(link_uid==location_sequence(link_id),1),Y(link_uid==location_sequence(link_id+1),1)],...
%             [Y(link_uid==location_sequence(link_id),2),Y(link_uid==location_sequence(link_id+1),2)],'k')
% %         quiver(Y(link_uid==location_sequence(link_id),1),Y(link_uid==location_sequence(link_id),2),...
% %                Y(link_uid==location_sequence(link_id+1),1)-Y(link_uid==location_sequence(link_id),1),...
% %                Y(link_uid==location_sequence(link_id+1),2)-Y(link_uid==location_sequence(link_id),2),'k')
%     end
    
else
    result.wait_time_violation_status = 1;
    result.delay_time_violation_status = 1;
    result.accu_travel_time = [];
    result.onboard_num = [];
    result.travel_time_trajectory = [];
    result.fuel_trajectory = [];
    result.location_trajectory = vehicle.location;
    result.feasible_soluition = 0;
end
