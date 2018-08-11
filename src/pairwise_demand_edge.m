function can_combine = pairwise_demand_edge(od1,od2,tt_matrix,link_uid)
location_sequence = [
    od1.origin,od1.destination,od2.origin,od2.destination;
    od1.origin,od2.origin,od1.destination,od2.destination;
    od1.origin,od2.origin,od2.destination,od1.destination;
    od2.origin,od2.destination,od1.origin,od1.destination;
    od2.origin,od1.origin,od2.destination,od1.destination;
    od2.origin,od1.origin,od1.destination,od2.destination;    
];
trip_travel_time = zeros(6,3);
for trip_id = 1:6
    trip = location_sequence(trip_id,:);
    trip_travel_time(trip_id,:) = [
        tt_matrix(link_uid==trip(1),link_uid==trip(2)), ...
        tt_matrix(link_uid==trip(2),link_uid==trip(3)), ...
        tt_matrix(link_uid==trip(3),link_uid==trip(4)) ...
    ];
end
tt_1 = tt_matrix(link_uid==od1.origin,link_uid==od1.destination);
tt_2 = tt_matrix(link_uid==od2.origin,link_uid==od2.destination);

wait_time_1 = [0,0,0,trip_travel_time(4,1)+trip_travel_time(4,2),trip_travel_time(5,1),trip_travel_time(6,1)];
wait_time_2 = [trip_travel_time(1,1)+trip_travel_time(1,2),trip_travel_time(2,1),trip_travel_time(3,1),0,0,0];
total_time_1 = [trip_travel_time(1,1),...
    trip_travel_time(2,1)+trip_travel_time(2,2),...
    trip_travel_time(3,1)+trip_travel_time(3,2)+trip_travel_time(3,3),...
    trip_travel_time(4,1)+trip_travel_time(4,2)+trip_travel_time(4,3),...
    trip_travel_time(5,1)+trip_travel_time(5,2)+trip_travel_time(5,3),...
    trip_travel_time(6,1)+trip_travel_time(6,2)...
    ];
total_time_2 = [...
    trip_travel_time(1,1)+trip_travel_time(1,2)+trip_travel_time(1,3),...
    trip_travel_time(2,1)+trip_travel_time(2,2)+trip_travel_time(2,3),...
    trip_travel_time(3,1)+trip_travel_time(3,2),...
    trip_travel_time(4,1),...
    trip_travel_time(5,1)+trip_travel_time(5,2),...
    trip_travel_time(6,1)+trip_travel_time(6,2)+trip_travel_time(6,3)...
    ];
delay_time_1 = total_time_1 - tt_1;
delay_time_2 = total_time_2 - tt_2;

wait_time_1_constraint = wait_time_1<=od1.max_wait_time;
wait_time_2_constraint = wait_time_2<=od2.max_wait_time;
delay_1_constraint = delay_time_1<=od1.max_delay_time;
delay_2_constraint = delay_time_2<=od2.max_delay_time;

constraint = wait_time_1_constraint&wait_time_2_constraint&delay_1_constraint&delay_2_constraint;
can_combine = any(constraint);