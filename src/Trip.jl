mutable struct Trip
    vehicle_id :: Int
    customer_id_list
    wait_time_list
    delay_time_list
    customer_location_sequence
    customer_onboard_sequence
    customer_location_time
    link_sequence
    vehicle_trajectory
    departure_time :: Float64
    total_wait_time :: Float64
    total_delay_time :: Float64
    violate_wait_time :: Float64
    violate_delay_time :: Float64
    total_fuel :: Float64
    routing_policy
end 
