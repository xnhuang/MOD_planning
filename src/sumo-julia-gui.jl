NUMSTUDY = 200
include("constants.jl")
include("rendering.jl")
using PyCall
unshift!(PyVector(pyimport("sys")["path"]), os.path[:join](SUMO_HOME, "tools"))
unshift!(PyVector(pyimport("sys")["path"]), "/media/mSATA/UM/Simulation")
@pyimport mvn_gmr
@pyimport sys
@pyimport os
@pyimport traci
@pyimport traci.constants as tc
@pyimport sumolib
@pyimport numpy as np
screen_width = 1000
screen_height = 1000
side_screen_width, side_screen_height = 800, 1000
world_width = XBOUNDRIGHT-XBOUNDLEFT
world_widthx = XBOUNDRIGHT-XBOUNDLEFT
scalex = screen_width/world_widthx
world_widthy = YBOUNDTOP-YBOUNDBOT
scaley = screen_width/world_widthy

hevgmm = mvn_gmr.GMM(np.array(GMMHEVMEANS), np.array(GMMHEVCOVS), np.array(GMMHEVWEIGHTS))
evgmm = mvn_gmr.GMM(np.array(GMMEVMEANS), np.array(GMMEVCOVS), np.array(GMMEVWEIGHTS))
hevgmr = mvn_gmr.GMR(hevgmm)
evgmr = mvn_gmr.GMR(evgmm)
type Status
 start::String
 dest::String
 step::Array{Int,1}
 edge::Array{String,1}
 pos::Array{Tuple{Float64,Float64},1}
 speed::Array{Float64,1}
 arrived::Bool
end
type BackStatus
 step::Int
 edge::Array{String, 1}
 speed::Array{Float64, 1}
 starttime::Array{Int, 1}
 endtime::Array{Int, 1}
 startspeed::Array{Float64, 1}
 endspeed::Array{Float64, 1}
 meanspeed::Array{Float64, 1}
 stdspeed::Array{Float64, 1}
 arrived::Bool
end
type EdgeStatus
 step::Array{Int, 1}
 traveltime::Array{Float64, 1}
 travelspeed::Array{Float64, 1}
 spdlimit::Float64
end
type CarHandler
    xs::Array{Float64, 1}
    ys::Array{Float64, 1}
    cidxs::Array{Int, 1}
    isshowns::Array{Bool, 1}
end
function compute_energy(meanspd, meangrade, stdspd, initsoc, spdlimit, dynamic)
    meanspd2 = [min(max(x, HEVMINSPD) , HEVMAXSPD) for x in meanspd]
    meangrade2 = [min(max(x, HEVMINGRADE) , HEVMAXGRADE) for x in meangrade]
    stdspd2 = [min(max(x, HEVMINSSPD) , HEVMAXSSPD) for x in stdspd]
    initsoc2 = [min(max(x, HEVMININSOC) , HEVMAXINSOC) for x in initsoc]
    spdlimit2 = [min(max(x, HEVMINSPDLM) , HEVMAXSPDLM) for x in spdlimit]
    dynamic2 = [min(max(x, HEVMINDE) , HEVMAXDE) for x in dynamic]
    meanspd2 = (meanspd2 - HEVAVGSPD)/HEVSTDSPD
    meangrade2 = (meangrade2 - HEVAVGGRADE)/HEVSTDGRADE
    stdspd2 = (stdspd2 - HEVAVGSSPD)/HEVSTDSSPD
    initsoc2 = (initsoc2 - HEVAVGINSOC)/HEVSTDINSOC
    spdlimit2 = (spdlimit2 - HEVAVGSPDLM)/HEVSTDSPDLM
    dynamic2 = (dynamic2 - HEVAVGDE)/HEVSTDDE
    XX = (np.array([meanspd2,meangrade2,stdspd2,initsoc2,spdlimit2,dynamic2]))'
    hevre = hevgmr[:predict](np.array([0,1,2,3,4,5]), XX)
    hevre = hevre*HEVSTDRE + HEVAVGRE

    meanspd2 = [min(max(x, EVMINSPD) , EVMAXSPD) for x in meanspd]
    meangrade2 = [min(max(x, EVMINGRADE) , EVMAXGRADE) for x in meangrade]
    stdspd2 = [min(max(x, EVMINSSPD) , EVMAXSSPD) for x in stdspd]
    initsoc2 = [min(max(x, EVMININSOC) , EVMAXINSOC) for x in initsoc]
    spdlimit2 = [min(max(x, EVMINSPDLM) , EVMAXSPDLM) for x in spdlimit]
    dynamic2 = [min(max(x, EVMINDE) , EVMAXDE) for x in dynamic]
    meanspd2 = (meanspd2 - EVAVGSPD)/EVSTDSPD
    meangrade2 = (meangrade2 - EVAVGGRADE)/EVSTDGRADE
    stdspd2 = (stdspd2 - EVAVGSSPD)/EVSTDSSPD
    initsoc2 = (initsoc2 - EVAVGINSOC)/EVSTDINSOC
    spdlimit2 = (spdlimit2 - EVAVGSPDLM)/EVSTDSPDLM
    dynamic2 = (dynamic2 - EVAVGDE)/EVSTDDE
    XX = (np.array([meanspd2,meangrade2,stdspd2,initsoc2,spdlimit2,dynamic2]))'
    evre = evgmr[:predict](np.array([0,1,2,3,4,5]), XX)
    evre = evre*EVSTDRE + EVAVGRE
    #println(hevre)
    #println(evre)
    return (hevre, evre)
end
function update_geom(handler::CarHandler, id::Int, x::Float64, y::Float64, cidx::Int)
    if id > length(handler.xs)
        push!(handler.xs, x*scalex-5)
        push!(handler.ys, y*scaley+45)
        push!(handler.cidxs, cidx)
        push!(handler.isshowns, true)
    else
        handler.xs[id] = x*scalex-5
        handler.ys[id] = y*scaley+45
        handler.cidxs[id] = cidx
    end
end
vehicleStatus = Dict()
vehicleVisual = Dict()
backgroundStatus = Dict()
netStatus = Dict()

ANNARBOR = sumolib.net[:readNet]("/media/mSATA/UM/Simulation/f_AnnArbor1.6.net.xml")
EDGES = ANNARBOR[:getEdges]()
NET = Dict{String,Array{Int}}()
for e in EDGES
    NET[e[:getID]()]  = [-1, -1]
end
NETLOOKUP = Dict()
for e in EDGES
    NETLOOKUP[e[:getID]()]  = e
end
StudiedEdges = []
for e in EDGES
    if e[:getPriority]() >= 6
        push!(StudiedEdges, e[:getID]())
    end
end
link_grades = Dict()
for (idx,ln) in enumerate(data_grades[:link])
    link_grades[ln] = idx
end
NETGRADE = Dict()
NETDIST = Dict()
open("/media/mSATA/UM/Simulation/finalclusters.txt") do f
    this_zone = -1
    for line in eachline(f)
        res = split(line)
        if res[1] == "Zone"
            this_zone = parse(Int, res[2])
        elseif res[1] == "Links"
            for lidx in res[2:end]
                ln = data_links[:link][parse(Int, lidx)]
                if string(ln, "-0") in keys(NET)
                    NET[string(ln, "-0")] = [this_zone, ln]
                    NETDIST[string(ln, "-0")] = NETLOOKUP[string(ln, "-0")][:getLength]()
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-0")] = data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-0")] = mean(data_grades[:meangrade])
                    end
                end
                if string(ln, "-1") in keys(NET)
                    NET[string(ln, "-1")] = [this_zone, ln]
                    NETDIST[string(ln, "-1")] = NETLOOKUP[string(ln, "-1")][:getLength]()
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-1")] = data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-1")] = mean(data_grades[:meangrade])
                    end
                end
                if string(ln, "-1-Pocket") in keys(NET)
                    NET[string(ln, "-1-Pocket")] = [this_zone, ln]
                    NETDIST[string(ln, "-1-Pocket")] = NETLOOKUP[string(ln, "-1-Pocket")][:getLength]()
                    NETDIST[string(ln, "-1")] += NETDIST[string(ln, "-1-Pocket")]
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-1-Pocket")] = data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-1-Pocket")] = mean(data_grades[:meangrade])
                    end
                end
                if string(ln, "-0-Pocket") in keys(NET)
                    NET[string(ln, "-0-Pocket")] = [this_zone, ln]
                    NETDIST[string(ln, "-0-Pocket")] = NETLOOKUP[string(ln, "-0-Pocket")][:getLength]()
                    NETDIST[string(ln, "-0")] += NETDIST[string(ln, "-0-Pocket")]
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-0-Pocket")] = data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-0-Pocket")] = mean(data_grades[:meangrade])
                    end
                end
            end
        end
    end
end
ZoneEgeCount = zeros(14)
for ln in keys(NET)
    e = NETLOOKUP[ln]
    if e[:getPriority]() >= 6
        ZoneEgeCount[NET[ln][1]] += 1
    end
end

function _checkBackInitialPositions(vehicleID, start, dest, edge, pos, time, speed)
 if !(vehicleID in keys(backgroundStatus))
   backgroundStatus[vehicleID] = BackStatus(Int(time/1000), [edge], [speed], [Int(time/1000)], Int[], [speed], Float64[], Float64[], Float64[], dest==edge)
 end
end
function _checkInitialPositions(vehicleID, start, dest, edge, pos, time, speed, viewer, cars, carhandler)
 if !(vehicleID in keys(vehicleStatus))
  # a new vehicle not in the analysis list
  if length(vehicleStatus) < NUMSTUDY && start == edge
   # make sure this new vehicle is the first 200 cars and at the start of its route
   vehicleStatus[vehicleID] = Status(start, dest, [Int(time/1000)], [edge], [pos], [speed], dest==edge)
   #vehicleVisual[vehicleID] = Image("/home/boqi/Pictures/noun_Car_982516.png", 25, 25, pos[1]*scale-15, pos[2]*scale-15)
   #viewer[:add_geom](vehicleVisual[vehicleID])
   vehicleVisual[vehicleID] = length(vehicleVisual) + 1
   update_geom(carhandler, vehicleVisual[vehicleID], pos[1], pos[2], 1)
   #cars[:update_geom](vehicleVisual[vehicleID], pos[1]*scale-15, pos[2]*scale-15, 1)
  end
 end
end

function doStep(viewer, cars, carhandler, datas)
    traci.simulationStep()
    #moveNodes = []
    subscribes = traci.vehicle[:getSubscriptionResults]()
    time = traci.simulation[:getCurrentTime]()

    #count = 0
    if time%(30*1000) == 0
        datas["bar2"] = zeros(Int, 14)
        datas["bar3"] = zeros(14)
        datas["bar3count"] = ones(14)*0.000001
        for (veh, subs) in subscribes
            edge = traci.vehicle[:getRoadID](veh)
            speed = traci.vehicle[:getSpeed](veh)
            try
                datas["bar2"][NET[edge][1]] += 1
                datas["bar3"][NET[edge][1]] += speed
                datas["bar3count"][NET[edge][1]] += 1

            if backgroundStatus[veh].edge[end] != edge && !(contains(backgroundStatus[veh].edge[end], edge)||contains(edge, backgroundStatus[veh].edge[end]))
                push!(backgroundStatus[veh].edge, edge)
                push!(backgroundStatus[veh].endtime, backgroundStatus[veh].step)
                push!(backgroundStatus[veh].starttime, Int(time/1000))
                push!(backgroundStatus[veh].endspeed, backgroundStatus[veh].speed[end])
                push!(backgroundStatus[veh].startspeed, speed)
                push!(backgroundStatus[veh].meanspeed, mean(backgroundStatus[veh].speed))
                push!(backgroundStatus[veh].stdspeed, isnan(std(backgroundStatus[veh].speed))?0.0:std(backgroundStatus[veh].speed))
                backgroundStatus[veh].speed = Float64[]
            end
            backgroundStatus[veh].step = Int(time/1000)
            push!(backgroundStatus[veh].speed, speed)
            end
        end
        cost = Dict()
        for i=1:14
            cost[i] = Dict("meanspd"=>[],"meangrade"=>[],"stdspd"=>[],"initsoc"=>[],"spdlim"=>[],"dynamic"=>[])
        end
        for vid in keys(backgroundStatus)
            for (idx, e) in enumerate(backgroundStatus[vid].edge)
                if idx <= length(backgroundStatus[vid].endtime)
                    if e in keys(NET)
                        push!(cost[NET[e][1]]["meanspd"], backgroundStatus[vid].meanspeed[idx])
                        push!(cost[NET[e][1]]["meangrade"], NETGRADE[e])
                        push!(cost[NET[e][1]]["stdspd"], backgroundStatus[vid].stdspeed[idx])
                        push!(cost[NET[e][1]]["initsoc"], 0.7)
                        push!(cost[NET[e][1]]["spdlim"], NETLOOKUP[e][:getSpeed]())
                        push!(cost[NET[e][1]]["dynamic"], ((backgroundStatus[vid].endspeed[idx])^2-(backgroundStatus[vid].startspeed[idx])^2)/NETDIST[e])
                    end
                end
            end
            if backgroundStatus[vid].arrived
                delete!(backgroundStatus, vid)
            else
                backgroundStatus[vid].edge = [backgroundStatus[vid].edge[end]]
                backgroundStatus[vid].endtime = []
                backgroundStatus[vid].starttime = [backgroundStatus[vid].starttime[end]]
                backgroundStatus[vid].endspeed = []
                backgroundStatus[vid].startspeed = [backgroundStatus[vid].startspeed[end]]
                backgroundStatus[vid].meanspeed = []
                backgroundStatus[vid].stdspeed = []
            end
        end
        for i =1:14
            if length(cost[i]["meanspd"]) > 0
                costresult = compute_energy(cost[i]["meanspd"],
                                                       cost[i]["meangrade"],
                                                       cost[i]["stdspd"],
                                                       cost[i]["initsoc"],
                                                       cost[i]["spdlim"],
                                                       cost[i]["dynamic"])
                datas["bar4"][i][1] = mean(costresult[1])*1000
                datas["bar4"][i][2] = mean(costresult[2])*1000
            end
        end
        #push!(datas["archive"]["bar1"], deepcopy(datas["bar1"]))
        #push!(datas["archive"]["bar1_1"], datas["bar1_1"])
        #push!(datas["archive"]["bar2"], datas["bar2"])
        #push!(datas["archive"]["bar3"], datas["bar3"]./datas["bar3count"])
        datas["bar1_1"] = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
    #=
    else
        for (veh, subs) in subscribes
            edge = traci.vehicle[:getRoadID](veh)
            speed = traci.vehicle[:getSpeed](veh)
            if backgroundStatus[veh].edge[end] != edge && !(contains(backgroundStatus[veh].edge[end], edge)||contains(edge, backgroundStatus[veh].edge[end]))
             push!(backgroundStatus[veh].edge, edge)
             push!(backgroundStatus[veh].endtime, backgroundStatus[veh].step)
             push!(backgroundStatus[veh].starttime, Int(time/1000))
             push!(backgroundStatus[veh].endspeed, backgroundStatus[veh].speed[end])
             push!(backgroundStatus[veh].startspeed, speed)
             push!(backgroundStatus[veh].meanspeed, mean(backgroundStatus[veh].speed))
             push!(backgroundStatus[veh].stdspeed, isnan(std(backgroundStatus[veh].speed))?0.0:std(backgroundStatus[veh].speed))
             backgroundStatus[veh].speed = Float64[]
            end
            backgroundStatus[veh].step = Int(time/1000)
            push!(backgroundStatus[veh].speed, speed)
        end
    =#
    end
    departed = traci.simulation[:getSubscriptionResults]()[tc.VAR_DEPARTED_VEHICLES_IDS]
    arrived = traci.simulation[:getSubscriptionResults]()[tc.VAR_ARRIVED_VEHICLES_IDS]
    for v in departed
        traci.vehicle[:subscribe](v)
        #subs = traci.vehicle[:getSubscriptionResults](v)
        route = traci.vehicle[:getRoute](v)
        speed = traci.vehicle[:getSpeed](v)
        pos = traci.vehicle[:getPosition](v)
        start = route[1]
        dest = route[end]
        edge = traci.vehicle[:getRoadID](v)
        #push!(moveNodes, (v, route[1], route[end], subs[tc.VAR_ROAD_ID], pos, speed))
        _checkBackInitialPositions(v, start, dest, edge, pos, time, speed)
        _checkInitialPositions(v, start, dest, edge, pos, time, speed, viewer, cars, carhandler)
        try
            datas["bar1"][NET[start][1]][1] += 1
            datas["bar1"][NET[dest][1]][2] += 1
            # origin production
            datas["bar1_1"][NET[start][1]][1] += 1
        end
    end
    for v in arrived
        try
            datas["bar1"][NET[backgroundStatus[v].start][1]][1] -= 1
            datas["bar1"][NET[backgroundStatus[v].dest][1]][2] -= 1
            datas["bar1_1"][NET[backgroundStatus[v].dest][1]][2] += 1
        end
        if backgroundStatus[v].arrived==false
         backgroundStatus[v].arrived = true
         push!(backgroundStatus[v].endtime, backgroundStatus[v].step)
         push!(backgroundStatus[v].endspeed, backgroundStatus[v].speed[end])
         push!(backgroundStatus[v].meanspeed, mean(backgroundStatus[v].speed))
         push!(backgroundStatus[v].stdspeed, std(backgroundStatus[v].speed))
        end
    end
    for vehicleID in keys(vehicleStatus)
        if vehicleID in arrived && vehicleStatus[vehicleID].arrived==false
            vehicleStatus[vehicleID].arrived = true
            carhandler.isshowns[vehicleVisual[vehicleID]] = false
        end
        if !vehicleStatus[vehicleID].arrived
            #vehicles still running
            speed = traci.vehicle[:getSpeed](vehicleID)
            pos = traci.vehicle[:getPosition](vehicleID)
            #edge = traci.vehicle[:getRoadID](vehicleID)
            #push!(vehicleStatus[vehicleID].edge, edge)
            x = pos[1]*scalex-5
            y = pos[2]*scaley+45
            if (carhandler.xs[vehicleVisual[vehicleID]]-x)^2 + (carhandler.ys[vehicleVisual[vehicleID]]-y)^2 > 25
                #cars[:update_geom](vehicleVisual[vehicleID], x, y, 1)
                update_geom(carhandler, vehicleVisual[vehicleID], pos[1], pos[2], 1)
                #cars[:handler] = carhandler
                #vehicleVisual[vehicleID][:x] = x
                #vehicleVisual[vehicleID][:y] = y
            end
        end
    end
end
function main(T)
 sumoExe = SUMO
 sumoConfig = SUMO_CONFIG
 traci.start([sumoExe, "-c", sumoConfig])
 traci.simulation[:subscribe](varIDs=(tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS))
 viewer = Viewer(screen_width, screen_height)
 sideviewer = Viewer(side_screen_width,side_screen_height, 1000, 0)
 ymargin = 40
 xmargin = 20
 plotwidth = 760
 plotheight = 200
 datas = Dict{String, Any}()
 datas["bar1"] = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
 datas["bar1_1"] = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
 datas["bar2"] = zeros(Int, 14)
 datas["bar3"] = zeros(14)
 datas["bar4"] = [[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]]
 datas["bar3count"] = ones(14)*0.000001
 datas["archive"] = Dict()
 datas["archive"]["bar1"] = []
 datas["archive"]["bar1_1"] = []
 datas["archive"]["bar2"] = []
 datas["archive"]["bar3"] = []
 bar1 = BarPlot(plotwidth,plotheight,xmargin,ymargin)
 bar1[:set_axis](["Zone ID", "Loc Count"])
 bar1[:add_data](datas["bar1"], color = COLORPOOL, renew=true)
 bar2 = BarPlot(plotwidth,plotheight,xmargin,2*ymargin+plotheight)
 bar2[:set_axis](["Zone ID", "Veh Count"])
 bar2[:add_data](datas["bar2"], color = COLORPOOL, renew=true)
 bar3 = BarPlot(plotwidth,plotheight,xmargin,3*ymargin+2*plotheight)
 bar3[:add_data](datas["bar3"]./datas["bar3count"], color = COLORPOOL, renew=true)
 bar3[:set_axis](["Zone ID", "Travel Speed"])
 bar4 = BarPlot(plotwidth,plotheight,xmargin,4*ymargin+3*plotheight)
 bar4[:add_data](datas["bar4"], color = COLORPOOL, renew=true)
 bar4[:set_axis](["Zone ID", "Energy Cost Rate"])

 sideviewer[:add_geom](bar1)
 sideviewer[:add_geom](bar2)
 sideviewer[:add_geom](bar3)
 sideviewer[:add_geom](bar4)
 background = Image(BACKGROUND_FILE, screen_width, screen_height)
 viewer[:add_geom](background)

 carhandler = CarHandler(Float64[], Float64[], Int[], Bool[])
 cars = ImageCompound(CARICONS, 20, 20, carhandler)

 viewer[:add_geom](cars)
 #=
 for e in EDGES
  netStatus[e[:getID]()] = EdgeStatus(Int[],Float64[],Float64[],e[:getSpeed]())
 end
 =#
 #traci.vehicle[:subscribe]("1", (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
 t = 0
 while t<T
  #println(t)
  doStep(viewer, cars, carhandler, datas)
  viewer[:render]()
  bar1[:add_data](datas["bar1"])
  bar2[:add_data](datas["bar2"])
  bar3[:add_data](datas["bar3"]./datas["bar3count"])
  bar4[:add_data](datas["bar4"])
  sideviewer[:render]()
  t=t+1
  #if t%60==0
  #   datas["bar1_1"] = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
  #end
  #=
  if length(vehicleStatus) == NUMSTUDY
   simulationend = true
   for vehID in keys(vehicleStatus)
    simulationend = simulationend && vehicleStatus[vehID].arrived
   end
   if simulationend
    break
   end
  end
  =#
 end
 #=
 open("/media/mSATA/UM/Upper routing simulation/SUMOdata/f_veh.txt", "w") do f
     write(f, "Bar1\n")
     for (idx, d) in enumerate(datas["archive"]["bar1"])
         write(f, "Step $idx\n")
         for e in d
             write(f, "$(e[1]) $(e[2])\n")
         end
     end
     write(f, "Bar1_1\n")
     for (idx, d) in enumerate(datas["archive"]["bar1_1"])
         write(f, "Step $idx\n")
         for e in d
             write(f, "$(e[1]) $(e[2])\n")
         end
     end
     write(f, "Bar2\n")
     for (idx, d) in enumerate(datas["archive"]["bar2"])
         write(f, "Step $idx\n")
         for (eidx, e) in enumerate(d)
             if eidx < length(d)
                 write(f, "$e ")
             else
                 write(f, "$e\n")
             end
         end
     end
     write(f, "Bar3\n")
     for (idx, d) in enumerate(datas["archive"]["bar3"])
         write(f, "Step $idx\n")
         for (eidx, e) in enumerate(d)
             if eidx < length(d)
                 write(f, "$e ")
             else
                 write(f, "$e\n")
             end
         end
     end
 end
 =#
 traci.close()
 viewer[:close]()
 sideviewer[:close]()
end
