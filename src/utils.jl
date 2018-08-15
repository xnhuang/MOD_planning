using PyCall
@pyimport os
unshift!(PyVector(pyimport("sys")["path"]), os.path[:join](SUMO_HOME, "tools"))
@pyimport sumolib
@pyimport numpy as np
ANNARBOR = sumolib.net[:readNet]("/media/mSATA/UM/Simulation/f_AnnArbor1.6.net.xml")
EDGES = ANNARBOR[:getEdges]()
NODES = ANNARBOR[:getNodes]()
NETEDGES = Dict{String,Array{Int}}()
NETEDGESLOOKUP = Dict{String, Any}()
NETNODES = Dict{String,Int}()
NETNODESLOOKUP = Dict{String, Any}()
for e in EDGES
    NETEDGES[e[:getID]()]  = [-1, -1]
end
for e in EDGES
    NETEDGESLOOKUP[e[:getID]()]  = e
end

for v in NODES
    NETNODES[v[:getID]()]  = -1
end
for v in NODES
    NETNODESLOOKUP[v[:getID]()] = v
end

type Zone
    graph::Any
    sln2graph::Dict{String, Int}
    graph2sln::Dict{Int, String}
    froms::Array{Int, 1}
    tos::Array{Int, 1}
    times::Array{Float64, 1}
    socs::Array{Float64, 1}
    Ecosts::Array{Float64, 1}
end
struct SumoMap
    edges
    nodes
end
type Status
 start::String
 dest::String
 step::Int
 edge::String
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
function load_sumo_map(zonelidxdict, zonenidxdict)
    E = Dict()
    N = Dict()
    for zid in keys(zonelidxdict)
        E[zid] = []
        N[zid] = []
        for lidx in zonelidxdict[zid]
            ln = data_links[:link][lidx]
            if string(ln, "-0") in keys(NETEDGES)
                NETEDGES[string(ln, "-0")]=[zid, ln]
                push!(E[zid], NETEDGESLOOKUP[string(ln, "-0")])
            end
            if string(ln, "-1") in keys(NETEDGES)
                NETEDGES[string(ln, "-1")]=[zid, ln]
                push!(E[zid], NETEDGESLOOKUP[string(ln, "-1")])
            end
        end
        for nidx in zonenidxdict[zid]
            nn = data_nodes[:node][nidx]
            NETNODES[String(nn)] = zid
            push!(N[zid], NETNODESLOOKUP[string(nn)])
        end
    end
    return SumoMap(E, N)
end
function read_zonefile(filename)
    zonelidxdict = Dict{Int, Set{Int}}()
    zonenidxdict = Dict{Int, Set{Int}}()
    zonecnidxdict = Dict{Int, Dict{Int, Set{Int}}}()
    zonecnidxlist = Dict{Int, Set{Int}}()
    open(filename) do f
        thiszone = nothing
        for line in eachline(f)
            res = split(line)
            if res[1] == "Zone"
                thiszone = parse(Int, res[2])
                zonelidxdict[thiszone] = Set{Int}()
                zonenidxdict[thiszone] = Set{Int}()
                zonecnidxdict[thiszone] = Dict{Int, Set{Int}}()
                zonecnidxlist[thiszone] = Set{Int}()
            elseif res[1] == "Links"
                for lidx in res[2:end]
                    push!(zonelidxdict[thiszone], parse(Int, lidx))
                end
            elseif res[1] == "Nodes"
                for nidx in res[2:end]
                    push!(zonenidxdict[thiszone], parse(Int, nidx))
                end
            elseif res[1] == "Neigh"
                nz = parse(Int, res[2])
                zonecnidxdict[thiszone][nz] = Set{Int}()
                for nidx in res[3:end]
                    push!(zonecnidxdict[thiszone][nz], parse(Int, nidx))
                    push!(zonecnidxlist[thiszone], parse(Int, nidx))
                end
            end
        end
    end
    return [zonelidxdict, zonenidxdict, zonecnidxdict, zonecnidxlist]
end
function read_traffic1(filename)
    hevs = Dict{Int, Array{Float64}}()
    evs = Dict{Int, Array{Float64}}()
    lns = Dict{Int, Array{Int}}()
    edges = Int[]
    speeds = Dict{Int, Array{Float64}}()
    open(filename) do f
        record = -1
        step = nothing
        l = nothing
        for line in eachline(f)
            res = split(line)
            if res[1] == "Bar6"
                record = 1
            elseif res[1] == "Bar4_1"
                record = 2
            elseif res[1] == "Bar4"
                record = 3
            elseif contains(res[1], "Bar")
                record = 4
            elseif res[1] == "Step" && record == 1
                step = parse(Int, res[2])
                l = 1
                hevs[step] = Float64[]
                evs[step] = Float64[]
                lns[step] = Int[]
            elseif res[1] == "Step" && record == 3
                step = parse(Int, res[2])
                speeds[step] = Float64[]
            elseif record == 1
                if l == 1
                    l = 2
                    for hev in res
                        push!(hevs[step], parse(Float64, hev))
                    end
                elseif l == 2
                    l = 3
                    for ev in res
                        push!(evs[step], parse(Float64, ev))
                    end
                else
                    for ln in res
                        push!(lns[step], parse(Int, ln))
                    end
                end
            elseif record == 2
                for ln in res
                    push!(edges, parse(Int, ln))
                end
            elseif record == 3
                for spd in res
                    push!(speeds[step], parse(Float64, spd))
                end
            end
        end
    end
    return (hevs, evs, lns, speeds, edges)
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
function batchpush!(lists, items)
    for (idx, list) in enumerate(lists)
        push!(list, items[idx])
    end
end
