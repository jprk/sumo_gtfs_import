import os
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import unidecode
from datetime import datetime, timedelta
from matplotlib.category import UnitData
from collections import OrderedDict, defaultdict
from sortedcontainers import SortedDict

import sumolib

do_graph = True
LINE_LIST = ['11']


class SUMOPTStop:
    """
    Data about a public transport stop in SUMO.
    """

    def __init__(self, name, stop_id):
        self.name = name
        self.id = stop_id


class SUMOBasePTRouteWithStops:
    """
    Holds data for a single base route serviced by a PT vehicle.
    Base routes are routes that are virtual, they always start at midnight and are supposed to be
    converted into real routes by applying a concrete offset using `offset_by()` method.
    """

    def __init__(self, stops, route_id):
        """
        Instantiate a new PT vehicle route.
        The element contains time-mapped stops visited during this run and its SUMO route ID.
        :param stops: Dictionary of stops indexed by arrival time
        :param route_id: String ID of SUMO route that corresponds to this run
        """
        skeys = sorted(stops.keys())
        midnight_datetime = datetime(1970, 1, 1)
        self.route_stops = OrderedDict()
        for t in skeys:
            # fromtimestamp(0) returns 1970-01-01 01:00:00, but we need midnight
            dt = midnight_datetime + timedelta(seconds=t)
            self.route_stops[dt] = stops[t]
        t0 = skeys[0]
        self.time_from = midnight_datetime + timedelta(seconds=t0)
        self.stop_from = stops[t0]
        t1 = skeys[-1]
        self.time_to = midnight_datetime + timedelta(seconds=t1)
        self.stop_to = stops[t1]
        self.sumo_route_id = route_id

    def offset_by(self, time_offset):
        """
        Return a copy of the current basic route with stops where the stop times are shifted in time.
        This will create a `SUMOConcretePTRouteWithStops` instance.
        :param time_offset: Time offset in seconds to add to the master stop times
        :return: A route with stops moved in time.
        :rtype: SUMOConcretePTRouteWithStops
        """
        td = timedelta(seconds=time_offset)
        stops = OrderedDict((stop_time + td, self.route_stops[stop_time]) for stop_time in self.route_stops)
        return SUMOConcretePTRouteWithStops(stops, self.sumo_route_id)


class SUMOConcretePTRouteWithStops:
    """
    Holds data for a single base route serviced by a PT vehicle.
    Base routes are routes that are virtual, they always start at midnight and are supposed to be
    converted into real routes by applying a concrete offset using `offset_by()` method.
    """

    def __init__(self, stops, route_id):
        """
        Instantiate a new PT vehicle route.
        The element contains time-mapped stops visited during this run and its SUMO route ID.
        :param stops: Dictionary of stops indexed by arrival time
        :type stops: OrderedDict
        :param route_id: String ID of SUMO route that corresponds to this run
        :type route_id: str
        """
        self.route_stops = stops
        # Calling keys() returns a view, which can be turned into an iterator, but getting the last element of
        # an iterator may be tricky (cf. *_, last = iterator). So we will stick with a list.
        skeys = list(stops.keys())
        t0 = skeys[0]
        self.time_from = t0
        self.stop_from = stops[t0]
        t1 = skeys[-1]
        self.time_to = t1
        self.stop_to = stops[t1]
        self.sumo_route_id = route_id


class CirculationElement:
    pass


class CirculationRunElement(CirculationElement):
    """
    Holds data for a single run in a vehicle circulation.
    """

    def __init__(self, line_no, route_with_stops, time_offset):
        """
        Create a single circulation run that is based on a time-offset SUMO route with stops.
        :param line_no: Line number
        :param route_with_stops: Basic SUMO PT vehicle route with its stops, originating at time=0
        :param time_offset: Time offset of this run in seconds since midnight
        """
        self.line_no = line_no
        self.run = route_with_stops.offset_by(time_offset)


class FinalStopElement(CirculationElement):
    pass


class FinalStopSequentialElement(FinalStopElement):
    """
    Final stop of a vehicle line (run) where vehicles are queueing at the
    platform, moving forward when the first vehicle leaves to service another
    leg of its planned route.
    """


class FinalStopMultiplatformElement(FinalStopElement):
    """
    Final stop of a vehicle line (run) where vehicles normally park at a parking lot
    waiting for their next turn. This cannot be (probably) represented as a SUMO
    parking area, and we represent it as a set of parallel one-way lanes, each with
    its own bus stop (and possibly also EV charger).
    """

    def __init__(self, element_name, base_id, comes_after, travel_time, element_edges):
        self.name = element_name
        self.base_id = base_id
        self.prev_name = comes_after
        self.travel_time = travel_time
        self.edge_list = element_edges
        self.id_list = [f'{self.base_id}_{i}' for i in range(len(element_edges))]


def complete_run(all_runs, name_from, min_time_from):
    runs_from_here = all_runs[name_from]
    next_runs = []
    for time_from, run_data in runs_from_here.items():
        if time_from > min_time_from:
            if run_data is not None:
                run, route_id = run_data
                id_to, name_to, time_to = run[-1]
                print(f'      at {time_from.time()} leaving to {name_to}, arrival {time_to.time()}')
                c = complete_run(rsbt, name_to, time_to)
                next_runs = [(time_from, run, [route_id])] + c
                break
            else:
                print(f'      at {time_from.time()} already deleted')
    return next_runs


def collect_runs_by_starting_points(all_runs):
    """
    Create dictionary of starting points of vehicle runs referring to runs themselves.
    :param all_runs:
    :type all_runs: list[CirculationRunElement]
    :return:
    """
    route_dict = defaultdict(dict)
    # Loop over all CirculationRunElements in `all_runs` and reorganise them by the "from" stops.
    for cre in all_runs:
        # A single run from the starting stop `sfrom` at `tfrom` to a final stop `sfinal` at `tfinal`.
        # This run has a GTFS id `run_id`, vehicle travels along SUMO route `route_id` and contains stops
        # listed in `stops`.
        # Every stop is represented as a tuple (id,name).
        # Times are represented as seconds since midnight
        if tfinal > 95000:
            # print('   skipped')
            continue
        print(f'GTFS run {run_id}:')
        # Stop sequence ordered by stop time
        stop_seq = []
        for scheduled_time in sorted(stops):
            stop_tuple = stops[scheduled_time]
            stop_id, stop_name = stop_tuple
            stop_datetime = datetime.fromtimestamp(scheduled_time)
            stop_seq.append((stop_id, stop_name, stop_datetime))
            print(f'-- {stop_name:20s}: {stop_datetime.time()}')
        # stop_list = stop_seq.keys()
        # time_list = stop_seq.values()
        # stop_list = []
        # time_list = []
        # for stop_name in graph_stop_list:
        #    stop_list.append(stop_name)
        #    if stop_name in stop_seq:
        #        time_list.append(stop_seq[stop_name])
        #    else:
        #        time_list.append(None)
        #    print(f'-- {stop_name:20s}: {time_list[-1]}')
        # graph_run_data = (stop_list, time_list)
        # graph_routes.append(graph_run_data)
        # ax.plot(time_list, stop_list, linewidth=0.5, yunits=line_y_units)
        # Get the name of the starting stop of this run
        starting_stop_name = cre.get_stop_from_name()  # sfrom[1]  # sfrom[0] would be stop id
        # Convert the starting timestamp to a datetime
        # TODO: This is probably the same as the timestamp of the first element in `stop_seq`
        datetime_from = datetime.fromtimestamp(tfrom)
        # Enter the run into the collection of runs that start at `starting_stop_name` as a run that starts at
        # `datetime_from` Note that the route_dict[starting_stop_name] is (a) not build starting from the smallest
        # timestamp, (b) implemented as a dictionary so the iteration over it is not guaranteed to occur in the same
        # order as was the order of insering the keys. We will make sure that routes are sorted by their departure
        # times by switching to OderedDict below.
        route_dict[starting_stop_name][datetime_from] = (stop_seq, [route_id])

    # Sort `route_dict` dictionary entries for every key and store them in that sorted order.
    # For every starting point (first-level key) we order the particular runs from that starting point by the
    # departure time and put them into an OderderDict instance.
    for key, val in route_dict.items():
        # Need to maintain the order of keys
        sorted_val = SortedDict()
        for k in val.keys():
            sorted_val[k] = val[k]
        # Replace the route_dict entry
        route_dict[key] = sorted_val

    return route_dict


def connect_runs(route_dict):
    # Loop over route_dict and try to connect single runs into vehicle circulations. 
    # This will modify the dict in place and therefore in general multiple runs are 
    # needed so that the whole thing converges to a minimal list of daily vehicle
    # circulations.
    #
    # Notation:
    # * a _run_ is a single vehicle drive from a starting stop A to the final stop B
    #   of some line, e.g. run from "Ústřední hřbitov" to "Křimice" in Pilsen, CZ
    # * a _run_route is a SUMO route of this run, i.e. all edges covering the vehicle
    #   movement from the edge where stop A is located to the edge where the stop B
    #   is located
    # * a _circulation_ is a sequence of runs, preferrably starting and finishing at
    #   the same stop. This is however not guaranteed as the GTFS data do not cover
    #   empty vehicle runs and vehicle runs that serve different lines are not easily
    #   detectable.
    from_stops = list(route_dict.keys())
    # Find all endpoints of vehicle runs. Each endpoint will be later
    # connected to the closest startpoint of the next run.
    endpoint_dict = defaultdict(SortedDict)
    for name_from in from_stops:
        from_times = route_dict[name_from].keys()
        for time_from in from_times:
            # A single vehicle runs starting at `name_from` and ending somewhere
            run = route_dict[name_from][time_from][0]
            _, name_to, time_to = run[-1]
            endpoint_dict[name_to][time_to] = (name_from, time_from)
    # Sort the arrival times for every endpoint from the earliest to the latest
    for key, val in endpoint_dict.items():
        # Need to maintain the order of keys
        sorted_val = SortedDict()
        for k in sorted(val.keys()):
            sorted_val[k] = val[k]
        # Replace the route_dict entry
        endpoint_dict[key] = sorted_val
    # Start with an arbitraty endpoint and try to connect all runs / partical circulations that finished
    # there with the next run that is the first to leave. This heuristics ignores
    # situations where vehicle has a longer pause at the endpoint, but this is
    # something GTFS will not tell us about.
    # As this process is modifying the run/circulation storage, possibly creating 
    # new entries, we need to repeat until no join has occured.
    # TODO: Check this, possibly it is not true and one single iteration is sufficient
    join_loop_id = 0
    vehicle_pauses = defaultdict(list)
    while True:
        join_loop_id += 1
        print('JOIN LOOP', join_loop_id)
        # Exit by default
        no_joins = True
        for endpoint_name in from_stops:
            print('JOINING RUNS AT ENDPOINT:', endpoint_name)
            # Arrival times are sorted
            # Need to get a list of keys instead of generator as the keys will
            # be deleted if processed
            arrival_times = list(endpoint_dict[endpoint_name].keys())
            for arrival_time in arrival_times:
                # Get sorted departure times for the same endpoint
                next_run = None
                # print('   arrival:', arrival_time)
                # print('   all departures:', list(route_dict[endpoint_name].keys()))
                for departure_time in route_dict[endpoint_name]:
                    # print('   departure:', departure_time)
                    if departure_time > arrival_time:
                        # This is the first run that starts after the arrival
                        # of the vehicle, use it
                        next_run = route_dict[endpoint_name][departure_time]
                        # print('   next run found!')
                        break
                if next_run is not None:
                    # Store information about the period when the vehicle is parked
                    # at this stop. We need that to decide on parking capabilities
                    # and to insert vehicle position updates as SUMO does not force
                    # vehicles once stopped at a bus stop to move forward once
                    # the preceding vehicle leaves.
                    pause = (arrival_time, departure_time)
                    vehicle_pauses[endpoint_name].append(pause)
                    # We need to connect the run that led to this endpoint with
                    # the run that departs from it
                    prev_name, prev_time = endpoint_dict[endpoint_name][arrival_time]
                    prev_run = route_dict[prev_name][prev_time]
                    # print('** prev run:', prev_run)
                    # print('** next run:', next_run)
                    # Join the previous and the next run
                    prev_stops, prev_route_id = prev_run
                    next_stops, next_route_id = next_run
                    print('-- NEW JOINED ROUTE:')
                    new_from_name = prev_stops[0][1]
                    new_from_time = prev_stops[0][2]
                    print('   ** from:', new_from_name, new_from_time)
                    new_to_name = next_stops[-1][1]
                    new_to_time = next_stops[-1][2]
                    print('   ** to:', new_to_name, new_to_time)
                    stops = prev_stops + next_stops
                    route_ids = prev_route_id + next_route_id
                    # print('   ** stops:', stops)
                    print('   ** route_ids:', route_ids)
                    # Delete old entries
                    del route_dict[endpoint_name][departure_time]
                    del route_dict[prev_name][prev_time]
                    del endpoint_dict[endpoint_name][arrival_time]
                    # Add joined entry
                    route_dict[new_from_name][new_from_time] = (stops, route_ids)
                    endpoint_dict[new_to_name][new_to_time] = (new_from_name, new_from_time)
                    # Prevent the loop from exitting at the end
                    no_joins = False
            print('** all incoming routes processed')
        if no_joins:
            print('** no joins in this loop, exitting')
            break
            
    # assert False
    
    # for name_from in from_stops:
        # print('RUN STARTING POINT:', name_from)
        # from_times = route_dict[name_from].keys()
        # for time_from in from_times:
            # run_data = route_dict[name_from][time_from]
            # if run_data is None:
                # print(f'  RUN at {time_from} already deleted')
                # continue
            # run, route_id = run_data
            # id_to, name_to, time_to = run[-1]
            # print(f'  RUN at {time_from.time()} to {name_to}, route {route_id}, arrival {time_to.time()}')
            # c = complete_run(route_dict, name_to, time_to)
            # vehicle_run_seq = [(time_from, run, [route_id])] + c
            # print(vehicle_run_seq)
            # # Delete the items and convert vehilce run into a sequence of stops
            # vehicle_run = []
            # vehicle_routes = []
            # for timestamp, trun, troutes in vehicle_run_seq:
                # tname_from = trun[0][1]
                # print(f'  deleting run from {tname_from} at {timestamp}')
                # assert timestamp in route_dict[tname_from]
                # route_dict[tname_from][timestamp] = None
                # vehicle_run += trun
                # vehicle_routes += troutes
            # # Convert vehicle run into a sequence of stops
            # # print(vehicle_run)
            # route_dict[name_from][time_from] = (vehicle_run, vehicle_routes)
    # Print all pauses
    hln_platforms_endtimestamp = [datetime(1970, 1, 1), datetime(1970, 1, 1)]
    for stop_name, pauses in vehicle_pauses.items():
        pauses.sort()
        print('PAUSES AT:', stop_name)
        for pause in pauses:
            pause_from, pause_to = pause
            if stop_name == 'Hlavní nádraží TERM':
                print('   -- multiple plaform endpoint, 2 platforms')
                if hln_platforms_endtimestamp[0] < pause_from:
                    # First platform is free
                    platform_no = 1
                    hln_platforms_endtimestamp[0] = pause_to
                elif hln_platforms_endtimestamp[1] < pause_from:
                    # Second platform is free
                    platform_no = 2
                    hln_platforms_endtimestamp[1] = pause_to
                else:
                    # No platform available
                    platform_no = -1
                print(f'      {pause_from.time()} -- {pause_to.time()} at platform {platform_no}')
            else:
                print('   -- sequential platform as a vehicle stack')
                print(f'      {pause_from.time()} -- {pause_to.time()}')
            
    return route_dict


def graph_runs(route_dict, stop_names, file_name):
    fig, ax = plt.subplots(figsize=(25, 10))
    time_formatter = mdates.DateFormatter('%H:%M')
    ax.xaxis.set_major_formatter(time_formatter)
    line_y_units = UnitData(stop_names)
    for name_from, runs in route_dict.items():
        for run_time, run_data in runs.items():
            if run_data is None:
                continue
            run, routes = run_data
            # print(run)
            stop_list = []
            time_list = []
            for _, stop_name, stop_time in run:
                if stop_name in stop_names:
                    stop_list.append(stop_name)
                    time_list.append(stop_time)
            assert stop_list
            ax.plot(time_list, stop_list, linewidth=0.5, yunits=line_y_units)
    # Show the plot
    plt.savefig(file_name, orientation='landscape')
    plt.show()


def write_stops_and_routes(net, route_edges, stop_list, stop_id_map, route_dict, file_name):
    vehicle_num = 1
    vehicles = []
    with open(file_name, 'w', encoding="utf8") as rout:
        sumolib.xml.writeHeader(rout, os.path.basename(__file__), "additional")
        # Write bus stops
        for s in stop_list:
            rout.write(f'    <busStop id="{stop_id_map[s.id]}" lane="{s.lane}" startPos="{s.startPos}" endPos="{s.endPos}" '
                       f'friendlyPos="{s.friendlyPos}" name="{s.attr_name}"/>\n')
        # Write routes
        for name_from, runs in route_dict.items():
            for run_time, run_data in runs.items():
                if run_data is None:
                    continue
                run, routes = run_data
                vehicle_id = f'tb11.{vehicle_num:02d}'
                vehicle_route_id = f'route.tb11.{vehicle_num:02d}'
                vehicle_num += 1
                print(vehicle_route_id)
                print(routes)
                edges = []
                last_edge = None
                for r in routes:
                    r_edge_list = route_edges[r]
                    # Is there a previous vehicle run that we need to connect to this one?
                    if last_edge:
                        # Last edge of the previous run
                        e1 = net.getEdge(last_edge)
                        # First edge of this run
                        e2 = net.getEdge(r_edge_list[0])
                        # In some cases e1 and e2 are identical
                        if e1 == e2:
                            print(f"looking for path from {last_edge} to {last_edge}, "
                                  f"shortening the route of the current run")
                            r_edge_list = r_edge_list[1:]
                        else:
                            elist, cost = net.getFastestPath(e1, e2, vClass="bus")
                            assert elist is not None
                            assert len(elist) > 2  # elist may be [e1,e2] if e1 == e2
                            # `elist` contains `e1` as the first element and `e2` as the last element
                            # but `e1` is already part of `edges` and `e2` is part of `r_edge_list`
                            elist = [e.getID() for e in elist[1:-1]]
                            estr = " ".join(elist)
                            print(f'adding path from `{last_edge}` to `{r_edge_list[0]}`: "{estr}"')
                            edges += elist
                    edges += r_edge_list
                    last_edge = r_edge_list[-1]
                # Construct a space-delimited string of the full route
                edge_str = ' '.join(edges)
                rout.write(f'    <route id="{vehicle_route_id}" edges="{edge_str}">\n')
                assert run
                tzero = run[0][2]
                for stop in run:
                    stop_id, stop_name, stop_dt = stop
                    # Windows-related error, cannot use timestamp for 1970-01-01
                    tto = (stop_dt - tzero).total_seconds()
                    rout.write(f'        <stop busStop="{stop_id}" duration="10" until="{int(tto)}"/>\n')
                rout.write(u'    </route>\n')
                vehicle_tuple = (vehicle_id, vehicle_route_id, tzero)
                vehicles.append(vehicle_tuple)
        rout.write('</additional>\n')
    return vehicles


def write_vehicles(vehicles, vehicle_type_id, file_name):
    with open(file_name, 'w', encoding="utf8") as rout:
        sumolib.xml.writeHeader(rout, os.path.basename(__file__), "routes")
        tzero = datetime(1970, 1, 1)
        for vehicle_id, vehicle_route_id, vehicle_depart in vehicles:
            print(vehicle_depart)
            tto = (vehicle_depart - tzero).total_seconds() - 3600
            rout.write(f'    <vehicle id="{vehicle_id}" route="{vehicle_route_id}" '
                       f'type="{vehicle_type_id}" depart="{tto}" line="11"/>\n')
        rout.write('</routes>\n')


def get_transformed_stop_ids(rts):
    elems = sumolib.xml.parse_fast(rts, "busStop", ("id", "lane", "startPos", "endPos", "friendlyPos", "name"))
    stop_list = []
    stop_id_map = {}
    stop_name_map = {}
    id_dict = defaultdict(int)
    for elem in elems:
        name = elem.attr_name
        base_id = unidecode.unidecode(name).lower().replace(" ", "_").replace(",", "_")
        base_no = id_dict[base_id] + 1
        id_dict[base_id] = base_no
        new_id = f'{base_id}#{base_no}'
        stop_id_map[elem.id] = new_id
        stop_name_map[new_id] = name
        stop_list.append(elem)
    # Sort the stop list in place according to stop name and the new stop id
    stop_list.sort(key=lambda e: (e.attr_name, stop_id_map[e.id]))
    return stop_list, stop_id_map, stop_name_map


def main():
    network = "plzen_trd_paper.net.xml"
    # parse the net
    net = sumolib.net.readNet(network)

    rts = "plzen_gtfs_pt_stops_routes.add.xml"
    stop_elems, stop_ids, stop_map = get_transformed_stop_ids(rts)

    route_data = sumolib.xml.parse(rts, ["route"])
    # print(list(route_data))

    # HLN_TERM_NAME = 'Hlavní nádraží TERM'
    # HLN_TERM_IDS = ("bs_hln_terminal0", "bs_hln_terminal1")
    # HLN_TERM_EDGES = ("E11", "E10")
    # HLN_TERM_OFFSET = 60
    # stop_map[HLN_TERM_IDS[0]] = HLN_TERM_NAME
    # stop_map[HLN_TERM_IDS[1]] = HLN_TERM_NAME

    hln = FinalStopMultiplatformElement(element_name='Hlavní nádraží TERM', base_id='bs_hln_terminal',
                                        comes_after='Hlavní nádraží',
                                        element_edges=("E11", "E10"), travel_time=60)

    print('PREPROCESSING ROUTE EDGES')
    routes = {}
    # route_edges = {}
    for r in route_data:
        # print(r.id)
        # print(r)
        try:
            route_stops = {}
            edges = r.edges.split()
            for s in r['stop']:
                bus_stop_id = stop_ids[s.busStop]
                until = int(s.until)
                route_stops[until] = SUMOPTStop(bus_stop_id, stop_map[bus_stop_id])
            # skeys = sorted(route_stops.keys())
            # print(skeys)
            # from_time = skeys[0]
            # stop_from = route_stops[from_time]
            # # Handle the loop at Hlavní nádraží that is not part of GTFS data
            # if stop_from[1] == 'Hlavní nádraží':
            #     from_time -= HLN_TERM_OFFSET
            #     stop_from = (HLN_TERM_ID, HLN_TERM_NAME)
            #     route_stops[from_time] = stop_from
            #     print(f'route {r.id} from {stop_from[1]}:{from_time}')
            #     e1 = net.getEdge(HLN_TERM_EDGE)
            #     e2 = net.getEdge(edges[0])
            #     print(f'      connecting edge `{e1.getID()}` and `{e2.getID()}`')
            #     assert e1 != e2
            #     elist, cost = net.getFastestPath(e1, e2, vClass="bus")
            #     assert elist is not None
            #     assert len(elist) > 2  # elist may be [e1,e2] if e1 == e2
            #     # `elist` contains e1 as the first element and `e2` as the last element
            #     eidlist = [e.getID() for e in elist[0:-1]]
            #     estr = " ".join(eidlist)
            #     print(f'      found path: "{estr}"')
            #     edges = eidlist + edges
            # to_time = skeys[-1]
            # stop_to = route_stops[to_time]
            # # Handle the loop at Hlavní nádraží that is not part of GTFS data
            # if stop_to[1] == 'Hlavní nádraží':
            #     to_time += HLN_TERM_OFFSET
            #     stop_to = (HLN_TERM_ID, HLN_TERM_NAME)
            #     route_stops[to_time] = stop_to
            #     print(f'route {r.id} to {stop_to[1]}:{to_time}')
            #     e1 = net.getEdge(edges[-1])
            #     e2 = net.getEdge(HLN_TERM_EDGE)
            #     print(f'      connecting edge `{e1.getID()}` and `{e2.getID()}`')
            #     assert e1 != e2
            #     elist, cost = net.getFastestPath(e1, e2, vClass="bus")
            #     assert elist is not None
            #     assert len(elist) > 2  # elist may be [e1,e2] if e1 == e2
            #     # `elist` contains e1 as the first element and `e2` as the last element
            #     eidlist = [e.getID() for e in elist[1:]]
            #     estr = " ".join(eidlist)
            #     print(f'      found path: "{estr}"')
            #     edges += eidlist
            routes[r.id] = SUMOBasePTRouteWithStops(route_stops, r.id)
            # route_edges[r.id] = edges
        except KeyError as e:
            print('No element', e)
    # print(routes)

    print()
    print('PROCESSING VEHICLE RUNS')
    line_runs = []
    graph_stops = set()
    inp = "plzen_gtfs_pt_vehicles.rou.xml"
    for veh in sumolib.xml.parse_fast(inp, "vehicle", ("id", "route", "type", "depart", "line")):
        line_no, run_id = veh.line.split('_')
        # If the line is between the lines that we shall process ...
        if line_no in LINE_LIST:
            # ... get the route prescribed for this vehicle and its stops
            srws = routes[veh.route]
            # print(f'route {veh.route} from {stop_from[1]:20s}:{from_time} to {stop_to[1]:20s}:{to_time}')
            print(f'route {veh.route} from {srws.stop_from.name}:{srws.time_from} to {srws.stop_to.name}:{srws.time_to}')
            # Time offset of this run. The time shall be added to the times of the SUMO route
            offset = int(veh.depart)
            # Create a circulation run element that is based on this SUMO vehicle route offset in time
            vehicle_run = CirculationRunElement(veh.line, srws, offset)
            line_runs.append(vehicle_run)
            # This is just to get the list of unique final stops for graphing purposes
            graph_stops.add(srws.stop_from.name)
            graph_stops.add(srws.stop_to.name)
    # print(line_routes)
    # print(graph_stops)
    # make it a list
    graph_stop_list = ['Cínová', 'Malesice', 'Křimice', 'CAN Husova', 'Hlavní nádraží', 'Ústřední hřbitov']

    # Create a collection of routes sorted by their starting time and indexed by their starting stop.
    rsbt = collect_runs_by_starting_points(line_runs)
    # Connect runs into the longest possible route
    rsbt = connect_runs(rsbt)

    if do_graph:
        # Graph the connected runs
        graph_runs(rsbt, graph_stop_list, 'runs.pdf')

    # Compute the new routes and run
    vehicles = write_stops_and_routes(stop_elems, stop_ids, rsbt, 'outplzen_gtfs_pt_stops_routes.add.xml')
    write_vehicles(vehicles, 'trolleybus_pilsen_11', 'outplzen_gtfs_pt_vehicles.tb11.rou.xml')

    # No connection between edge '-478136367#1' and edge '25259681#4'.


if __name__ == '__main__':
    main()
