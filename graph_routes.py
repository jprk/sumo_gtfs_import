import os
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import unidecode
from datetime import datetime, timedelta
from matplotlib.category import UnitData
from collections import OrderedDict, defaultdict, namedtuple
from sortedcontainers import SortedDict
import portion

import sumolib

do_graph = True
LINE_LIST = ['11']


class SUMOPTStop:
    """
    Data about a public transport stop in SUMO.
    """

    def __init__(self, stop_id, name, lane=None):
        self.id = stop_id
        self.name = name
        self.lane = lane

    def __repr__(self):
        return f'SUMOPTStop({repr(self.id)},{repr(self.name)})'


class SUMOPTMultiplatformStop(SUMOPTStop):
    """
    Special public transport stop represented by multiple instances of SUMOPTStop in SUMO.
    """

    def __init__(self, stop_id, name, element_edges):
        super().__init__(stop_id, name)
        self.num_platforms = len(element_edges)
        self.stops = [SUMOPTStop(f'{stop_id}_platform{i+1}', name, lane=f'{element_edges[i]}_0')
                      for i in range(self.num_platforms)]
        self.edges = element_edges
        self.occupied = [portion.empty() for i in range(self.num_platforms)]

    def __repr__(self):
        return f'SUMOPTMultiplatformStop({repr(self.id)},{repr(self.name)}, {repr(self.edges)})'

    def get_free_platform(self, time_from, time_to):
        interval = portion.closed(time_from, time_to)
        idx = 0
        for occupancy_interval in self.occupied:
            if not occupancy_interval & interval:
                break
            idx += 1
        if idx < self.num_platforms:
            # Join the original occupancy interval set and the new interval
            self.occupied[idx] |= interval
            platform_edge = self.edges[idx]
            stop = self.stops[idx]
        else:
            # No platform found.
            # TODO: Generate warning or error?
            idx = None
            platform_edge = None
            stop = None
        return stop, platform_edge, idx


class SUMORoute:
    """
    Representation of SUMO vehicle route.
    """
    def __init__(self, route_id, edges):
        """
        Instantiate a new vehicle route.
        :param route_id: String ID of SUMO route that corresponds to this run
        :type route_id: str
        :param edges: SUMO network edges representing route `route_id`
        :type edges: list[str]
        """
        self.sumo_route_id = route_id
        self.sumo_route_edges = edges

    def get_edges(self):
        """
        Return a sequence of SUMO edges representing this route.
        :return: A list of edges for this route.
        :rtype: list[str]
        """
        return self.sumo_route_edges


class SUMOBasePTRouteWithStops(SUMORoute):
    """
    Holds data for a single base route serviced by a PT vehicle.
    Base routes are routes that are virtual, they always start at midnight and are supposed to be
    converted into real routes by applying a concrete offset using `offset_by()` method.
    """

    def __init__(self, route_id, edges, stops):
        """
        Instantiate a new PT vehicle route.
        The element contains time-mapped stops visited during this run and its SUMO route ID.
        :param stops: Dictionary of stops indexed by arrival time
        :type stops: dict[int, SUMOPTStop]
        :param route_id: String ID of SUMO route that corresponds to this run
        :type route_id: str
        :param edges: SUMO network edges representing route `route_id`
        :type edges: list[str]
        """
        # Initialise our parent object
        super().__init__(route_id, edges)
        # And add our own initialisation block for PT stops belonging to this route.
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
        return SUMOConcretePTRouteWithStops(self.sumo_route_id, self.sumo_route_edges, stops)


class SUMOConcretePTRouteWithStops(SUMORoute):
    """
    Holds data for a single base route serviced by a PT vehicle.
    Base routes are routes that are virtual, they always start at midnight and are supposed to be
    converted into real routes by applying a concrete offset using `offset_by()` method.
    """

    def __init__(self, route_id, edges, stops):
        """
        Instantiate a new PT vehicle route.
        The element contains time-mapped stops visited during this run and its SUMO route ID.
        :param stops: Dictionary of stops indexed by arrival time
        :type stops: OrderedDict
        :param route_id: String ID of SUMO route that corresponds to this run
        :type route_id: str
        """
        # Initialise our parent object
        super().__init__(route_id, edges)
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

    def __repr__(self):
        return f'SUMOConcretePTRouteWithStops({repr(self.stop_from)}, {self.time_from.time()}, {repr(self.stop_to)}, ' \
               f'{self.time_to.time()}, {repr(self.sumo_route_id)})'


class CirculationElement:
    def __init__(self, line_no):
        """
        Create a single circulation run that is based on a time-offset SUMO route with stops.
        :param line_no: Line number
        """
        self.line_no = line_no
        # TODO: Need something like SUMOPTRoute instance here
        self.run = SUMORoute('empty', [])  # All instances are bound to have this

    def get_edges(self):
        """
        Return SUMO edges representing vehicle route for this CirculationElement.
        :return: List of edges representing a route.
        :rtype: list[str]
        """
        if self.run is None:
            raise NotImplementedError('A CirculationElement instance needs to have `self.run` initialised.')
        return self.run.get_edges()


class CirculationRunElement(CirculationElement):
    """
    Holds data for a single run in a vehicle circulation.
    """

    def __init__(self, line_no, route_with_stops, time_offset):
        """
        Create a single circulation run that is based on a time-offset SUMO route with stops.
        :param line_no: Line number
        :type line_no: str
        :param route_with_stops: Basic SUMO PT vehicle route with its stops, originating at time=0
        :type route_with_stops: SUMOBasePTRouteWithStops
        :param time_offset: Time offset of this run in seconds since midnight
        """
        super().__init__(line_no)
        self.run = route_with_stops.offset_by(time_offset)

    def get_stop_from(self):
        """
        Get the stop where this circulation run starts.
        :return: Starting stop
        :rtype: SUMOPTStop
        """
        return self.run.stop_from

    def get_time_from(self):
        """
        Get the time at which this circulation run starts.
        :return: Starting time
        :rtype: datetime
        """
        return self.run.time_from

    def get_stop_to(self):
        """
        Get the stop where this circulation run ends.
        :return: Ending stop
        :rtype: SUMOPTStop
        """
        return self.run.stop_to

    def get_time_to(self):
        """
        Get the time at which this circulation run ends.
        :return: Ending time
        :rtype: datetime
        """
        return self.run.time_to

    def append(self, next_cre):
        """
        Append a circulation run element to this run, creating a new instance/
        :param next_cre:
        :return:
        """

    def __repr__(self):
        return f'CirculationRunElement({repr(self.line_no)}, {repr(self.run)})'


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

    def __init__(self, line_no, stop, time_from, time_to):
        """

        :type time_from: datetime.datetime
        :type time_to: datetime.datetime
        :type stop: SUMOPTMultiplatformStop
        :type line_no: str
        :param line_no:
        :param stop:
        :param time_from:
        :param time_to:
        """
        super().__init__(line_no)
        self.mpstop = stop
        self.time_from = time_from
        self.time_to = time_to
        # Determine the stop instance, free platform and the edge it is on
        self.stop, self.edge_id, self.platform_id = stop.get_free_platform(time_from, time_to)
        # Create a single-element route
        route_id = f'{stop.id}_platform{self.platform_id}'
        edges = [self.edge_id]
        stops = OrderedDict([(self.time_from, self.stop), (self.time_to, self.stop)])
        self.run = SUMOConcretePTRouteWithStops(route_id, edges, stops)
        assert self.edge_id


class FinalStopMultiplatformElementGenerator:
    """
    Final stop of a vehicle line (run) where vehicles normally park at a parking lot
    waiting for their next turn. This cannot be (probably) represented as a SUMO
    parking area, and we represent it as a set of parallel one-way lanes, each with
    its own bus stop (and possibly also EV charger).
    This class generates a CirculationElement / FinalStopElement with the given timing.
    """

    def __init__(self, name, base_id, comes_after, travel_time, element_edges):
        self.stop = SUMOPTMultiplatformStop(base_id, name, element_edges)
        self.name = name
        self.base_id = base_id
        self.prev_name = comes_after
        self.timedelta = timedelta(seconds=travel_time)

    def generate_between(self, route_elem_0, route_elem_1):
        """
        Generate a new route element representing this stop.
        The element is assumed to be placed between the `route_elem_0` and `route_elem_1` route elements.
        :param route_elem_0: Previous route element before hitting this additional stop
        :type route_elem_0: CirculationRunElement
        :param route_elem_1: Next route element that will be served after leaving this additional stop
        :type route_elem_1: CirculationRunElement
        :return: Final stop witch appropriate timestamp and platform location
        :rtype: FinalStopMultiplatformElement
        """
        time_from = route_elem_0.get_time_to() + self.timedelta
        time_to = route_elem_1.get_time_from() - self.timedelta
        return FinalStopMultiplatformElement('unknown', self.stop, time_from, time_to)

    def get_sumo_busstops(self):
        busstops = []
        BusStop = namedtuple('busStop', ['id', 'lane', 'startPos', 'endPos', 'friendlyPos', 'attr_name'])
        for stop in self.stop.stops:
            s = BusStop(id=stop.id, lane=stop.lane,
                        startPos='15.0', endPos='25.0', friendlyPos='true', attr_name=stop.name)
            busstops.append(s)
        return busstops


class Circulation:
    """
    A sequence of CirculationElements that make a vehicle circulation.
    """

    def __init__(self, element=None):
        self.sequence = []
        if element is not None:
            self.sequence.append(element)

    def append(self, acirc):
        seq = self.sequence + acirc.sequence
        new_circ = Circulation()
        new_circ.sequence = seq
        return new_circ

    def get_stop_from(self):
        """
        Get the stop where this circulation starts.
        :return: Starting stop
        :rtype: SUMOPTStop
        """
        first_cre = self.sequence[0]
        return first_cre.run.stop_from

    def get_time_from(self):
        """
        Get the time at which this circulation starts.
        :return: Starting time
        :rtype: datetime
        """
        first_cre = self.sequence[0]
        return first_cre.run.time_from

    def get_stop_to(self):
        """
        Get the stop where this circulation ends.
        :return: Ending stop
        :rtype: SUMOPTStop
        """
        last_cre = self.sequence[-1]
        return last_cre.run.stop_to

    def get_time_to(self):
        """
        Get the time at which this circulation ends.
        :return: Ending time
        :rtype: datetime
        """
        last_cre = self.sequence[-1]
        return last_cre.run.time_to

    def get_route_ids(self):
        """
        Return a sequence of route IDs for this circulation.
        A route ID may be a SUMO route ID or a GTFS route ID or some other ID used by the user.
        :return: List of strings containing route IDs for this circulation.
        :rtype: list[str]
        """
        return [x.line_no for x in self.sequence]

    def conditional_extend(self, extra_stop):
        """
        Prepend or append an extra final stop to a route in case that the route starts or ends at a given stop.
        Used to extend the route with final stops that are not mentioned in GTFS, like stopping at dedicated parking
        area or EV charger.
        :param extra_stop: Description of the extra stop, contains reference to the final stop it extends.
        :type extra_stop: FinalStopMultiplatformElementGenerator
        """
        new_sequence = []
        elem_iter = iter(self.sequence)
        prev_elem = next(elem_iter)
        for next_elem in elem_iter:
            new_sequence.append(prev_elem)
            # Check if the previous route element ends in the stop that should be extended
            if prev_elem.get_stop_to().name == extra_stop.prev_name:
                # Generate a new route element corresponding to the visit of `extra_stop` and add it to the
                # sequence of route elements
                new_element = extra_stop.generate_between(prev_elem, next_elem)
                new_sequence.append(new_element)
            # The next_elem will be stored in the next cycle (or after leaving the loop)
            prev_elem = next_elem
        # Append the last piece
        new_sequence.append(prev_elem)
        self.sequence = new_sequence


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
    # We need the second level of the route dict to be sorted according to departure times.
    # We could have used defaultdict(SortedDict) but as we need to check the keys of SortedDict for uniqueness
    # and this has to be done manually, we will build the dictonary of sorted dictionaries manually..
    run_dict = {}
    # Loop over all CirculationRunElements in `all_runs` and reorganise them by their "from" stops.
    for cre in all_runs:
        # A single run from the starting stop `sfrom` at `tfrom` to a final stop `sfinal` at `tfinal`.
        # This run has a GTFS id `run_id`, vehicle travels along SUMO route `route_id` and contains stops
        # listed in `stops`.
        # Every stop is represented as a tuple (id,name).
        # Times are represented as seconds since midnight
        # if tfinal > 95000:
        #     # print('   skipped')
        #     continue
        print(f'GTFS run {cre.line_no}:')
        # Stop sequence ordered by stop time
        stop_seq = []
        for stop_time, stop in cre.run.route_stops.items():
            print(f' -- {stop.name:20s}: {stop_time.time()}')
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
        starting_stop_name = cre.get_stop_from().name  # sfrom[1]  # sfrom[0] would be stop id
        # Convert the starting timestamp to a datetime
        # TODO: This is probably the same as the timestamp of the first element in `stop_seq`
        datetime_from = cre.get_time_from()
        # Enter the run into the collection of runs that start at `starting_stop_name` as a run that starts at
        # `datetime_from` Note that the run_dict[starting_stop_name] is (a) not build starting from the smallest
        # timestamp, (b) implemented as a dictionary so the iteration over it is not guaranteed to occur in the same
        # order as was the order of inserting the keys. We will make sure that routes are sorted by their departure
        # times by switching to OderedDict below.
        if starting_stop_name in run_dict:
            # The `starting_stop_name` key exists, what about the second level?
            if datetime_from in run_dict[starting_stop_name]:
                # This is not handled yet: two runs starting at the same time
                raise NotImplementedError(f'I do not know how to handle two runs from `{starting_stop_name}` starting '
                                          f'at {datetime_from}')
        else:
            run_dict[starting_stop_name] = SortedDict()
        # Every element of `run_dict` is an instance of Circulation. This way the elements can be joined together
        # at their final stops with other elements that start there.
        run_dict[starting_stop_name][datetime_from] = Circulation(element=cre)
        print(f' ** added as run ({starting_stop_name}, {datetime_from})\n')

    # Sort `run_dict` dictionary entries for every key and store them in that sorted order.
    # For every starting point (first-level key) we order the particular runs from that starting point by the
    # departure time and put them into an SortedDict instance.
    # for key, val in run_dict.items():
    #    # Need to maintain the order of keys
    #    sorted_val = SortedDict()
    #    for k in val.keys():
    #        sorted_val[k] = val[k]
    #    # Replace the run_dict entry
    #    run_dict[key] = sorted_val

    return run_dict


def connect_runs(route_dict):
    """

    :type route_dict: dict[SortedDict[Circulation]]
    """
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
            # A single Circulation instance starting at `name_from` and ending somewhere
            # At this moment the instance contains just a single run
            circ = route_dict[name_from][time_from]
            name_to = circ.get_stop_to().name
            time_to = circ.get_time_to()
            # The run is a se_, name_to, time_to = run[-1]
            endpoint_dict[name_to][time_to] = (name_from, time_from)
    # Sort the arrival times for every endpoint from the earliest to the latest
    # for key, val in endpoint_dict.items():
    #    # Need to maintain the order of keys
    #    sorted_val = SortedDict()
    #    for k in sorted(val.keys()):
    #        sorted_val[k] = val[k]
    #    # Replace the route_dict entry
    #    endpoint_dict[key] = sorted_val
    # Start with an arbitraty endpoint and try to connect all runs / partial circulations that finished
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
                next_circ = None
                # print('   arrival:', arrival_time)
                # print('   all departures:', list(route_dict[endpoint_name].keys()))
                for departure_time in route_dict[endpoint_name]:
                    # print('   departure:', departure_time)
                    if departure_time > arrival_time:
                        # This is the first run that starts after the arrival
                        # of the vehicle, use it
                        next_circ = route_dict[endpoint_name][departure_time]
                        # print('   next run found!')
                        break
                if next_circ is not None:
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
                    prev_circ = route_dict[prev_name][prev_time]
                    # print('** prev run:', prev_circ)
                    # print('** next run:', next_circ)
                    # Join the previous and the next run
                    joined_circ = prev_circ.append(next_circ)
                    # prev_stops, prev_route_id = prev_circ
                    # next_stops, next_route_id = next_circ
                    print('-- NEW JOINED ROUTE:')
                    new_from_name = joined_circ.get_stop_from().name
                    new_from_time = joined_circ.get_time_from()
                    print(f'   ** from: `{new_from_name}` at {new_from_time.time()}')
                    new_to_name = joined_circ.get_stop_to().name
                    new_to_time = joined_circ.get_time_to()
                    print(f'   ** to: `{new_to_name}` at {new_to_time.time()}')
                    # stops = prev_stops + next_stops
                    # route_ids = prev_route_id + next_route_id
                    # print('   ** stops:', stops)
                    print(f'   ** route_ids: {joined_circ.get_route_ids()}')
                    # Delete old entries
                    del route_dict[endpoint_name][departure_time]
                    del route_dict[prev_name][prev_time]
                    del endpoint_dict[endpoint_name][arrival_time]
                    # Add joined entry
                    route_dict[new_from_name][new_from_time] = joined_circ
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


def extend_runs(circulations, extra_stop):
    """

    :param circulations:
    :type circulations: dict[dict[Circulation]]
    :param extra_stop:
    :type extra_stop: FinalStopMultiplatformElementGenerator
    :return:
    :rtype: dict[dict[Circulation]]
    """
    for stop_from in circulations:
        for time_from in circulations[stop_from]:
            # Get a single circulation, i.e. a sequence of vehicle runs between some final stops
            circ = circulations[stop_from][time_from]
            # Loop over all runs in this circulation, looking for beginning or end stop being the
            # _previous stop_ of the `extra_stop`, injecting the extra stop where appropriate
            circ.conditional_extend(extra_stop)
    # TODO: Probably not necessary as we modify the dictionary in place ...
    return circulations


def graph_runs(circulation_dict, stop_names, file_name):
    """
    Create a PDF image containing all circulations between given stops in the network.
    :param circulation_dict:
    :type circulation_dict: Dict[str, SortedDict[datetime.datetime, CirculationRunElement]]
    :param stop_names:
    :param file_name:
    """
    fig, ax = plt.subplots(figsize=(25, 10))
    time_formatter = mdates.DateFormatter('%H:%M')
    ax.xaxis.set_major_formatter(time_formatter)
    line_y_units = UnitData(stop_names)
    for name_from, circulations in circulation_dict.items():
        for circulation_start, circulation in circulations.items():
            # TODO: Does this ever happen?
            if circulation is None:
                continue
            stop_list = []
            time_list = []
            for cre in circulation.sequence:
                if isinstance(cre, CirculationRunElement):
                    for stop_time, stop_obj in cre.run.route_stops.items():
                        if stop_obj.name in stop_names:
                            stop_list.append(stop_obj.name)
                            time_list.append(stop_time)
            assert stop_list
            ax.plot(time_list, stop_list, linewidth=0.5, yunits=line_y_units)
    # Show the plot
    plt.savefig(file_name, orientation='landscape')
    plt.show()


def write_stops_and_routes(net, stop_list, stop_id_map, circulation_dict, file_name):
    vehicle_num = 1
    vehicles = []
    with open(file_name, 'w', encoding="utf8") as rout:
        sumolib.xml.writeHeader(rout, os.path.basename(__file__), "additional")
        # Write bus stops
        for s in stop_list:
            rout.write(f'    <busStop id="{stop_id_map[s.id]}" lane="{s.lane}" startPos="{s.startPos}" endPos="{s.endPos}" '
                       f'friendlyPos="{s.friendlyPos}" name="{s.attr_name}"/>\n')
        # Write routes
        for name_from, circulations in circulation_dict.items():
            for circulation_start, circulation in circulations.items():
                # TODO: Does this ever happen?
                if circulation is None:
                    continue
                vehicle_id = f'tb11.{vehicle_num:02d}'
                vehicle_route_id = f'route.tb11.{vehicle_num:02d}'
                vehicle_num += 1
                print(vehicle_route_id)
                edges = []
                last_edge = None
                for cre in circulation.sequence:
                    # Get the list of edges representing SUMO vehicle route for this CirculationElement
                    r_edge_list = cre.get_edges()
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
                # Write the route but do not close it, we need to add all stops of the PT vehicle during this route.
                rout.write(f'    <route id="{vehicle_route_id}" edges="{edge_str}">\n')
                # Collect and write the stops. Vehicle will appear at the relative time 0 at the first stop
                # and disappear after circulating the whole circulation at its last stop. This double loop
                # covers all original single trip from one final stop to another, which were originally served
                # by different vehicles.
                zero_time = circulation.get_time_from()
                for cre in circulation.sequence:
                    for stop_time, stop_obj in cre.run.route_stops.items():
                        # Windows-related error, cannot use timestamp for 1970-01-01
                        tto = (stop_time - zero_time).total_seconds()
                        rout.write(f'        <stop busStop="{stop_obj.id}" duration="10" until="{int(tto)}"/>\n')
                rout.write(u'    </route>\n')
                vehicle_tuple = (vehicle_id, vehicle_route_id, zero_time, stop_time)
                vehicles.append(vehicle_tuple)
        rout.write('</additional>\n')
    return vehicles


def write_vehicles(vehicles, vehicle_type_id, file_name):
    with open(file_name, 'w', encoding="utf8") as rout:
        sumolib.xml.writeHeader(rout, os.path.basename(__file__), "routes")
        midnight_datetime = datetime(1970, 1, 1)
        for vehicle_id, vehicle_route_id, vehicle_depart, vehicle_arrive in vehicles:
            print(f'-- vehicle_id: {vehicle_id} departs at {vehicle_depart.time()} to route {vehicle_route_id}'
                  f' and finishes as {vehicle_arrive.time()}')
            tto = (vehicle_depart - midnight_datetime).total_seconds()
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

    hln = FinalStopMultiplatformElementGenerator(
        name='Hlavní nádraží - terminál', base_id='bs_hln_terminal', comes_after='Hlavní nádraží',
        element_edges=("E11", "E10"), travel_time=60)

    stop_elems += hln.get_sumo_busstops()
    # TODO: This is a hack
    for stop in hln.stop.stops:
        stop_ids[stop.id] = stop.id

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
            base_route = SUMOBasePTRouteWithStops(r.id, edges, route_stops)
            routes[r.id] = base_route
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
            print(f'-- route {veh.route} from {srws.stop_from.name}:{srws.time_from} to {srws.stop_to.name}:{srws.time_to}')
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

    # Create a collection of runs indexed by their starting stop and for each starting stop sorted by
    # their starting time.
    rsbt = collect_runs_by_starting_points(line_runs)
    # Connect runs into the longest possible runs
    rsbt = connect_runs(rsbt)
    # Modify the collected runs by adding stop elements that are not official final stops
    rsbt = extend_runs(rsbt, hln)

    if do_graph:
        # Graph the connected runs
        graph_runs(rsbt, graph_stop_list, 'runs.pdf')

    # Compute the new routes and run
    vehicles = write_stops_and_routes(
        net, stop_elems, stop_ids, rsbt, 'outplzen_gtfs_pt_stops_routes.add.xml')
    write_vehicles(
        vehicles, 'trolleybus_pilsen_11', 'outplzen_gtfs_pt_vehicles.tb11.rou.xml')

    # No connection between edge '-478136367#1' and edge '25259681#4'.


if __name__ == '__main__':
    main()
