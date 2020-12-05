from __future__ import division, print_function, absolute_import


from lxml import etree
from opendrivepy.road import Road, RoadLink
from opendrivepy.roadgeometry import RoadLine, RoadSpiral, RoadArc, RoadElevation
from opendrivepy.junction import Junction, Connection
from opendrivepy.lane import Lanes, Lane, LaneLink, LaneSection, LaneWidth

from OSMtype import Node,Way

class XMLParser(object):
    def __init__(self, file):
        self.xml = etree.parse(file)
        self.root = self.xml.getroot()


    # Parses all roads in the xodr and instantiates them into objects
    # Returns a list of Road objects
    def parse_roads(self):
        ret = dict()

        for road in self.root.iter('road'):

            # Create the Road object
            name = road.get('name')
            length = road.get('length')
            id = road.get('id')
            junction = road.get('junction')

            # Parses link for predecessor and successors
            # No support for neighbor is implemented
            link = road.find('link')
            predecessor = None
            successor = None
            if link is not None:
                xpredecessor = link.find('predecessor')
                if xpredecessor is not None:
                    element_type = xpredecessor.get('elementType')
                    element_id = xpredecessor.get('elementId')
                    contact_point = xpredecessor.get('contactPoint')
                    predecessor = (RoadLink(element_type, element_id, contact_point))

                xsuccessor = link.find('successor')
                if xsuccessor is not None:
                    element_type = xsuccessor.get('elementType')
                    element_id = xsuccessor.get('elementId')
                    contact_point = xsuccessor.get('contactPoint')
                    successor = (RoadLink(element_type, element_id, contact_point))

            # Parses planView for geometry records
            xplan_view = road.find('planView')
            plan_view = list()
            for geometry in xplan_view.iter('geometry'):
                record = geometry[0].tag

                s = float(geometry.get('s'))
                x = float(geometry.get('x'))
                y = float(geometry.get('y'))
                hdg = float(geometry.get('hdg'))
                length = float(geometry.get('length'))

                if record == 'line':
                    plan_view.append(RoadLine(s, x, y, hdg, length))
                elif record == 'arc':
                    curvature = float(geometry[0].get('curvature'))
                    plan_view.append(RoadArc(s, x, y, hdg, length, curvature))
                elif record == 'spiral':
                    curv_start = float(geometry[0].get('curvStart'))
                    curv_end = float(geometry[0].get('curvEnd'))
                    plan_view.append(RoadSpiral(s, x, y, hdg, length, curv_start, curv_end))

            # Parses elevationProfile for geometry records
            elevationProfile = road.find('elevationProfile')
            elevations = list()
            for elevation in elevationProfile.iter('elevation'):
                record = geometry[0].tag

                s = float(elevation.get('s'))
                a = float(elevation.get('a'))
                b = float(elevation.get('b'))
                c = float(elevation.get('c'))
                d = float(elevation.get('d'))
                
                elevations.append(RoadElevation(s,a,b,c,d))

            # Parse lanes for lane
            xlanes = road.find('lanes')

            # Lane Sections
            xlane_section = xlanes.find('laneSection')

            # Center Lane
            center = list()
            xcenter = xlane_section.find('center')
            if xcenter is not None:
                xlane = xcenter.find('lane')
                center.append(self.parse_lane(xlane))

            # Left Lanes
            left = list()
            xleft = xlane_section.find('left')
            if xleft is not None:
                for xlane in xleft.iter('lane'):
                    left.append(self.parse_lane(xlane))

            # Right Lanes
            right = list()
            xright = xlane_section.find('right')
            if xright is not None:
                for xlane in xright.iter('lane'):
                    right.append(self.parse_lane(xlane))

            lane_section = LaneSection(left, center, right)

            lanes = Lanes(lane_section)

            new_road = Road(name, length, id, junction, predecessor, successor, plan_view, elevations, lanes)
            ret[new_road.id] = new_road
        return ret

    def parse_lane(self, xlane):

        # Attributes
        id = int(xlane.get('id'))
        type = xlane.get('type')
        level = xlane.get('level')

        # Lane Links
        xlink = xlane.find('link')
        predecessor = None
        successor = None

        if xlink is not None:
            xpredecessor = xlink.find('predecessor')
            if xpredecessor is not None:
                link_id = int(xpredecessor.get('id'))
                predecessor = LaneLink(link_id)

            xsuccessor = xlink.find('successor')
            if xsuccessor is not None:
                link_id = int(xsuccessor.get('id'))
                successor = LaneLink(link_id)

        # Width
        width = None
        xwidth = xlane.find('width')
        if xwidth is not None:
            s_offset = float(xwidth.get('sOffset'))
            a = float(xwidth.get('a'))
            b = float(xwidth.get('b'))
            c = float(xwidth.get('c'))
            d = float(xwidth.get('d'))
            width = LaneWidth(s_offset, a, b, c, d)

        return Lane(id, type, level, predecessor, successor, width)

    # TODO Add Priorities, JunctionGroups and LaneLinks
    def parse_junctions(self):
        ret = dict()

        for junction in self.root.iter('junction'):
            new_junction = Junction(junction.get('name'), junction.get('id'))

            for connection in junction.iter('connection'):
                id = connection.get('id')
                incoming_road = connection.get('incomingRoad')
                connecting_road = connection.get('connectingRoad')
                contact_point = connection.get('contactPoint')
                new_connection = Connection(id, incoming_road, connecting_road, contact_point)

                new_junction.add_connection(new_connection)

            ret[new_junction.id] = new_junction

        return ret
