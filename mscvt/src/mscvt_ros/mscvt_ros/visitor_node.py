from mscvt.agents.visitor import Visitor as vi
from rclpy.node import Node
from mscvt_messages.srv import Visitor
from mscvt_messages.msg import findCI
from math import sqrt


class Visitor_Node(Node):

    def __init__(self, host: str, host_location: str, ID: str):
        self.agent = vi(host, host_location, ID)
        self.travelCount = 0
        self.coordinates = (0.0, 0.0, 0.0)

        self.client = self.create_client(Visitor, 'ci_communication')
        self.request = Visitor.Request()
        self.request.visitorid = ID
        self.request.hostid = host
        self.request.hostlocation = host_location

        self.pub = self.create_publisher(findCI, 'need_ci', 1)
        self.timer = self.create_timer(1.0, self.searchForCI)
        self.subs = self.create_subscription(
            findCI, 'ci_reply', self.isCIAvailable, 10)

    def searchForCI(self):
        if self.agent.ci is not None:
            del self.timer
            return
        msg = findCI()
        msg.id = self.agent.Id
        msg.desc = 'TAKEME'
        self.pub.publish(msg)

    def isCIAvailable(self, msg):
        if (self.agent.ci is not None) and (msg.desc == 'YES'):
            self.agent.ci = msg.id
            self.talkWithCI()

    def talkWithCI(self):
        future = self.client.call_async(self.request)
        if future.done():
            result = future.result()
            self.speed = result.speed
            self.path = result.points
            self.travel()

    def travel(self):
        n = len(self.path)
        i = 0
        while i < n:
            next = self.path[i]
            grad = [0, 0, 0]
            for dim in range(3):
                grad[dim] = next[dim] - self.coordinates[dim]
            normalizer = sqrt(sum([dim**2 for dim in grad]))
            while not self.__compareCoordinates(next, self.coordinates):
                for dim in range(3):
                    self.coordinates[dim] += grad[dim] * \
                        self.speed*0.5/normalizer
            i += 1
        self.travelCount += 1
        if self.travelCount == 1:
            self.request.hostlocation = 'INSIDE'
        else:
            self.request.hostlocation = 'MAIN GATE'
        self.talkWithCI()

    def __compareCoordinates(self, c1, c2):
        if c1[0] != c2[0]:
            return False
        if c1[1] != c2[1]:
            return False
        if c1[2] != c2[2]:
            return False
        return True
