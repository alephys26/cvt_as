from crewai import Agent

class CI_Agent(Agent):
    def __init__(self, map, ID: str, mode: str):
        super().__init__(
            role='Campus Incharge',
            goal="To facilitate visitors to meet their intended host inside the campus from main gate to host's location.",
            memory=True,
            verbose=True
        )
        self.map = map
        self.visitor = None
        self.destination = None
        self.path = None
        self.Id = ID
        self.speed_dict = {'car': 2.0, 'bike': 1.0, 'walk': 0.5}
        self.mode = mode
        self.speed = self.speed_dict[mode]

    def run_visitor(self, host: str, host_location: str, visitor_id: str):
        self.visitor = visitor_id
        self.host = host
        self.destination = host_location
        self.path = self.map[self.destination][1]
        return self.speed, self.path
