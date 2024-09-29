# from crewai import Agent

# class Visitor(Agent):
#     def __init__(self, host: str, host_location: str, ID: str):
#         super().__init__(role='Visitor',
#                          goal="Meet host at the host's location using CI's and BI's help.",
#                          backstory='A guest on campus, unfamiliar with the layout, seeking help to meet your host.',
#                          memory=True,
#                          verbose=True)
#         self.ci = ''
#         self.Id = ID
#         self.host = host
#         self.destination = host_location

class Visitor():
    def __init__(self, host: str, host_location: str, ID: str):
        self.ci = ''
        self.Id = ID
        self.host = host
        self.destination = host_location
