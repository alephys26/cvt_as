from crewai_tools import BaseTool


class visitorTools(BaseTool):
    name = "Visitor Tools"
    description = "All tools for handling Visitor actions."

    def __call__(self):
        return self._run()

    def findCI(self):
        if self.agent.memory['ci']:
            return
        # TODO: communicate using ROS to get resultant below boolean by broadcasting movement request
        freeCI = True
        if freeCI:
            # stop broadcasting movement request
            self.agent.memory['ci'] = True
        return

    def _run(self):
        return self.findCI()
