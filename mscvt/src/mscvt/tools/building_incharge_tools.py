from crewai_tools import BaseTool
import time


class replyToCI(BaseTool):
    name = "replyToCI"
    description = "Reply to CI's request."

    def __call__(self):
        self._run()

    def checkHostPresence(self, host):
        return host in self.agent.memory['map']

    def isVisitorAuthorized(self, host, visitor):
        return visitor in self.agent.memory['auth'][host]

    def isHostFree(self, host):
        if self.agent.memory['meet'][host] == None:
            return True
        if self.agent.memory['meet'][host] <= time.time():
            self.agent.memory['meet'][host] = None
            return True
        return False

    def tellPath(self, host, visitor):
        if self.checkHostPresence(host):
            if self.isVisitorAuthorized(host, visitor):
                if self.isHostFree(host):
                    self.agent.memory['meet'][host] = time.time(
                    ) + self.meeting_time
                    return "GO", self.agent.memory['path'][host]
                else:
                    return "WAIT", None
            else:
                return "UNAUTHORIZED", None
        else:
            return "DNE", None

    def isOOS(self):
        return self.isHostFree(self.agent.memory['id'])

    def OOSHandler(self):
        remaining_oos_time = self.agent.memory['meet'][self.agent.memory['id']] - time.time()
        return f"OOS for {remaining_oos_time}", None

    def _run(self):
        self.meeting_time = 3
        if self.isOOS():
            self.OOSHandler()
        return self.tellPath(self.agent.memory['host'], self.agent.memory['visitor'])
