import time
import rclpy.logging

class BI_Agent():
    def __init__(self, path: dict[str, tuple[float, list[tuple]]], residentList: dict[str, str], authorisation: dict[str, list[str]], ID: str):
        # super().__init__(role='Building Incharge',
        #                  goal='To facilitate visitors to meet their intended host inside their building of care.',
        #                  backstory='A dedicated building manager, trusted to ensure visitors reach their hosts smoothly inside your building',
        #                  memory=True,
        #                  verbose=True)
        self.meeting_time = 3
        self.residentList = residentList
        self.auth = authorisation
        self.Id = ID
        self.meet = {}
        for resident in residentList:
            self.meet[resident] = None
        # path is a dict[key=resident, value=tuple[distance, list[points]]]
        self.path = path
        self.travel_time = None
        self.logger = rclpy.logging.get_logger('BI_Agent')

    def checkHostPresence(self, host: str) -> bool:
        return host in self.residentList

    def isVisitorAuthorized(self, host: str, visitor: str) -> bool:
        return visitor in self.auth[host]

    def isHostFree(self, host: str) -> bool:
        if self.meet[host] is None:
            return True
        if self.meet[host] <= time.time():
            self.meet[host] = None
            return True
        return False

    def tellPath(self, host: str, visitor: str) -> tuple[str, int, list[tuple[float, float, float]]]:
        if self.checkHostPresence(host):
            if self.isVisitorAuthorized(host, visitor):
                if self.isHostFree(host):
                    hostPath = self.path[host]
                    self.travel_time = hostPath[0] / 0.5
                    self.meet[host] = time.time() + self.meeting_time + self.travel_time
                    self.logger.info(f'Host {host} is free. Visitor {visitor} can proceed to meet.')
                    return "GO", 0, hostPath
                else:
                    remaining_time = self.meet[host] - time.time()
                    self.logger.info(f'Host {host} is busy. Visitor {visitor} needs to wait for {remaining_time:.2f} seconds.')
                    return "WAIT", remaining_time, []
            else:
                self.logger.warning(f'Visitor {visitor} is unauthorized to meet host {host}.')
                return "UNAUTHORIZED", 0, []
        else:
            self.logger.warning(f'Host {host} does not exist.')
            return "DNE", 0, []

    def isOOS(self) -> bool:
        return self.isHostFree(self.Id)

    def OOSHandler(self) -> tuple[str, int, list[None]]:
        remaining_oos_time = self.meet[self.Id] - time.time()
        self.logger.info(f'BI Agent {self.Id} is out of service. Remaining time: {remaining_oos_time:.2f} seconds.')
        return f"OOS", remaining_oos_time, []

    def run(self, host: str, visitor: str) -> tuple[str, int, list[tuple[float, float, float]]]:
        if self.isOOS():
            return self.OOSHandler()
        return self.tellPath(host, visitor)
