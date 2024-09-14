# from xyz_map import xyz
class Building:
    def __init__(self, name):
        self.name = name
        self.setMyDetails()

    def setMyDetails(self):
        match self.name:
            case 'G5':
                self.residentList = {
                    'B21CS079': 1,
                    'B21AI048': 2  # ,
                    # ...
                }
                self.authorisation = {
                    'B21CS079': ['Prerna', 'Virendra'],
                    'B21AI048': ['Sunny']
                }
                self.map = None  # xyz
                self.BI_Id = 'B21AI021'
        # SIMILARLY ALL THE BUILDINGS WILL BE FILLED HERE
