scale_factor = 1.0

locations = {
    # Main Gate
    'Main_Gate': (0 / scale_factor, 0 / scale_factor, 0 / scale_factor),

    # Administrative and Academic Buildings
    'Office': (5 / scale_factor, 7 / scale_factor, 0 / scale_factor),
    'Library': (7 / scale_factor, 8 / scale_factor, 0 / scale_factor),
    'Data_Center': (8 / scale_factor, 9 / scale_factor, 0 / scale_factor),
    'LHC': (10 / scale_factor, 9 / scale_factor, 0 / scale_factor),
    'CSE': (11 / scale_factor, 10 / scale_factor, 0 / scale_factor),
    'BLB': (12 / scale_factor, 11 / scale_factor, 0 / scale_factor),
    'BIO': (13 / scale_factor, 12 / scale_factor, 0 / scale_factor),
    'Chemical': (14 / scale_factor, 13 / scale_factor, 0 / scale_factor),
    'Civil': (15 / scale_factor, 14 / scale_factor, 0 / scale_factor),
    'Mechanical': (16 / scale_factor, 15 / scale_factor, 0 / scale_factor),
    'SME': (17 / scale_factor, 16 / scale_factor, 0 / scale_factor),
    'Electrical': (18 / scale_factor, 17 / scale_factor, 0 / scale_factor),
    'Physics': (19 / scale_factor, 18 / scale_factor, 0 / scale_factor),
    'SOLA': (20 / scale_factor, 19 / scale_factor, 0 / scale_factor),

    # Hostels
    'G1': (-5 / scale_factor, 15 / scale_factor, 0 / scale_factor),
    'G2': (-6 / scale_factor, 14 / scale_factor, 0 / scale_factor),
    'G3': (-7 / scale_factor, 13 / scale_factor, 0 / scale_factor),
    'G4': (-8 / scale_factor, 12 / scale_factor, 0 / scale_factor),
    'G5': (-9 / scale_factor, 11 / scale_factor, 0 / scale_factor),
    'G6': (-10 / scale_factor, 10 / scale_factor, 0 / scale_factor),

    # Blocks and Old Mess
    'B1': (-15 / scale_factor, -2 / scale_factor, 0 / scale_factor),
    'B2': (-16 / scale_factor, -3 / scale_factor, 0 / scale_factor),
    'B3': (-17 / scale_factor, -3 / scale_factor, 0 / scale_factor),
    'B4': (-18 / scale_factor, -2 / scale_factor, 0 / scale_factor),
    'B5': (-19 / scale_factor, -1 / scale_factor, 0 / scale_factor),
    'B6': (-20 / scale_factor, 0 / scale_factor, 0 / scale_factor),
    'Old_Mess': (-12 / scale_factor, 8 / scale_factor, 0 / scale_factor),

    # Isolated Areas
    'I2': (-5 / scale_factor, -5 / scale_factor, 0 / scale_factor),
    'I3': (-6 / scale_factor, -4 / scale_factor, 0 / scale_factor),
    'Director_House': (-7 / scale_factor, -10 / scale_factor, 0 / scale_factor),
    'Faculty_Quarters': (-8 / scale_factor, -11 / scale_factor, 0 / scale_factor)
}


# Edges (pairs of connected nodes)
edges = [
    ('Main_Gate', 'Office'),
    ('Main_Gate', 'I2'),
    ('Main_Gate', 'B1'),
    ('Main_Gate', 'B3'),
    ('Main_Gate', 'B5'),
    ('Main_Gate', 'G4'),
    ('Main_Gate', 'G3'),
    ('Main_Gate', 'G1'),
    ('Main_Gate', 'G6'),
    ('Main_Gate', 'Director_House'),
    ('Main_Gate', 'Faculty_Quarters'),
    ('Office', 'Library'),
    ('Library', 'Data_Center'),
    ('Data_Center', 'LHC'),
    ('LHC', 'BLB'),
    ('BLB', 'Chemical'),
    ('CSE', 'BIO'),
    ('BIO', 'Electrical'),
    ('Chemical', 'Electrical'),
    ('Electrical', 'Physics'),
    ('Civil', 'Mechanical'),
    ('Physics', 'SOLA'),
    ('SOLA', 'Mechanical'),
    ('I2', 'I3'),
    ('B1', 'B2'),
    ('B2', 'B3'),
    ('B3', 'Old_Mess'),
    ('B4', 'B5'),
    ('G1', 'G2'),
    ('G2', 'G3'),
    ('G3', 'G4'),
    ('G4', 'G5'),
    ('G5', 'G6'),
]
