scale_factor = 1.0

locations = {
    'Main_Gate': (0 / scale_factor, 0 / scale_factor, 0 / scale_factor),
    'Office': (8 / scale_factor, 6 / scale_factor, 0 / scale_factor),
    'Library': (11 / scale_factor, 10 / scale_factor, 0 / scale_factor),
    'Data_Center': (11 / scale_factor, 11 / scale_factor, 0 / scale_factor),
    'LHC': (15 / scale_factor, 11 / scale_factor, 0 / scale_factor),
    'CSE': (15 / scale_factor, 12 / scale_factor, 0 / scale_factor),
    'BLB': (16 / scale_factor, 12 / scale_factor, 0 / scale_factor),
    'BIO': (15 / scale_factor, 13 / scale_factor, 0 / scale_factor),
    'Chemical': (16 / scale_factor, 13 / scale_factor, 0 / scale_factor),
    'Civil': (16 / scale_factor, 14 / scale_factor, 0 / scale_factor),
    'Mechanical': (16 / scale_factor, 15 / scale_factor, 0 / scale_factor),
    'SME': (16 / scale_factor, 16 / scale_factor, 0 / scale_factor),
    'Electrical': (15 / scale_factor, 14 / scale_factor, 0 / scale_factor),
    'Physics': (15 / scale_factor, 15 / scale_factor, 0 / scale_factor),
    'SOLA': (15 / scale_factor, 16 / scale_factor, 0 / scale_factor),

    # Hostels
    'G1': (0 / scale_factor, 15 / scale_factor, 0 / scale_factor),
    'G2': (0 / scale_factor, 14 / scale_factor, 0 / scale_factor),
    'G3': (0 / scale_factor, 13 / scale_factor, 0 / scale_factor),
    'G4': (-1 / scale_factor, 15 / scale_factor, 0 / scale_factor),
    'G5': (-1 / scale_factor, 14 / scale_factor, 0 / scale_factor),
    'G6': (-1 / scale_factor, 13 / scale_factor, 0 / scale_factor),

    # Blocks and Old_Mess
    'B1': (-14 / scale_factor, 0 / scale_factor, 0 / scale_factor),
    'B2': (-15 / scale_factor, 0 / scale_factor, 0 / scale_factor),
    'B3': (-16 / scale_factor, 0 / scale_factor, 0 / scale_factor),
    'B4': (-15 / scale_factor, 1 / scale_factor, 0 / scale_factor),
    'B5': (-14 / scale_factor, 1 / scale_factor, 0 / scale_factor),
    'Old_Mess': (-7.5 / scale_factor, 7 / scale_factor, 0 / scale_factor),

    # Isolated Areas
    'I2': (-3 / scale_factor, -4 / scale_factor, 0 / scale_factor),
    'I3': (-3 / scale_factor, -3 / scale_factor, 0 / scale_factor),
    'Director_House': (0 / scale_factor, -10 / scale_factor, 0 / scale_factor),
    'Faculty_Quarters': (0 / scale_factor, -11 / scale_factor, 0 / scale_factor)
}


# Edges (pairs of connected nodes)
edges = [
    ('Main_Gate', 'Office'),
    ('Main_Gate', 'I2'),
    ('Main_Gate', 'B1'),
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
    ('LHC', 'CSE'),
    ('LHC', 'Library'),
    ('BLB', 'Chemical'),
    ('CSE', 'BIO'),
    ('BIO', 'Electrical'),
    ('Civil', 'BIO'),
    ('Chemical', 'Electrical'),
    ('Electrical', 'Physics'),
    ('Civil', 'Mechanical'),
    ('Physics', 'SOLA'),
    ('SOLA', 'Mechanical'),
    ('Chemical', 'Civil'),
    ('Physics', 'SME'),
    ('I2', 'I3'),
    ('B1', 'B2'),
    ('B2', 'B3'),
    ('B3', 'Old_Mess'),
    ('B4', 'B5'),
    ('B2', 'B4'),
    ('B5', 'B1'),
    ('G1', 'G2'),
    ('G2', 'G3'),
    ('G3', 'G4'),
    ('G4', 'G5'),
    ('G5', 'G6'),
    ('G5', 'G2'),
    ('G6', 'G1')
]
