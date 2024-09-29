scale_factor = 5

locations = {
    # Main Gate and Offices
    'Main Gate': (0 / scale_factor, 0 / scale_factor, 0),
    'Office': (8 / scale_factor, 6 / scale_factor, 0),
    'Library': (11 / scale_factor, 10 / scale_factor, 0),
    'Data Center': (11 / scale_factor, 11 / scale_factor, 0),
    'LHC': (15 / scale_factor, 11 / scale_factor, 0),
    'CSE': (15 / scale_factor, 12 / scale_factor, 0),
    'BLB': (16 / scale_factor, 12 / scale_factor, 0),
    'BIO': (15 / scale_factor, 13 / scale_factor, 0),
    'Chemical': (16 / scale_factor, 13 / scale_factor, 0),
    'Civil': (16 / scale_factor, 14 / scale_factor, 0),
    'Mechanical': (16 / scale_factor, 15 / scale_factor, 0),
    'SME': (16 / scale_factor, 16 / scale_factor, 0),
    'Electrical': (15 / scale_factor, 14 / scale_factor, 0),
    'Physics': (15 / scale_factor, 15 / scale_factor, 0),
    'SOLA': (15 / scale_factor, 16 / scale_factor, 0),

    # Hostels
    'G1': (0 / scale_factor, 15 / scale_factor, 0),
    'G2': (0 / scale_factor, 14 / scale_factor, 0),
    'G3': (0 / scale_factor, 13 / scale_factor, 0),
    'G4': (-1 / scale_factor, 15 / scale_factor, 0),
    'G5': (-1 / scale_factor, 14 / scale_factor, 0),
    'G6': (-1 / scale_factor, 13 / scale_factor, 0),

    # Blocks and Old Mess
    'B1': (-14 / scale_factor, 0 / scale_factor, 0),
    'B2': (-15 / scale_factor, 0 / scale_factor, 0),
    'B3': (-15 / scale_factor, 0 / scale_factor, 0),
    'B4': (-14 / scale_factor, 1 / scale_factor, 0),
    'B5': (-15 / scale_factor, 1 / scale_factor, 0),
    'B6': (-16 / scale_factor, 1 / scale_factor, 0),
    'Old Mess': (-7.5 / scale_factor, 7 / scale_factor, 0),

    # Isolated Areas
    'I2': (-3 / scale_factor, -4 / scale_factor, 0),
    'I3': (-3 / scale_factor, -3 / scale_factor, 0),
    'Director House': (0 / scale_factor, -10 / scale_factor, 0),
    'Faculty Quarters': (0 / scale_factor, -11 / scale_factor, 0)
}

# Edges (pairs of connected nodes)
edges = [
    ('Main Gate', 'Office'),
    ('Main Gate', 'I2'),
    ('Main Gate', 'B1'),
    ('Main Gate', 'B3'),
    ('Main Gate', 'B5'),
    ('Main Gate', 'G4'),
    ('Main Gate', 'G3'),
    ('Main Gate', 'G1'),
    ('Main Gate', 'G6'),
    ('Main Gate', 'Director House'),
    ('Main Gate', 'Faculty Quarters'),
    ('Office', 'Library'),
    ('Library', 'Data Center'),
    ('Data Center', 'LHC'),
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
    ('B3', 'Old Mess'),
    ('B4', 'B5'),
    ('G1', 'G2'),
    ('G2', 'G3'),
    ('G3', 'G4'),
    ('G4', 'G5'),
    ('G5', 'G6'),
]