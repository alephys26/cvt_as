from mscvt_ros.network_graph import CampusMap
import sys
import random

auth_visitors_count = int(sys.argv[1])
ci_count = int(sys.argv[2])

file = open('rviz_config.rviz', 'w')
code_file = open('mscvt_ros/spawnned_visitors.py', 'w')
init_content = """Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: MarkerArray
      Namespaces:
        campus_map: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /campus_map
      Value: true"""
file.write(init_content)

campusMap = CampusMap()

for i in range(31):
    building = campusMap.building[i]
    bi_content = f"""
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Marker
      Namespaces:
        visitor: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /bi_location_marker_{building.building_name}_F1_R101
      Value: true"""
    file.write(bi_content)

for i in range(1, ci_count+1):
    ci_content = f"""
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Marker
      Namespaces:
        ci: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /ci_location_marker_CI_{i}
      Value: true"""
    file.write(ci_content)


def generate_meeting_times(length):
    meeting_times = []
    for _ in range(length):
        meeting_time = 5 if random.random() < 0.95 else 4
        meeting_times.append(meeting_time)
    return meeting_times

def get_visitors(campus_map, n_visitors):
    visitors_auth = {
        'id': [],
        'host': [],
        'host_location': []
    }
    for building in campus_map.building:
        visitors_auth['id'] += building.visitors['id']
        visitors_auth['host'] += building.visitors['host']
        visitors_auth['host_location'] += building.visitors['host_location']

    n_visitors = min(n_visitors, len(visitors_auth['host']))
    visitors_auth['meeting_time'] = generate_meeting_times(
        len(visitors_auth['host']))
    indices = random.sample(range(2, len(visitors_auth['host'])), n_visitors)
    indices.append(0)
    return indices, visitors_auth

ind, va = get_visitors(campusMap, auth_visitors_count)

code = f"""
visitors = {va}
indices = {ind}
"""
code_file.write(code)

for i in range(auth_visitors_count+1):
    visitor_content = f"""
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Marker
      Namespaces:
        visitor: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visitor_location_marker_{va['id'][ind[i]]}
      Value: true"""
    file.write(visitor_content)
visitor_content = f"""
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Marker
      Namespaces:
        visitor: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visitor_location_marker_{va['id'][1]}
      Value: true
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Marker
      Namespaces:
        visitor: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visitor_location_marker_Bandit
      Value: true"""
file.write(visitor_content)

final_content = """
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/FPS
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.5697963237762451
      Position:
        X: 0
        Y: 0
        Z: 20
      Target Frame: <Fixed Frame>
      Value: FPS (rviz_default_plugins)
      Yaw: 0
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1000
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001f7000002bcfc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b000000b000fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000006e000002bc0000018200fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000015f000002bcfc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000006e000002bc0000013200fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e1000001970000000300000a5e0000005afc0100000002fb0000000800540069006d0065010000000000000a5e0000058100fffffffb0000000800540069006d00650100000000000004500000000000000000000006f0000002bc00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 2000
  X: 100
  Y: 100
"""
file.write(final_content)
