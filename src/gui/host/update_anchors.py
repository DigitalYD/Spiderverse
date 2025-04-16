#!/usr/bin/env python3

import os
import sys
import argparse
import re

# Define the paths to the relevant files
LIDAR_WS_PATH = os.path.expanduser("~/Documents/Spiderverse/lidar_ws")
BILATERATION_LAUNCH_PATH = os.path.join(LIDAR_WS_PATH, "src/lidar_udp_receiver/launch/bilateration_slam_launch.py")
TRILATERATION_LAUNCH_PATH = os.path.join(LIDAR_WS_PATH, "src/lidar_udp_receiver/launch/trilateration_slam_launch.py")
BILATERATION_PY_PATH = os.path.join(LIDAR_WS_PATH, "src/lidar_udp_receiver/lidar_udp_receiver/bilateration.py")
TRILATERATION_PY_PATH = os.path.join(LIDAR_WS_PATH, "src/lidar_udp_receiver/lidar_udp_receiver/trilateration.py")

def parse_arguments():
    parser = argparse.ArgumentParser(description='Update UWB anchor positions for bilateration and trilateration.')
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Bilateration command
    bi_parser = subparsers.add_parser('bi', help='Update bilateration anchor positions')
    bi_parser.add_argument('anchor1', type=str, help='Anchor 1 position (x,y,z) in centimeters')
    bi_parser.add_argument('anchor2', type=str, help='Anchor 2 position (x,y,z) in centimeters')
    bi_parser.add_argument('--reference_y', type=float, 
                           help='Reference Y value for bilateration (default: 300.0). Used to select between ambiguous solutions; typically set to the expected Y position of the robot. For a robot moving in a hallway with anchors along the X-axis, this would be the approximate Y-distance from the anchor line.', 
                           default=300.0)
    bi_parser.add_argument('--positive_y', action='store_true', 
                           help='Prefer positive Y solutions (default: True). When set, the system will prefer positions with positive Y values when there are ambiguous solutions.')
    
    # Trilateration command
    tri_parser = subparsers.add_parser('tri', help='Update trilateration anchor positions')
    tri_parser.add_argument('anchor1', type=str, help='Anchor 1 position (x,y,z) in centimeters')
    tri_parser.add_argument('anchor2', type=str, help='Anchor 2 position (x,y,z) in centimeters')
    tri_parser.add_argument('anchor3', type=str, help='Anchor 3 position (x,y,z) in centimeters')
    
    # Common options
    for p in [bi_parser, tri_parser]:
        p.add_argument('--rebuild', action='store_true', 
                       help='Rebuild the workspace after updating')
    
    # List current values command
    list_parser = subparsers.add_parser('list', help='List current anchor positions')
    
    # Add examples to the parser
    parser.epilog = '''
EXAMPLES:
  # List current anchor positions
  ./update_anchors.py list
  
  # Update bilateration anchor positions (2 anchors)
  ./update_anchors.py bi "0,0,90" "310,0,90" --reference_y 300 --positive_y --rebuild
  
  # Update trilateration anchor positions (3 anchors)
  ./update_anchors.py tri "0,0,90" "310,0,90" "250,600,90" --rebuild
  
NOTE:
  - Anchor positions are specified in centimeters as "x,y,z"
  - The reference_y parameter is important for bilateration as it helps resolve
    ambiguity when determining position using only two anchors.
  - With bilateration, two possible positions often exist (one on each side of the
    anchor line). The reference_y and prefer_positive_y options help select the
    correct one.
'''
    parser.formatter_class = argparse.RawDescriptionHelpFormatter
    
    return parser.parse_args()

def parse_position(pos_str):
    """Parse position string in format 'x,y,z' to a list of floats"""
    try:
        x, y, z = map(float, pos_str.split(','))
        return [x, y, z]
    except ValueError:
        print(f"Error: Position '{pos_str}' must be in format 'x,y,z'")
        sys.exit(1)

def update_launch_file(file_path, anchor_positions, reference_y=None, prefer_positive_y=None):
    """Update anchor positions in a launch file"""
    with open(file_path, 'r') as f:
        content = f.read()
    
    for i, pos in enumerate(anchor_positions, start=1):
        # Replace anchor position in the format 'anchor{i}_pos': [x, y, z]
        pattern = r"'anchor{}_pos':\s*\[\s*[\d.-]+\s*,\s*[\d.-]+\s*,\s*[\d.-]+\s*\]".format(i)
        replacement = f"'anchor{i}_pos': [{pos[0]}, {pos[1]}, {pos[2]}]"
        content = re.sub(pattern, replacement, content)
    
    if reference_y is not None:
        # Replace reference_y value
        pattern = r"'reference_y':\s*[\d.-]+"
        replacement = f"'reference_y': {reference_y}"
        content = re.sub(pattern, replacement, content)
    
    if prefer_positive_y is not None:
        # Replace prefer_positive_y value
        pattern = r"'prefer_positive_y':\s*(True|False)"
        replacement = f"'prefer_positive_y': {prefer_positive_y}"
        content = re.sub(pattern, replacement, content)
    
    with open(file_path, 'w') as f:
        f.write(content)
    
    print(f"Updated {file_path}")

def update_py_file(file_path, anchor_positions):
    """Update anchor positions in a Python file"""
    with open(file_path, 'r') as f:
        content = f.read()
    
    for i, pos in enumerate(anchor_positions, start=1):
        # Replace anchor position in the format ANCHOR_{i}_POSITION = (x, y, z)
        pattern = r"ANCHOR_{}_POSITION = \([\d.-]+, [\d.-]+, [\d.-]+\)".format(i)
        replacement = f"ANCHOR_{i}_POSITION = ({pos[0]}, {pos[1]}, {pos[2]})"
        content = re.sub(pattern, replacement, content)
    
    with open(file_path, 'w') as f:
        f.write(content)
    
    print(f"Updated {file_path}")

def read_current_positions(file_path, num_anchors):
    """Read current anchor positions from a file"""
    positions = []
    try:
        with open(file_path, 'r') as f:
            content = f.read()
            
        for i in range(1, num_anchors + 1):
            # Check for launch file format first
            pattern = r"'anchor{}_pos':\s*\[\s*([\d.-]+)\s*,\s*([\d.-]+)\s*,\s*([\d.-]+)\s*\]".format(i)
            match = re.search(pattern, content)
            if match:
                x, y, z = map(float, match.groups())
                positions.append([x, y, z])
                continue
                
            # Check for py file format
            pattern = r"ANCHOR_{}_POSITION = \(([\d.-]+), ([\d.-]+), ([\d.-]+)\)".format(i)
            match = re.search(pattern, content)
            if match:
                x, y, z = map(float, match.groups())
                positions.append([x, y, z])
        
        # For bilateration, also read reference_y and prefer_positive_y
        extras = {}
        if num_anchors == 2:
            pattern = r"'reference_y':\s*([\d.-]+)"
            match = re.search(pattern, content)
            if match:
                extras['reference_y'] = float(match.group(1))
                
            pattern = r"'prefer_positive_y':\s*(True|False)"
            match = re.search(pattern, content)
            if match:
                extras['prefer_positive_y'] = match.group(1) == "True"
        
        return positions, extras
    except Exception as e:
        print(f"Error reading positions from {file_path}: {e}")
        return [], {}

def list_current_positions():
    """List current anchor positions from all files"""
    print("\n=== CURRENT ANCHOR POSITIONS ===\n")
    
    # Bilateration
    print("Bilateration (2 anchors):")
    print("-------------------------")
    
    # Launch file
    bi_launch_positions, bi_launch_extras = read_current_positions(BILATERATION_LAUNCH_PATH, 2)
    if bi_launch_positions:
        print(f"  Launch file:")
        print(f"    Anchor 1: [{bi_launch_positions[0][0]}, {bi_launch_positions[0][1]}, {bi_launch_positions[0][2]}]")
        print(f"    Anchor 2: [{bi_launch_positions[1][0]}, {bi_launch_positions[1][1]}, {bi_launch_positions[1][2]}]")
        if 'reference_y' in bi_launch_extras:
            print(f"    Reference Y: {bi_launch_extras['reference_y']}")
        if 'prefer_positive_y' in bi_launch_extras:
            print(f"    Prefer positive Y: {bi_launch_extras['prefer_positive_y']}")
    
    # Python file
    bi_py_positions, _ = read_current_positions(BILATERATION_PY_PATH, 2)
    if bi_py_positions:
        print(f"  Python file defaults:")
        print(f"    Anchor 1: [{bi_py_positions[0][0]}, {bi_py_positions[0][1]}, {bi_py_positions[0][2]}]")
        print(f"    Anchor 2: [{bi_py_positions[1][0]}, {bi_py_positions[1][1]}, {bi_py_positions[1][2]}]")
    
    print("\nTrilateration (3 anchors):")
    print("-------------------------")
    
    # Launch file
    tri_launch_positions, _ = read_current_positions(TRILATERATION_LAUNCH_PATH, 3)
    if tri_launch_positions:
        print(f"  Launch file:")
        print(f"    Anchor 1: [{tri_launch_positions[0][0]}, {tri_launch_positions[0][1]}, {tri_launch_positions[0][2]}]")
        print(f"    Anchor 2: [{tri_launch_positions[1][0]}, {tri_launch_positions[1][1]}, {tri_launch_positions[1][2]}]")
        print(f"    Anchor 3: [{tri_launch_positions[2][0]}, {tri_launch_positions[2][1]}, {tri_launch_positions[2][2]}]")
    
    # Python file
    tri_py_positions, _ = read_current_positions(TRILATERATION_PY_PATH, 3)
    if tri_py_positions:
        print(f"  Python file defaults:")
        print(f"    Anchor 1: [{tri_py_positions[0][0]}, {tri_py_positions[0][1]}, {tri_py_positions[0][2]}]")
        print(f"    Anchor 2: [{tri_py_positions[1][0]}, {tri_py_positions[1][1]}, {tri_py_positions[1][2]}]")
        print(f"    Anchor 3: [{tri_py_positions[2][0]}, {tri_py_positions[2][1]}, {tri_py_positions[2][2]}]")
    
    print("\nNOTES:")
    print("- Launch file values take precedence over Python file defaults")
    print("  when the ROS nodes are launched.")
    print("- For bilateration (2 anchors), the reference_y parameter is critical")
    print("  for accurate positioning. It helps choose between two possible")
    print("  solutions when only two anchors are used.")
    print("- The reference_y should be set to approximately where you expect")
    print("  the robot to be relative to the anchor line. For example, if the")
    print("  anchors are along a wall (x-axis) and the robot moves in a hallway")
    print("  about 3 meters from the wall, set reference_y to 300 (cm).")
    print("- When --positive_y is set, the system prefers solutions where the")
    print("  robot is on the positive Y side of the anchor line.")
    print("\nEXAMPLES:")
    print("  # To update bilateration anchors with the default values shown above:")
    print("  ./update_anchors.py bi \"0,0,90\" \"310,0,90\" --reference_y 300 --positive_y --rebuild")
    print("\n  # To update trilateration anchors with the default values shown above:")
    print("  ./update_anchors.py tri \"0,0,90\" \"310,0,90\" \"250,600,90\" --rebuild\n")

def rebuild_workspace():
    """Rebuild the workspace after updating"""
    print("\nRebuilding workspace...")
    build_script = os.path.join(LIDAR_WS_PATH, "build.sh")
    
    if not os.path.exists(build_script):
        print(f"Error: Build script not found at {build_script}")
        return False
    
    # Run the build script
    result = os.system(f"cd {LIDAR_WS_PATH} && ./build.sh")
    
    if result == 0:
        print("Workspace rebuilt successfully!")
        return True
    else:
        print("Error rebuilding workspace.")
        return False

def show_anchor_visualization():
    """Show a visualization of anchor positioning to help with setup"""
    bilateration_viz = """
    Bilateration Setup (2 anchors):
    -------------------------------

       ^ Y+                        
       |                  
       |   Robot area   
       |   (reference_y)
       |                  
       |                  
    ---+---------------------> X+
       |                  
    A1 |                 A2
       |                  
    
    A1, A2 = Anchor positions

    - With just two anchors, there are typically two possible positions
      (one above and one below the X-axis)
    - reference_y helps select the most likely position
    - prefer_positive_y=True makes the system prefer solutions above the X-axis
    """

    trilateration_viz = """
    Trilateration Setup (3 anchors):
    --------------------------------

       ^ Y+                        
       |                  
       |                  
       |                  
       |                A3
       |                  
    ---+---------------------> X+
       |                  
    A1 |                 A2
       |                  
    
    A1, A2, A3 = Anchor positions

    - With three anchors, a unique position can be calculated
    - Ideally, anchors should form a triangle for best results
    - Typically, A1 and A2 define a baseline, and A3 provides the third point
    """

    print(bilateration_viz)
    print(trilateration_viz)

def main():
    args = parse_arguments()
    
    # Check if files exist
    for file_path in [BILATERATION_LAUNCH_PATH, TRILATERATION_LAUNCH_PATH, 
                      BILATERATION_PY_PATH, TRILATERATION_PY_PATH]:
        if not os.path.exists(file_path):
            print(f"Error: Required file {file_path} not found.")
            sys.exit(1)
            
    # If no command provided, show help and visualization
    if args.command is None:
        show_anchor_visualization()
        print("\nUse --help for usage information")
        return
    
    if args.command == 'list':
        list_current_positions()
        return
    
    if args.command == 'bi':
        # Update bilateration anchor positions
        anchor1_pos = parse_position(args.anchor1)
        anchor2_pos = parse_position(args.anchor2)
        
        print(f"\nUpdating bilateration anchor positions:")
        print(f"  Anchor 1: {anchor1_pos}")
        print(f"  Anchor 2: {anchor2_pos}")
        print(f"  Reference Y: {args.reference_y}")
        print(f"  Prefer positive Y: {args.positive_y}")
        
        # Update launch file
        update_launch_file(
            BILATERATION_LAUNCH_PATH, 
            [anchor1_pos, anchor2_pos], 
            args.reference_y, 
            args.positive_y
        )
        
        # Update Python file
        update_py_file(
            BILATERATION_PY_PATH, 
            [anchor1_pos, anchor2_pos]
        )
    
    elif args.command == 'tri':
        # Update trilateration anchor positions
        anchor1_pos = parse_position(args.anchor1)
        anchor2_pos = parse_position(args.anchor2)
        anchor3_pos = parse_position(args.anchor3)
        
        print(f"\nUpdating trilateration anchor positions:")
        print(f"  Anchor 1: {anchor1_pos}")
        print(f"  Anchor 2: {anchor2_pos}")
        print(f"  Anchor 3: {anchor3_pos}")
        
        # Update launch file
        update_launch_file(
            TRILATERATION_LAUNCH_PATH, 
            [anchor1_pos, anchor2_pos, anchor3_pos]
        )
        
        # Update Python file
        update_py_file(
            TRILATERATION_PY_PATH, 
            [anchor1_pos, anchor2_pos, anchor3_pos]
        )
    
    # Rebuild workspace if requested
    if hasattr(args, 'rebuild') and args.rebuild:
        rebuild_workspace()
    else:
        print("\nRemember to rebuild the workspace for changes to take effect:")
        print(f"cd {LIDAR_WS_PATH} && ./build.sh")
    
    print("\nDone!")

if __name__ == "__main__":
    main()