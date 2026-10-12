import glob
import os
import xml.etree.ElementTree as ET

def gpx_waypoints_to_yaml(gpx_file_path):
    # Should work with any kind of formatting, as long as it uses the <wpt> tags to convey lat, lon.

    root = ET.parse(gpx_file_path).getroot()

    wpts = [el for el in root.iter() if el.tag.split("}")[-1] == "wpt"]

    name = os.path.basename(gpx_file_path)[:-4] + '_gpx'

    lines = [f"name: {name}", "waypoints:"]
    for i, wpt in enumerate(wpts, start=1):
        lat = float(wpt.attrib["lat"])
        lon = float(wpt.attrib["lon"])
        key = f"waypoint_{i}:"
        lines.append(f"  {key:<14}{{lat: {lat}, lon: {lon}}}")
    return "\n".join(lines) + "\n"

def gpx_to_yaml(gpx_file_path, yaml_folder_path):

    if not os.path.exists(gpx_file_path):
        print(f'File {gpx_file_path} does not exist. Cannot transform it to yaml.')
        return

    yaml_string = gpx_waypoints_to_yaml(gpx_file_path)

    yaml_file_name = os.path.join(yaml_folder_path, os.path.basename(gpx_file_path)[:-4] + '_gpx.yaml')

    if os.path.exists(yaml_file_name):
        print(f"File {yaml_file_name} already exists! Will not overwrite, if this is unexpected, please resolve manually!")
        return

    with open(yaml_file_name, "w") as f:
        f.write(yaml_string)

def gpx_to_yaml_all(gpx_folder_path, yaml_folder_path):
    
    gpx_files = glob.glob(os.path.join(gpx_folder_path, '*.gpx'))
    for gpx_file in gpx_files:
        gpx_to_yaml(gpx_file, yaml_folder_path)

if __name__ == "__main__":
    gpx_to_yaml("/home/herolada/ws_k/src/commander_operator/commander_operator/data/gpx/example.gpx",\
                "/home/herolada/ws_k/src/commander_operator/commander_operator/data/routes_wgs")

    gpx_to_yaml_all("/home/herolada/ws_k/src/commander_operator/commander_operator/data/gpx",
                    "/home/herolada/ws_k/src/commander_operator/commander_operator/data/routes_wgs")