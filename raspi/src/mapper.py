import json
import os

MAP_FILE = os.path.join(os.path.dirname(__file__), "..", "data", "map_data.json")

class ArenaMap:
    def __init__(self):
        self.data = {
            "home": {"x": 0, "y": 0},
            "water_station": None,
            "zones": {}
        }
        self.load()

    def load(self):
        if os.path.exists(MAP_FILE):
            with open(MAP_FILE, 'r') as f:
                self.data = json.load(f)
                print("[MAP] Loaded existing map")
        else:
            os.makedirs(os.path.dirname(MAP_FILE), exist_ok=True)
            self.save()
            print("[MAP] Created new map")

    def save(self):
        with open(MAP_FILE, 'w') as f:
            json.dump(self.data, f, indent=4)
        print("[MAP] Saved data")

    def update_zone(self, color, x, y):
        self.data["zones"][color] = {"x": x, "y": y}
        self.save()

    def update_station(self, x, y):
        self.data["water_station"] = {"x": x, "y": y}
        self.save()
