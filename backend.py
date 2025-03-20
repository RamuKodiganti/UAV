from flask import Flask, jsonify, request
from flask_cors import CORS
import numpy as np
import math
import random

# Import the updated 3D algorithm
from uav_path_planning import RRTPlanner  # Assuming uav_path_planning is updated below

app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

@app.route('/plan_path', methods=['POST'])
def plan_path():
    data = request.get_json()
    start = data['start']  # Now [x, y, z]
    goal = data['goal']    # Now [x, y, z]
    obstacles = data['obstacles']  # Updated for 3D obstacles
    bounds = data['bounds']  # Now includes min_z, max_z

    planner = RRTPlanner()
    path = planner.find_path(start, goal, obstacles, bounds)

    if path:
        return jsonify({'path': path})  # Path is now list of [x, y, z]
    else:
        return jsonify({'path': []})

if __name__ == '__main__':
    app.run(debug=False)