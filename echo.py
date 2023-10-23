# Copyright 2022 Cartesi Pte. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use
# this file except in compliance with the License. You may obtain a copy of the
# License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.

from os import environ
import logging
import requests
import pybullet as p
import time
import pybullet_data
import json


logging.basicConfig(level="INFO")
logger = logging.getLogger(__name__)

rollup_server = environ["ROLLUP_HTTP_SERVER_URL"]
logger.info(f"HTTP rollup_server url is {rollup_server}")

# Initialize physics
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)

# Create a plane collision shape
planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)

# Create a multi-body object for the plane with the collision shape, at position [0,0,0]
planeId = p.createMultiBody(baseMass=0,  # setting mass to 0 makes it static
                            baseCollisionShapeIndex=planeShape,
                            basePosition=[0, 0, 0])

startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
def hex2str(hex):
    """
    Decodes a hex string into a regular string
    """
    return bytes.fromhex(hex[2:]).decode("utf-8")

def handle_advance(data):
    logger.info(f"Received advance request data {data}")
    logger.info("Adding notice")
    notice = {"payload": data["payload"]}
    response = requests.post(rollup_server + "/notice", json=notice)
    logger.info(f"Received notice status {response.status_code} body {response.content}")

    # process input
    input_data = json.loads(hex2str(data["payload"]))  # Changed json.load to json.loads
    action = input_data["action"]
    if action == "add":
        start_position = input_data["start_position"]
        velocity = input_data["velocity"]
        sphere_mass = input_data.get("mass", 1.0)  # Default to 1.0 if mass is not provided
        sphere_radius = input_data.get("radius", 0.1)  # Default to 0.1 if radius is not provided

        # Add a sphere to the simulation
        sphere_collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=sphere_radius)
        sphere_id = p.createMultiBody(baseMass=sphere_mass,
                                    baseCollisionShapeIndex=sphere_collision_shape_id,
                                    basePosition=start_position)
        p.resetBaseVelocity(sphere_id, linearVelocity=velocity)
    else:
        return "reject"

    # Run simulation for 30 seconds
    for i in range(int(5 * 240)):
        p.stepSimulation()
        time.sleep(1./240.)

    return "accept"

def handle_inspect(data):
    logger.info(f"Received inspect request data {data}")
    logger.info("Adding report")
    report = {"payload": data["payload"]}
    response = requests.post(rollup_server + "/report", json=report)
    logger.info(f"Received report status {response.status_code}")
    return "accept"

handlers = {
    "advance_state": handle_advance,
    "inspect_state": handle_inspect,
}

finish = {"status": "accept"}

while True:
    logger.info("Sending finish")
    response = requests.post(rollup_server + "/finish", json=finish)
    logger.info(f"Received finish status {response.status_code}")
    if response.status_code == 202:
        logger.info("No pending rollup request, trying again")
    else:
        rollup_request = response.json()
        data = rollup_request["data"]
        
        handler = handlers[rollup_request["request_type"]]
        finish["status"] = handler(rollup_request["data"])
