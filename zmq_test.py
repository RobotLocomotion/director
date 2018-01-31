import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")
print("connected")

data = {
        "timestamp": 1486691399249288,
        "setgeometry": [
                {
                        "path": ["robot1", "link1"],
                        "geometry": {
                                "type": "box",
                                "color": [1, 0, 0, 0.5],
                                "lengths": [1, 0.5, 2]
                        }
                }
        ],
        "settransform": [],
        "delete": []
}
print("sending")
socket.send(json.dumps(data))
print("waiting for reply")
print(socket.recv())
