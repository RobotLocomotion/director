import os
import subprocess
import zmq
import msgpack

if __name__ == '__main__':
    vis_binary = os.path.join(os.path.dirname(sys.executable),
                              "drake-visualizer")
    vis_process = subprocess.Popen([vis_binary, '--testing', '--interactive', '--treeviewer-zmq-url=tcp://127.0.0.1:56300'])
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://127.0.0.1:56300")
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
    socket.send(msgpack.packb(data))
    print("waiting for reply")
    print(socket.recv())

    vis_process.terminate()
