import time
from message import Receiver, Sender, LoopUntil
from message_factory import (GetAdaptioVersion)


def test_calibration(zmq_sockets, adaptio_app, config_file):
    time.sleep(1)

    request_socket, reply_socket = zmq_sockets

    receive_timeout_ms = 200
    receiver = Receiver(reply_socket, receive_timeout_ms)
    sender = Sender(request_socket)

    adaptio_app.run(["--config-file", config_file])

    # Test unstable without a sleep here
    time.sleep(10)

    # Check adaptio started by print adaptio version
    for _ in LoopUntil(15.0):
        sender.send(GetAdaptioVersion())
        version_rsp = receiver.poll_specific("GetAdaptioVersionRsp")
        if version_rsp is not None:
            print("Adaptio version: " + version_rsp["payload"]["version"])
            break
        time.sleep(1)
    else:
        assert False, "GetAdaptioVersion not received"

    # Set LaserTorch configuration (v2)
    sender.send({"name": "LaserTorchCalSet", "payload": {"distanceLaserTorch": 150.0, "stickout": 25.0, "scannerMountAngle": 0.26}})
    for _ in LoopUntil(5.0):
        if receiver.verify("LaserTorchCalSetRsp", lambda payload: payload["result"] == "ok"):
            break
    else:
        assert False, "LaserTorchCalSetRsp not received"

    # Start WeldObject Calibration (v2)
    sender.send({"name": "WeldObjectCalStart", "payload": {"wireDiameter": 4.0, "stickout": 25.0, "weldObjectRadius": 4000.0}})
    for _ in LoopUntil(5.0):
        if receiver.verify("WeldObjectCalStartRsp", lambda payload: payload["result"] == "ok"):
            break
    else:
        assert False, "WeldObjectCalStartRsp not received"

    # Signal left and right positions
    sender.send({"name": "WeldObjectCalLeftPos", "payload": {}})
    for _ in LoopUntil(5.0):
        if receiver.verify("WeldObjectCalLeftPosRsp", lambda payload: payload["result"] in ["ok", "fail"]):
            break
    else:
        assert False, "WeldObjectCalLeftPosRsp not received"

    sender.send({"name": "WeldObjectCalRightPos", "payload": {}})
    for _ in LoopUntil(5.0):
        if receiver.verify("WeldObjectCalRightPosRsp", lambda payload: payload["result"] in ["ok", "fail"]):
            break
    else:
        assert False, "WeldObjectCalRightPosRsp not received"

    # Either we get an in-progress push or a final result; check GET for stored result
    sender.send({"name": "WeldObjectCalGet", "payload": {}})
    for _ in LoopUntil(10.0):
        rsp = receiver.poll_specific("WeldObjectCalGetRsp")
        if rsp is not None:
            assert rsp["payload"].get("result") in ["ok", "fail"]
            break
        time.sleep(0.5)
    else:
        assert False, "WeldObjectCalGetRsp not received"

    adaptio_app.quit(None)

    time.sleep(5)
