import time
from message import Receiver, Sender, LoopUntil
from message_factory import (GetAdaptioVersion, SetJointGeometry, WeldObjectCalStart, WeldObjectCalLeftPos, WeldObjectCalRightPos, WeldObjectCalGet)

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

    for _ in LoopUntil(15.0):
        sender.send(SetJointGeometry())
        response_rsp = receiver.poll_specific("SetJointGeometryRsp")
        if response_rsp is not None:
            print("Join Geometry: " + response_rsp["payload"]["result"])
            break
        time.sleep(1)
    else:
        assert False, "SetJointGeometry failed"

    # Start calibration (v2)
    start_msg = WeldObjectCalStart()
    start_msg["payload"]["wireDiameter"] = 4.0
    start_msg["payload"]["stickout"] = 25.0
    start_msg["payload"]["weldObjectRadius"] = 4000.0
    sender.send(start_msg)

    for _ in LoopUntil(5.0):
        if receiver.verify("WeldObjectCalStartRsp", lambda payload: payload["result"] == "ok"):
            break
    else:
        assert False, "WeldObjectCalStartRsp not received"

    sender.send(WeldObjectCalLeftPos())
    for _ in LoopUntil(5.0):
        if receiver.verify("WeldObjectCalLeftPosRsp", lambda payload: payload["result"] == "ok"):
            break
    else:
        assert False, "WeldObjectCalLeftPosRsp not received"

    sender.send(WeldObjectCalRightPos())
    for _ in LoopUntil(5.0):
        if receiver.verify("WeldObjectCalRightPosRsp", lambda payload: payload["result"] == "ok"):
            break
    else:
        assert False, "WeldObjectCalRightPosRsp not received"

    sender.send(WeldObjectCalGet())
    for _ in LoopUntil(5.0):
        rsp = receiver.poll_specific("WeldObjectCalGetRsp")
        if rsp is not None:
            payload = rsp["payload"]
            assert payload["result"] == "ok"
            break
    else:
        assert False, "WeldObjectCalGetRsp not received"

    time.sleep(1)
    adaptio_app.quit(None)
    time.sleep(5)
