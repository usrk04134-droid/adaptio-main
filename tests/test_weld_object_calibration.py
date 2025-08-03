import time
from message import Receiver, Sender, LoopUntil
from message_factory import (WeldObjectCalibration, GetAdaptioVersion, SetJointGeometry)

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

    # WeldObject Calibration
    wo_cal_msg = WeldObjectCalibration()
    wo_cal_msg["payload"]["radius"] = 4000.
    wo_cal_msg["payload"]["stickout"] = 20

    sender.send(wo_cal_msg)

    # Wait for WeldObjectCalibrationRsp
    for _ in LoopUntil(10.0):
        if receiver.verify("WeldObjectCalibrationRsp",
                           lambda payload: payload["valid"] is True):
            assert True, "WeldObjectCalibrationRsp received"
            break
    else:
        assert False, "WeldObjectCalibrationRsp not received"

    time.sleep(1)
    assert True, "WeldObjectCalibration test completed"

    adaptio_app.quit(None)

    time.sleep(5)
