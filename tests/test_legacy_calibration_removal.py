import time
from message import Receiver, Sender, LoopUntil
from message_factory import (LaserToTorchCalibration, WeldObjectCalibration, GetAdaptioVersion)

def test_legacy_calibration_removal(zmq_sockets, adaptio_app, config_file):
    """Test that legacy calibration methods return failure responses"""
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

    # Test legacy LaserToTorchCalibration - should fail
    ltt_cal_msg = LaserToTorchCalibration()
    ltt_cal_msg["payload"]["offset"] = 5.0
    ltt_cal_msg["payload"]["angle"] = 0.5
    ltt_cal_msg["payload"]["stickout"] = 20

    sender.send(ltt_cal_msg)

    # Wait for LaserToTorchCalibrationRsp - should return valid=false
    for _ in LoopUntil(10.0):
        ltt_rsp = receiver.poll_specific("LaserToTorchCalibrationRsp")
        if ltt_rsp is not None:
            if ltt_rsp["payload"]["valid"] is False:
                print("Legacy LaserToTorchCalibration correctly returned failure")
            else:
                assert False, "Legacy LaserToTorchCalibration should have failed but returned valid=true"
            break
        time.sleep(1)
    else:
        assert False, "LaserToTorchCalibrationRsp not received"

    # Test legacy WeldObjectCalibration - should fail
    wo_cal_msg = WeldObjectCalibration()
    wo_cal_msg["payload"]["radius"] = 4000.0
    wo_cal_msg["payload"]["stickout"] = 20

    sender.send(wo_cal_msg)

    # Wait for WeldObjectCalibrationRsp - should return valid=false
    for _ in LoopUntil(10.0):
        wo_rsp = receiver.poll_specific("WeldObjectCalibrationRsp")
        if wo_rsp is not None:
            if wo_rsp["payload"]["valid"] is False:
                print("Legacy WeldObjectCalibration correctly returned failure")
            else:
                assert False, "Legacy WeldObjectCalibration should have failed but returned valid=true"
            break
        time.sleep(1)
    else:
        assert False, "WeldObjectCalibrationRsp not received"

    time.sleep(1)
    assert True, "Legacy calibration removal test completed successfully"

    adaptio_app.quit(None)

    time.sleep(5)