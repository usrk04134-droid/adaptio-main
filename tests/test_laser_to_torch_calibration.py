import time
from message import Receiver, Sender, LoopUntil
from message_factory import (LaserToTorchCalibration, GetAdaptioVersion)

def test_calibration(zmq_sockets, adaptio_app, config_file):
    time.sleep(1)

    request_socket, reply_socket = zmq_sockets

    receive_timeout_ms = 200
    receiver = Receiver(reply_socket, receive_timeout_ms)
    sender = Sender(request_socket)

    adaptio_app.run(["--config-file", "tests/configs/sil/configuration_laser_to_torch.yaml"])

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

    # LaserToTorch Calibration
    ltt_cal_msg = LaserToTorchCalibration()
    ltt_cal_msg["payload"]["offset"] = 5.
    ltt_cal_msg["payload"]["angle"] = 0.5
    ltt_cal_msg["payload"]["stickout"] = 20

    sender.send(ltt_cal_msg)

    # Wait for LaserToTorchCalibrationRsp
    for _ in LoopUntil(10.0):
        if receiver.verify("LaserToTorchCalibrationRsp",
                           lambda payload: payload["valid"] is True):
            assert True, "LaserToTorchCalibrationRsp received"
            break
    else:
        assert False, "LaserToTorchCalibrationRsp not received"

    time.sleep(1)
    assert True, "LaserToTorchCalibration test completed"

    adaptio_app.quit(None)

    time.sleep(5)
