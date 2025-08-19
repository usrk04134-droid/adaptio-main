import time
from message import Receiver, Sender, LoopUntil
from message_factory import (LaserTorchCalGet, LaserTorchCalSet, GetAdaptioVersion)

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

    # V2 LaserTorch Calibration - Get current calibration
    sender.send(LaserTorchCalGet())
    
    # Wait for LaserTorchCalGetRsp
    for _ in LoopUntil(10.0):
        get_rsp = receiver.poll_specific("LaserTorchCalGetRsp")
        if get_rsp is not None:
            print("LaserTorchCalGet response: " + str(get_rsp["payload"]))
            break
        time.sleep(1)
    else:
        assert False, "LaserTorchCalGetRsp not received"

    # Set new laser torch calibration values
    set_msg = LaserTorchCalSet()
    set_msg["payload"]["distanceLaserTorch"] = 150.0
    set_msg["payload"]["stickout"] = 25.0
    set_msg["payload"]["scannerMountAngle"] = 0.26
    
    sender.send(set_msg)

    # Wait for LaserTorchCalSetRsp
    for _ in LoopUntil(10.0):
        set_rsp = receiver.poll_specific("LaserTorchCalSetRsp")
        if set_rsp is not None and set_rsp["payload"]["result"] == "ok":
            print("LaserTorchCalSet successful")
            break
        time.sleep(1)
    else:
        assert False, "LaserTorchCalSetRsp not received or failed"

    # Verify the calibration was set by getting it again
    sender.send(LaserTorchCalGet())
    
    for _ in LoopUntil(10.0):
        verify_rsp = receiver.poll_specific("LaserTorchCalGetRsp")
        if verify_rsp is not None:
            payload = verify_rsp["payload"]
            if (payload["result"] == "ok" and 
                payload["distanceLaserTorch"] == 150.0 and
                payload["stickout"] == 25.0 and
                abs(payload["scannerMountAngle"] - 0.26) < 0.001):
                print("LaserTorch calibration verified successfully")
                break
        time.sleep(1)
    else:
        assert False, "LaserTorch calibration verification failed"

    time.sleep(1)
    assert True, "LaserTorchCalibration V2 test completed"

    adaptio_app.quit(None)

    time.sleep(5)
