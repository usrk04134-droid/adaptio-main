import time
from message import Receiver, Sender, LoopUntil
from message_factory import (WeldObjectCalStart, WeldObjectCalLeftPos, WeldObjectCalRightPos, 
                           WeldObjectCalGet, WeldObjectCalSet, WeldObjectCalStop,
                           GetAdaptioVersion, SetJointGeometry)

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

    # Set joint geometry first
    for _ in LoopUntil(15.0):
        sender.send(SetJointGeometry())
        response_rsp = receiver.poll_specific("SetJointGeometryRsp")
        if response_rsp is not None:
            print("Joint Geometry: " + response_rsp["payload"]["result"])
            break
        time.sleep(1)
    else:
        assert False, "SetJointGeometry failed"

    # V2 WeldObject Calibration - Start calibration
    start_msg = WeldObjectCalStart()
    start_msg["payload"]["wireDiameter"] = 4.0
    start_msg["payload"]["stickout"] = 25.0
    start_msg["payload"]["weldObjectRadius"] = 7000.0
    
    sender.send(start_msg)

    # Wait for WeldObjectCalStartRsp
    for _ in LoopUntil(10.0):
        start_rsp = receiver.poll_specific("WeldObjectCalStartRsp")
        if start_rsp is not None and start_rsp["payload"]["result"] == "ok":
            print("WeldObjectCalStart successful")
            break
        time.sleep(1)
    else:
        assert False, "WeldObjectCalStartRsp not received or failed"

    # Simulate left wall touch
    sender.send(WeldObjectCalLeftPos())
    
    # Wait for WeldObjectCalLeftPosRsp
    for _ in LoopUntil(10.0):
        left_rsp = receiver.poll_specific("WeldObjectCalLeftPosRsp")
        if left_rsp is not None and left_rsp["payload"]["result"] == "ok":
            print("WeldObjectCalLeftPos successful")
            break
        time.sleep(1)
    else:
        assert False, "WeldObjectCalLeftPosRsp not received or failed"

    # Simulate right wall touch
    sender.send(WeldObjectCalRightPos())
    
    # Wait for WeldObjectCalRightPosRsp
    for _ in LoopUntil(10.0):
        right_rsp = receiver.poll_specific("WeldObjectCalRightPosRsp")
        if right_rsp is not None and right_rsp["payload"]["result"] == "ok":
            print("WeldObjectCalRightPos successful")
            break
        time.sleep(1)
    else:
        assert False, "WeldObjectCalRightPosRsp not received or failed"

    # Wait for automatic calibration result (this is a push message)
    print("Waiting for calibration calculation to complete...")
    for _ in LoopUntil(30.0):  # Longer timeout for calibration calculation
        result_msg = receiver.poll_specific("WeldObjectCalResult")
        if result_msg is not None:
            payload = result_msg["payload"]
            if payload["result"] == "ok":
                print("WeldObjectCalResult successful: " + str(payload))
                
                # Set the calibration result
                set_msg = WeldObjectCalSet()
                set_msg["payload"] = {
                    "rotationCenter": payload["calibrationCenter"],
                    "torchToLpcsTranslation": {"c1": 0.0, "c2": 355.1, "c3": 23.1},  # Default values
                    "weldObjectRotationAxis": {"c1": 0.0, "c2": 0.0, "c3": 1.0}  # Default values
                }
                
                sender.send(set_msg)
                
                # Wait for WeldObjectCalSetRsp
                for _ in LoopUntil(10.0):
                    set_rsp = receiver.poll_specific("WeldObjectCalSetRsp")
                    if set_rsp is not None and set_rsp["payload"]["result"] == "ok":
                        print("WeldObjectCalSet successful")
                        break
                    time.sleep(1)
                else:
                    assert False, "WeldObjectCalSetRsp not received or failed"
                
                break
            else:
                print("WeldObjectCalResult failed: " + str(payload))
                assert False, "Calibration calculation failed"
        time.sleep(1)
    else:
        assert False, "WeldObjectCalResult not received"

    # Verify calibration by getting it
    sender.send(WeldObjectCalGet())
    
    for _ in LoopUntil(10.0):
        get_rsp = receiver.poll_specific("WeldObjectCalGetRsp")
        if get_rsp is not None and get_rsp["payload"]["result"] == "ok":
            print("WeldObjectCalGet verification successful: " + str(get_rsp["payload"]))
            break
        time.sleep(1)
    else:
        assert False, "WeldObjectCalGetRsp not received or failed"

    time.sleep(1)
    assert True, "WeldObjectCalibration V2 test completed"

    adaptio_app.quit(None)

    time.sleep(5)
