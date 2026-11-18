"""ONNX Runtime sessions for a robot computer that runs many nodes on a few cores."""

import onnxruntime as ort


def onnx_session(model_path, num_threads, spinning=False):
    """A CPU inference session that uses `num_threads` threads for each run.

    By default ONNX Runtime's worker threads keep spinning for a while after every run,
    waiting for the next one. On a Raspberry Pi 4 that runs the camera, the motor
    controllers and the web page too, those are CPU cycles the other nodes need between
    camera frames. `spinning` False lets the threads sleep instead.
    """
    options = ort.SessionOptions()
    options.intra_op_num_threads = num_threads
    options.add_session_config_entry(
        "session.intra_op.allow_spinning", "1" if spinning else "0")
    return ort.InferenceSession(model_path, options, providers=["CPUExecutionProvider"])
