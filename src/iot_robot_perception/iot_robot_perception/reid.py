"""Appearance embeddings for recognising the same person again (OSNet ReID)."""

from collections import deque

import cv2
import numpy as np
import onnxruntime as ort

# ImageNet statistics (RGB) that the OSNet weights were trained with
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


class ReidEncoder:
    """Turn a person crop into a unit-length appearance vector."""

    def __init__(self, model_path, num_threads):
        options = ort.SessionOptions()
        options.intra_op_num_threads = num_threads
        self.session = ort.InferenceSession(
            model_path, options, providers=["CPUExecutionProvider"])
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        # Shape is (batch, channels, height, width), 256 x 128 for OSNet
        self.height, self.width = model_input.shape[2:]

    def embed(self, bgr, box):
        """Return the embedding of the person inside box, or None if too small."""
        h, w = bgr.shape[:2]
        x1, x2 = np.clip(box[[0, 2]].round().astype(int), 0, w)
        y1, y2 = np.clip(box[[1, 3]].round().astype(int), 0, h)
        if x2 - x1 < 8 or y2 - y1 < 16:
            return None
        # A plain stretch to 128 x 256, the same preprocessing used in training
        crop = cv2.resize(bgr[y1:y2, x1:x2], (self.width, self.height),
                          interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = ((rgb - MEAN) / STD).transpose(2, 0, 1)[None]
        feature = self.session.run(None, {self.input_name: blob})[0][0]
        return feature / np.linalg.norm(feature)


class Gallery:
    """Embeddings of the person to follow."""

    def __init__(self, recent_size):
        # Captured at enrollment and kept, so the gallery cannot drift to someone else
        self.enrolled = []
        # Refreshed while following, to cope with new angles and lighting
        self.recent = deque(maxlen=recent_size)

    def __len__(self):
        return len(self.enrolled)

    def enroll(self, embeddings):
        self.enrolled = list(embeddings)
        self.recent.clear()

    def clear(self):
        self.enroll([])

    def add(self, embedding):
        self.recent.append(embedding)

    def similarity(self, embedding):
        """Cosine similarity (-1 to 1) to the closest stored embedding."""
        # All vectors have unit length, so the dot product is the cosine
        stored = np.array([*self.enrolled, *self.recent])
        return float((stored @ embedding).max())
