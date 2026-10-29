"""Small recorded-camera window; works with headless OpenCV builds."""

from contextlib import suppress

import numpy as np


class CameraWindow:
    def __init__(self):
        try:
            import tkinter as tk

            from PIL import Image, ImageTk
        except ImportError as exc:
            raise RuntimeError("Camera window unavailable; install python3-tk or use --no-camera") from exc
        self.tk, self.image, self.image_tk = tk, Image, ImageTk
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError("Camera window unavailable; use --headless or --no-camera") from exc
        self.root.title("Recorded head camera (q: stop)")
        self.open = True
        self.label = self.tk.Label(self.root)
        self.label.pack()
        self.root.bind("q", lambda _: self._request_close())
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)
        self.current = None

    def _request_close(self):
        self.open = False

    def show(self, image):
        if not self.open:
            return False
        rgb = np.clip(np.asarray(image) * 255, 0, 255).astype(np.uint8)
        self.current = self.image_tk.PhotoImage(self.image.fromarray(rgb))
        self.label.configure(image=self.current)
        try:
            self.root.update_idletasks()
            self.root.update()
        except self.tk.TclError:
            self.open = False
        return self.open

    def close(self):
        with suppress(self.tk.TclError):
            self.root.destroy()
        self.open = False
