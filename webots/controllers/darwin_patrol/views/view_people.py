# views/view_people.py
import cv2
from views.view import View

# Variable global para recordar el último conteo
_last_count = None

def view_people(view: View):
    """Detecta personas y solo imprime cuando cambia el número detectado."""
    global _last_count

    frame = view.get_frame()
    if frame is None:
        num_pedestrians = 0
    else:
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        rects, _ = hog.detectMultiScale(frame,
                                        winStride=(4,4),
                                        padding=(8,8),
                                        scale=1.05)
        num_pedestrians = len(rects)

    # Solo imprimir si hay cambio respecto al último conteo
    if num_pedestrians != _last_count:
        print(f"[vision] personas vistas: {num_pedestrians}")
        _last_count = num_pedestrians

    return num_pedestrians
