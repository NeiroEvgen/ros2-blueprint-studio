import cv2, socket, struct, math
import mediapipe as mp

UDP_ADDR = ("127.0.0.1", 9877)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)   # CAP_DSHOW: быстрый старт вебки на Windows

with mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.6,
                    min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)          # зеркало: движение вправо = вправо
        res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if res.multi_hand_landmarks:
            lm = res.multi_hand_landmarks[0].landmark
            wrist, mid = lm[0], lm[12]      # запястье и кончик среднего
            x, y = wrist.x, wrist.y
            tilt = math.atan2(mid.x - wrist.x, -(mid.y - wrist.y))
            # grip: среднее расстояние кончиков пальцев до запястья (нормированное)
            tips = [lm[i] for i in (4, 8, 12, 16, 20)]
            spread = sum(math.dist((t.x, t.y), (wrist.x, wrist.y)) for t in tips) / 5
            grip = max(0.0, min(1.0, (spread - 0.15) / 0.25))

            # 4 double: x, y, tilt, grip  (little-endian, 32 байта)
            sock.sendto(struct.pack("<4d", x, y, tilt, grip), UDP_ADDR)
            mp_draw.draw_landmarks(frame, res.multi_hand_landmarks[0],
                                   mp_hands.HAND_CONNECTIONS)

        cv2.imshow("Hand Tracker (ESC to quit)", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()