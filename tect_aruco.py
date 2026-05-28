import cv2

# Загружаем предопределенный словарь ArUco-маркеров
# В данном случае используется сетка 6x6 и словарь из 250 уникальных маркеров
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# Инициализируем детектор (актуально для OpenCV 4.7+)
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Подключаемся к первой камере (индекс 0)
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
cap.set(cv2.CAP_PROP_FPS,30)
cap.set(cv2.CAP_PROP_EXPOSURE,-7)



while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Конвертируем кадр в градации серого для анализа
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Детектируем маркеры: получаем углы, ID и отклоненные кандидаты
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    # Если маркеры найдены, рисуем их контуры и ID
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Выводим найденные ID в консоль
        for i in range(len(ids)):
            print(f"Обнаружен маркер с ID: {ids[i][0]}")

    # Отображаем видеопоток
    cv2.imshow('ArUco Detection', frame)

    # Нажмите 'q' для выхода
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()