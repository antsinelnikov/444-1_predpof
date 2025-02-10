import cv2
import numpy as np
import serial
import pygame
import time

# Инициализация Serial-порта для Raspberry Pi
ser = serial.Serial('/dev/serial0', 9600, timeout=1)  # Используйте /dev/serial0 или /dev/ttyAMA0
time.sleep(2)  # Ожидание инициализации Serial

# Инициализация Pygame для работы с джойстиком
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)  # Подключение первого джойстика
joystick.init()

# Параметры камеры (замените на реальные значения после калибровки)
mtx = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=np.float32)  # Матрица камеры
dist = np.array([-0.2, 0.1, 0.01, 0.01, 0], dtype=np.float32)  # Коэффициенты искажения

# Размер маркера (в метрах)
marker_size = 0.15

# Инициализация видеозахвата
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Ошибка: Камера не подключена!")
    exit()

# Функция для вычисления скоростей
def calculate_speed(center_x, center_y, frame_center_x, frame_center_y, distance, target_distance):
    # Пропорциональные коэффициенты для управления скоростью
    kp_linear = 0.5  # Коэффициент для линейной скорости
    kp_angular = 0.1  # Коэффициент для угловой скорости

    # Ошибка по расстоянию
    distance_error = distance - target_distance

    # Ошибка по положению
    x_error = center_x - frame_center_x
    y_error = center_y - frame_center_y

    # Линейная скорость (вперед/назад)
    linear_speed = -kp_linear * distance_error

    # Угловая скорость (влево/вправо)
    angular_speed_x = kp_angular * x_error
    angular_speed_y = kp_angular * y_error

    return linear_speed, angular_speed_x, angular_speed_y

# Функция для поиска теннисного мяча
def find_tennis_ball(frame):
    # Преобразование изображения в HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Определение диапазона желтого цвета в HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Создание маски для желтого цвета
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Поиск контуров
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Поиск кругов среди контуров
    for contour in contours:
        # Аппроксимация контура
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        if len(approx) > 8:  # Если контур похож на круг
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius > 10:  # Игнорируем маленькие круги
                return (int(x), int(y)), int(radius)
    return None, None

# Основной цикл
while True:
    # Чтение данных с кнопок джойстика
    pygame.event.pump()  # Обновление событий джойстика

    button_up = joystick.get_button(11)  # Вверх
    button_down = joystick.get_button(12)  # Вниз
    button_left = joystick.get_button(13)  # Влево
    button_right = joystick.get_button(14)  # Вправо

    if button_up:
        target_distance = 0.615  
    elif button_down:
        target_distance = 0.580  
    elif button_left:
        target_distance = 0.5  
    elif button_right:
        target_distance = 0.4  
    else:
        target_distance = 0.580  # По умолчанию

    #кадр с камеры
    ret, frame = cap.read()
    if not ret:
        print("Ошибка: Не удалось получить кадр!")
        break

    #Центр кадра
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2

    #Поиск аруко маркеров
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is not None:
        #определение положения маркеров
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)

        for i in range(len(ids)):
            cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.1)


            distance = np.linalg.norm(tvecs[i])
            print(f"Расстояние до маркера {ids[i]}: {distance:.2f} метров")

            marker_center_x = int(corners[i][0][:, 0].mean())
            marker_center_y = int(corners[i][0][:, 1].mean())

            #Вычисление скоростей для центрирования перед маркером
            linear_speed, angular_speed_x, angular_speed_y = calculate_speed(
                marker_center_x, marker_center_y, frame_center_x, frame_center_y, distance, target_distance
            )

            ser.write(f"{linear_speed:.2f},{angular_speed_x:.2f}\n".encode())

            #теннисный мяч
            if abs(linear_speed) < 0.1 and abs(angular_speed_x) < 0.1 and abs(angular_speed_y) < 0.1:
                print("Выравнивание завершено. Поиск теннисного мяча...")
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        print("Ошибка: Не удалось получить кадр!")
                        break

                    # Поиск теннисного мяча
                    (x, y), radius = find_tennis_ball(frame)
                    if x is not None and y is not None:
                        cv2.circle(frame, (x, y), radius, (0, 255, 0), 2)

                        distance_to_ball = 1.0 / radius

                        linear_speed, angular_speed_x, _ = calculate_speed(
                            x, y, frame_center_x, frame_center_y, distance_to_ball, 0.5  
                        )

                        ser.write(f"{linear_speed:.2f},{angular_speed_x:.2f}\n".encode())

                        if distance_to_ball < 0.06:
                            print("Мяч найден. Ожидание команд с джойстика.")
                            break

                    cv2.imshow('Frame', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

    #Отображение кадра для проверки
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
pygame.quit()
