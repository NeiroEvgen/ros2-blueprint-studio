# Здесь хранятся готовые конфигурации узлов для обучения
# pkg: имя пакета ROS2
# exec: имя исполняемого файла
# desc: описание для подсказки

ROS_PRESETS = {
    # === РАЗДЕЛ 1: Черепашка (Turtlesim) ===
    "1. Turtlesim Node": {
        "pkg": "turtlesim", 
        "exec": "turtlesim_node",
        "desc": "Симулятор черепахи. (Окно не откроется, но логика работает)"
    },
    "2. Auto Draw Square": {
        "pkg": "turtlesim", 
        "exec": "draw_square",
        "desc": "Контроллер: заставляет черепаху рисовать квадрат"
    },

    # === РАЗДЕЛ 2: C++ Pub/Sub ===
    "3. C++ Talker": {
        "pkg": "demo_nodes_cpp", 
        "exec": "talker",
        "desc": "Публикует 'Hello World' (C++)"
    },
    "4. C++ Listener": {
        "pkg": "demo_nodes_cpp", 
        "exec": "listener",
        "desc": "Слушает сообщения (C++)"
    },

    # === РАЗДЕЛ 3: Python Pub/Sub ===
    "5. Py Talker": {
        "pkg": "demo_nodes_py", 
        "exec": "talker",
        "desc": "Публикует 'Hello World' (Python)"
    },
    "6. Py Listener": {
        "pkg": "demo_nodes_py", 
        "exec": "listener",
        "desc": "Слушает сообщения (Python)"
    },

    # === РАЗДЕЛ 4: Сервисы ===
    "7. Service Server": {
        "pkg": "demo_nodes_cpp", 
        "exec": "add_two_ints_server",
        "desc": "Сервер сложения двух чисел"
    },
    "8. Service Client": {
        "pkg": "demo_nodes_cpp", 
        "exec": "add_two_ints_client",
        "desc": "Клиент, отправляющий запрос на сервер"
    }
}