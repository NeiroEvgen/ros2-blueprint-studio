from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, 
                             QTableWidgetItem, QPushButton, QHeaderView, QLabel, QMessageBox)
from PySide6.QtCore import Qt, QTimer

class DockerPanel(QWidget):
    def __init__(self, docker_manager):
        super().__init__()
        self.docker_manager = docker_manager
        self.init_ui()
        
        # Таймер для автообновления списка контейнеров
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_containers)
        self.timer.start(8000) # Обновление каждые 3 секунды

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Заголовок и кнопка обновления
        top_layout = QHBoxLayout()
        self.label = QLabel("Docker Containers")
        self.label.setStyleSheet("font-weight: bold; font-size: 14px;")
        
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_containers)
        
        top_layout.addWidget(self.label)
        top_layout.addStretch()
        top_layout.addWidget(self.refresh_btn)
        layout.addLayout(top_layout)
        
        # Таблица контейнеров
        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["Name", "Image", "Status", "Network", "Actions"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        layout.addWidget(self.table)
        
        self.refresh_containers()

    def refresh_containers(self):
        try:
            containers = self.docker_manager.list_containers()
            self.table.setRowCount(len(containers))
            
            for i, container in enumerate(containers):
                # Имя (убираем начальный слэш)
                name = container.name
                self.table.setItem(i, 0, QTableWidgetItem(name))
                
                # Образ
                try:
                    image = container.attrs.get('Config', {}).get('Image', '')
                    if not image:
                        image = container.attrs.get('Image', '')[:19]  # sha256:xxxx
                except Exception:
                    image = "unknown"
                self.table.setItem(i, 1, QTableWidgetItem(image))
                
                # Статус
                status = container.status
                status_item = QTableWidgetItem(status)
                if status == "running":
                    status_item.setForeground(Qt.green)
                else:
                    status_item.setForeground(Qt.red)
                self.table.setItem(i, 2, QTableWidgetItem(status_item))
                
                # Сеть
                networks = list(container.attrs['NetworkSettings']['Networks'].keys())
                net_text = ", ".join(networks)
                self.table.setItem(i, 3, QTableWidgetItem(net_text))
                
                # Кнопки управления
                actions_widget = QWidget()
                actions_layout = QHBoxLayout(actions_widget)
                actions_layout.setContentsMargins(2, 2, 2, 2)
                
                if status == "running":
                    stop_btn = QPushButton("Stop")
                    stop_btn.setFixedWidth(60)
                    stop_btn.clicked.connect(lambda checked, cid=container.id: self.stop_container(cid))
                    actions_layout.addWidget(stop_btn)
                else:
                    start_btn = QPushButton("Start")
                    start_btn.setFixedWidth(60)
                    start_btn.clicked.connect(lambda checked, cid=container.id: self.start_container(cid))
                    actions_layout.addWidget(start_btn)
                
                remove_btn = QPushButton("Remove")
                remove_btn.setFixedWidth(60)
                remove_btn.setStyleSheet("color: red;")
                remove_btn.clicked.connect(lambda checked, cid=container.id: self.remove_container(cid))
                actions_layout.addWidget(remove_btn)
                
                self.table.setCellWidget(i, 4, actions_widget)
                
        except Exception as e:
            print(f"Error refreshing containers: {e}")

    def stop_container(self, container_id):
        if self.docker_manager.stop_container(container_id):
            self.refresh_containers()

    def start_container(self, container_id):
        if self.docker_manager.start_existing_container(container_id):
            self.refresh_containers()

    def remove_container(self, container_id):
        reply = QMessageBox.question(self, 'Confirm Delete', 
                                   "Are you sure you want to remove this container?",
                                   QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            if self.docker_manager.remove_container(container_id):
                self.refresh_containers()
