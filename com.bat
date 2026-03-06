pyinstaller --noconsole --onefile --name="Ros2Studio" --hidden-import=NodeGraphQt --hidden-import=docker --paths=. main.py
pause