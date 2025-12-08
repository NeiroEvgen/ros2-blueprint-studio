import sys

def main():
    print("--- DIAGNOSTIC START ---")

    print("1. Checking Python environment...")
    print(f"   Version: {sys.version}")

    print("2. Importing PySide6...")
    try:
        from PySide6 import QtWidgets
        print("   PySide6: OK")
    except ImportError as e:
        print(f"   [ERROR] PySide6 import failed: {e}")
    except Exception as e:
        print(f"   [ERROR] Unexpected error importing PySide6: {e}")

    print("3. Importing NodeGraphQt...")
    try:
        import NodeGraphQt
        print("   NodeGraphQt: OK")
    except ImportError as e:
        print(f"   [ERROR] NodeGraphQt import failed: {e}")
    except Exception as e:
        print(f"   [ERROR] Unexpected error importing NodeGraphQt: {e}")

    print("4. Testing GUI (a small window should appear)...")
    try:
        app = QtWidgets.QApplication(sys.argv)
        
        msg = "SUCCESS: GUI is working!\nPlease close this window to finish the test."
        window = QtWidgets.QLabel(msg)
        window.setWindowTitle("Debug Check")
        window.setAlignment(Qt.AlignCenter) if 'Qt' in locals() else None
        window.resize(400, 200)
        window.show()
        
        print("   Window created, waiting for closure...")
        app.exec()
        print("   Window closed. Test passed.")
        
    except Exception as e:
        print(f"   [ERROR] GUI initialization failed: {e}")

    print("--- DIAGNOSTIC END ---")
    input("Press Enter to exit...")

if __name__ == "__main__":
    main()