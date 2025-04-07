#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt
from gui_window import MainWindow

def main():
    # Create application
    app = QApplication(sys.argv)
    
    # Set application name and organization
    app.setApplicationName("Pi5 Dashboard")
    app.setOrganizationName("Pi5Projects")
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    # Start the application event loop
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()