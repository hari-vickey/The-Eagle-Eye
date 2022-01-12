# import sys
# from PyQt5.QtWidgets import QApplication, QWidget, QTableWidget, QTableWidgetItem, QPushButton, QHeaderView, QHBoxLayout, QVBoxLayout
# from PyQt5.QtCore import Qt
# import pandas as pd # pip install pandas

# class MyApp(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.window_width, self.window_height = 700, 500
#         self.resize(self.window_width, self.window_height)
#         self.setWindowTitle('Load Excel (or CSV) data to QTableWidget')

#         layout = QVBoxLayout()
#         self.setLayout(layout)

#         self.table = QTableWidget()
#         layout.addWidget(self.table)

#         self.button = QPushButton('&Load Data')
#         self.button.clicked.connect(lambda _, xl_path=excel_file_path, sheet_name=worksheet_name: self.loadExcelData(xl_path, sheet_name))
#         layout.addWidget(self.button)

#     def loadExcelData(self, excel_file_dir, worksheet_name):
#         df = pd.read_excel(excel_file_dir, worksheet_name)
#         if df.size == 0:
#             return

#         df.fillna('', inplace=True)
#         self.table.setRowCount(df.shape[0])
#         self.table.setColumnCount(df.shape[1])
#         self.table.setHorizontalHeaderLabels(df.columns)

#         # returns pandas array object
#         for row in df.iterrows():
#             values = row[1]
#             for col_index, value in enumerate(values):
#                 if isinstance(value, (float, int)):
#                     value = '{0:0,.0f}'.format(value)
#                 tableItem = QTableWidgetItem(str(value))
#                 self.table.setItem(row[0], col_index, tableItem)

#         self.table.setColumnWidth(2, 300)
        
# if __name__ == '__main__':
#     # don't auto scale when drag app to a different monitor.
#     # QGuiApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    
#     excel_file_path = 'D:/git learning/The-Eagle-Eye/test_programs/sheet/Sample_Data.xlsx'
#     worksheet_name = 'Sheet1'

#     app = QApplication(sys.argv)
#     app.setStyleSheet('''
#         QWidget {
#             font-size: 17px;
#         }
#     ''')
    
#     myApp = MyApp()
#     myApp.show()

#     try:
#         sys.exit(app.exec())
#     except SystemExit:
#         print('Closing Window...')



#-------------------------------------------------------------------------
import pandas as pd 
import openpyxl as xlrd

read_file = pd.read_csv(r"path where your csv file is stored/Details.csv")
read_file.to_excel(r"path where you want to store xlsx/Details.xlsx", index=None, header=True)

self.anomaly_details = xlrd.open_workbook("./appData/Details.xlsx")
self.sheet = self.anomaly_details.sheet_by_index(0)
self.data = [
    [self.sheet.cell_value(r, c) for c in range(self.sheet.ncols)]
    for r in range(self.sheet.nrows)
]
# print(self.data)
self.alarm_tableWidget.setColumnCount(4)
self.alarm_tableWidget.setRowCount(
    self.sheet.nrows - 1
)  # same no.of rows as of csv file
for row, columnvalues in enumerate(self.data):
    for column, value in enumerate(columnvalues):
        self.item = QtWidgets.QTableWidgetItem(
            str(value)
        )  # str is to also display the integer values
        self.alarm_tableWidget.setItem(row - 1, column, self.item)
        # to set the elements read only
        self.item.setFlags(QtCore.Qt.ItemIsEnabled)