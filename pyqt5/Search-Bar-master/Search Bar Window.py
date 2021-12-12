import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from pymongo import MongoClient

class SearchBoxClass(object):
	def SearchBoxWinFn(self, window):
		window.setWindowTitle("SearchBoxWindow")
		window.resize(400, 400)
		window.setStyleSheet("font:Optima; background-color:rgb(0, 222, 148)")

		self.searchBar = QtWidgets.QTextEdit(window)
		self.searchBar.setGeometry(QtCore.QRect(95, 40, 200, 30) )
		self.searchBar.setStyleSheet("background-color:white")

		pushBtn = QtWidgets.QPushButton(window)
		pushBtn.setText("Search Now")
		pushBtn.move(140,100)
		pushBtn.setStyleSheet("background-color:rgba(255,255,255,0.2)")
		pushBtn.clicked.connect(self.searchMongoFunction)

		self.resultBox = QtWidgets.QTextEdit(window)
		self.resultBox.setGeometry(QtCore.QRect(60, 160, 280, 100))
		self.resultBox.setStyleSheet("background-color:rgba(0, 0, 0, 0.5); color:white;font:18px Optima;")
		window.show()


	def searchMongoFunction(self):
		dbconn = MongoClient("mongodb://localhost:27017/") # check for MongoDB port
		# Connect to MongoDB database as db and collection as coll
		query = self.searchBar.toPlainText().strip()
		result = dbconn.db.coll.find({"name":{'$regex' : query, '$options' : 'i'}})
		for i in result:
			concat = self.resultBox.toPlainText().strip()
			self.resultBox.setText( concat+"\n"+i['name'])
		dbconn.close()

if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	MainWindow = QtWidgets.QMainWindow()
	SearchBoxClass().SearchBoxWinFn(MainWindow)
	sys.exit(app.exec_())