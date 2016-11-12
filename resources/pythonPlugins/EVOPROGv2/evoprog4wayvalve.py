import time
from control import Control

class Evoprog4WayValve(Control):
	def __init__(self, params):
		"""constructor"""
		super(Evoprog4WayValve, self).__init__(5)
		self.address = int(params["address"])
		self.closePos = int(params["closePos"])
		self.availablePos = range(self.maxconnections)
		self.availablePos.remove(self.closePos)
		self.motorPositions = [0,1,2,3,4]
		self.map = {}
		self.lastPosition = self.closePos

	@classmethod
	def getParamsType(cls):
		"""must return a list with the types expected at the params variable in the init function"""
		dict = {}
		dict["address"] = "int"
		dict["closePos"] = "int"
		dict.update(super(Evoprog4WayValve, cls).getParamsType())
		return dict

	def getInstructions(self):
		""" must return a string with the instructions to make this component"""
		return ""

	def addConnection(self, idSource, idTarget, pos, communications):
		"""
			must register a new connection between idSource container and idTarget container
		"""
		self.map[(idSource, idTarget)] = pos

	def setConnection(self, idSource, idTarget, communications):
		"""
			must send instructions to the machine to allow connection between conatiners idSource and idTarget,
			the command must be sended throught the communications object,
			communications object has the next api:
				*) nbytessend sendString(string) -- send the string to the machine, return the bytes sended;
				*) string receiveString() -- receive and returns a string from the machine (stops when \n is received), can block;
				*) string readUntil(endCharacter) -- returns a string received from the machine, stops when the endCharacter arrives;
				*) void synch() -- synchronize with the machine, not always necesary, only for protocols compatibles;
		"""
		valvePos = self.closePos
		if (idSource, idTarget) in self.map:
			valvePos = self.map[(idSource, idTarget)]
		
		if valvePos != -1 and self.lastPosition != valvePos:
			command = "M " + str(self.address) + " " + str(valvePos) + "\r"
			print command
			communications.sendString(command)
			communications.synch()
			self.lastPosition = valvePos

	def clearConnections(self):
		"""
			must removed all the connections added with addConnection
		"""
		self.availablePos = range(self.maxconnections)
		self.availablePos.remove(self.closePos)
		self.map.clear()

	def getAvailablePos(self, communications):
		"""
			return the available position of the valve
		"""
		return self.availablePos