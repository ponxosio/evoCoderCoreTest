import abc

class Control(object):
	__metaclass__  = abc.ABCMeta

	def __init__(self, maxConnections):
		"""constructor"""
		self.maxconnections = maxConnections
		
	def getMaxConnections(self):
		return self.maxconnections

	@classmethod
	def getParamsType(cls):
		"""must return a list with the types expected at the params variable in the init function"""
		return {}

	@abc.abstractmethod
	def getInstructions(self):
		""" must return a string with the instructions to make this component"""

	@abc.abstractmethod
	def addConnection(self, idSource, idTarget, pos, communications):
		"""
			must register a new connection between idSource container and idTarget container
		"""

	@abc.abstractmethod
	def getAvailablePos(self, communications):
		"""
			return the available position of the valve
		"""

	@abc.abstractmethod
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
	@abc.abstractmethod
	def clearConnections(self):
		"""
			must removed all the connections added with addConnection
		"""