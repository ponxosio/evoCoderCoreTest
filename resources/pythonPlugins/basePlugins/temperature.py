import abc

class abstractclassmethod(classmethod):

	__isabstractmethod__ = True

	def __init__(self, callable):
		callable.__isabstractmethod__ = True
		super(abstractclassmethod, self).__init__(callable)


class Temperature(object):
	__metaclass__  = abc.ABCMeta

	@abc.abstractmethod
	def __init__(self, params):
		"""constructor"""

	@abstractclassmethod
	def getParamsType(cls):
		"""must return a list with the types expected at the params variable in the init function"""
		return cls()

	@abc.abstractmethod
	def getInstructions(self):
		""" must return a string with the instructions to make this component"""

	@abc.abstractmethod
	def apply(self, degrees, communications):
		"""must send instructions to change the temperature to the given real number degrees, 
		communication with the machine must be done via communications object
		ommunications object has the next api:
				*) nbytessend sendString(string) -- send the string to the machine, return the bytes sended;
				*) string receiveString() -- receive and returns a string from the machine (stops when \n is received), can block;
				*) string readUntil(endCharacter) -- returns a string received from the machine, stops when the endCharacter arrives;
				*) void synch() -- synchronize with the machine, not always necesary, only for protocols compatibles;
		"""