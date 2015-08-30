import memory, commands
from task import Task
from state_machine import *

class Playing(Task):
	def run(self):
		print("testing before Hello world")
		memory.speech.say('Hello, World')
		print("testing atfter Hello world")
		self.finish()

