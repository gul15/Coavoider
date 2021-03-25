import os
import speech_recognition

from threading import Thread

class Listener(object):
    room = None
    job  = None
    stop = False

    def __init__(self):
        self.recognizer = speech_recognition.Recognizer()
        self.microphone = speech_recognition.Microphone() 

        with self.microphone as source: 
            self.recognizer.adjust_for_ambient_noise(source)
	
	self.jobs = {'garbage','medical'}
        self.rooms = {'one', 'two', 'three', 'four','for','tree','wall','to','1','2','3','4'}

    def start(self):
        thread = Thread(target=self.run)
        thread.start()

    def run(self): 
	while 1:
		while not self.stop:
		    with self.microphone as source:
		        audio = self.recognizer.listen(source)

		    try:  # also we can run this try-except block in a thread for concurent commands 
		        value = self.recognizer.recognize_google(audio).lower()  # we can also use additional apis: bing, google_cloud, sphinx..
		        print (value)
		        
		        if 'close' in set(value.split()) and 'program' in set(value.split()):  # 
		            print('Closing program.')
		            break

		        elif 'cancel' in value:
			    self.job = None
		            self.room = None

		        else:
			    job = set(value.split())
			    room = set(value.split())
		            for room in self.rooms:
		                if room in set(value.split()):
		                    self.room = room
			    for job in self.jobs:
		                if job in set(value.split()):
		                    self.job = job
		    except:
		        #print('Command cannot be recognized!')
			self.job = None
		        self.room = None

if __name__ == '__main__':
    listener = Listener()
    listener.start()
