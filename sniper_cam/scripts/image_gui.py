import os
import sys
from Tkinter import *
from PIL import Image, ImageTk
from functools import partial

class Application(Frame):
    def loadNextImage(self):
	if self._job is not None:
            self.after_cancel(self._job)
	    self._job= self.after(self.delay, self.autoDiscard)
	filename = self.imagedir + '/image' + str(self.i)+ '.jpg'
	try:
	    if os.stat(filename).st_size > 0:
		self.error.grid_forget()
	except OSError:
	    self.error.grid(row=2,column=1)
	    return
        if self.i > 0:
            self.panel.destroy()
        self.image = Image.open(self.imagedir + '/image' + str(self.i)+ '.jpg')
        self.image = self.image.resize((800, 600))
        image_tk = ImageTk.PhotoImage(self.image)
        self.panel = Label(self, image=image_tk)
        self.panel.image = image_tk
        self.panel.grid(row=0, column=1, columnspan=4)
        self.i += 1

    def autoDiscard(self):
	if self.image:
		self.image.save(self.discarddir + '/image_' + str(self.i) +'.jpg')
	self.loadNextImage()

    def saveImage(self, discard=False):
	if self.image:
    	    if discard:
    	        self.image.save(self.discarddir + '/image_' + str(self.i) + '.jpg')
    	    else:
    		index = self.index[int(self.targetdir[-1])-1]
    		self.image.save(self.targetdir + '/image_' + str(index) + '.jpg')
    		self.index[int(self.targetdir[-1])-1] = index + 1
        self.loadNextImage()

    def createWidgets(self):
        self.load = Button(self, width=15, height=1, text="DISCARD", command=partial(self.saveImage,True))
        self.load.grid(row=1, column=1)

        self.save = Button(self, width=15, height=1, text="SAVE", command=self.saveImage)
        self.save.grid(row=1, column=2)

        self.quit = Button(self, width=15, height=1, text="QUIT", fg="red", command=self.quit)
        self.quit.grid(row=1, column=4)

        w = Label(self, text="Select Target:")
        w.grid(row=1,column=3)

        self.v = IntVar()
        self.v.set(1)
        self.index = []
        for i in range(1,self.targets):
        	self.index.append(0)
        	Radiobutton(self, text="Target " + str(i), variable=self.v, value=i, command=self.initTarget).grid(row=i+1, column=3)
        	self.targetdir = os.path.join(self.outputdir, 'target' + str(i))
        	if not os.path.isdir(self.targetdir):
        		os.makedirs(self.targetdir)
        	self.targetdir = self.targetdir[:-1] + '1'

    def initTarget(self):
    	self.targetdir = self.targetdir[:-1] + str(self.v.get())

    def __init__(self, master=None):
        #input counter
        self.i = 0
	self.delay = 2000
	self.image = None
        #variable toggle
        self.targetcount = 1
        self.targets=int(sys.argv[1])+1
        Frame.__init__(self, master)
        self.pack()

        self.outputdir = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'output')
        self.discarddir = os.path.join(self.outputdir, 'discarded')
        if not os.path.isdir(self.outputdir):
            os.makedirs(self.outputdir)
        if not os.path.isdir(self.discarddir):
            os.makedirs(self.discarddir)
        self.imagedir = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'images')
	if not os.path.isdir(self.imagedir):
	    os.makedirs(self.imagedir)
	self._job = self.after(self.delay, self.autoDiscard)
        self.error = Label(self, text='WAIT FOR NEXT IMAGE')
	self.loadNextImage()
        self.createWidgets()

root = Tk()
app = Application(master=root)
app.mainloop()
root.destroy()

