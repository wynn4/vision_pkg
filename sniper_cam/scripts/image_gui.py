#! /usr/bin/env python
import os
import sys
import numpy as np
# sudo apt-get install python-tk
from Tkinter import *
import tkFileDialog
import tkMessageBox
# sudo apt-get install python-imaging python-imaging-tk
from PIL import Image as Im
from PIL import ImageTk
from functools import partial
from math import ceil, sqrt
import rospy
import roslib
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sniper_cam.msg import interopImages

from cv_bridge import CvBridge, CvBridgeError

# extend the tkinter class Frame to adapt to our current application
class Application(Frame):

    def submitInfo(self):
        if self.imageType == 'Rotated':
            self.imageMessage = self.rotateImage
        elif self.imageType == 'Cropped':
            self.imageMessage = self.croppedImage
        else:
            self.imageMessage = self.image

        try:
            image_msg = self.bridge.cv2_to_imgmsg(np.array(self.imageMessage), "rgb8")
        except CvBridgeError as e:
            print(e)
	
        #file = 'characteristics_target_{}.txt'.format(self.targetDir[-1])
        #writeFile = open(file, 'wb')
        orientation = self.vals[2] - self.rotateValue.get()
        while(orientation < 0):
            orientation += 360

        self.msg.image = image_msg
        self.msg.type = self.typeContent.get()
        self.msg.gps_lati = float(self.vals[0])
        self.msg.gps_longit = float(self.vals[1])
        self.msg.target_color = self.tColorContent.get()
        self.msg.target_shape = self.tShapeContent.get()
        self.msg.symbol = self.letterContent.get()
        self.msg.symbol_color = self.lColorContent.get()
        self.msg.orientation = int(orientation)
        self.msg.description = self.descriptionContent.get()
        self.msg.autonomous = False
        self.pub.publish(self.msg)


        '''
        writeFile.write('{}\n,{}\n,{}\n,{}\n,{}\n,{}\n,{}\n,{}'.format(self.typeContent.get(), self.vals[0], self.vals[1],
                                                                orientation, self.tShapeContent.get(), self.letterContent.get(),
                                                                self.tColorContent.get(), self.lColorContent.get()))
        '''
        self.letterContent.set("")
        self.rotateValue.set(0)
	self.refValue=0
        self.master.after_cancel(self.rotateJob)
	self.rotateImage=None
        self.rotateJob=None
        self.image_tk=None

    def sampleRotate(self):
        width, height = self.image.size
        expand = True
        if self.rotateValue.get() != self.refValue:
            self.refValue = self.rotateValue.get()
            if self.croppedImage:
                self.rotateImage = self.croppedImage.rotate(self.refValue, resample=Im.BICUBIC, expand=expand)
            else:
                self.rotateImage = self.image.rotate(self.refValue, resample=Im.BICUBIC, expand=expand)
            #self.panel.destroy()
            # converts the PIL image to a tk image
            self.image_tk = ImageTk.PhotoImage(self.rotateImage)
            # create the label with the embedded image
            self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
            self.imageType = 'Rotated'
            # display the image in the GUI across 5 columns
            #self.panel.grid(row=0, column=1, columnspan=5)
        self.rotateJob = self.after(1000, self.sampleRotate)

    def on_button_press(self, event):
        if self.rect:
            self.canvas.delete(self.rect)
        # save mouse drag start position
        self.start_x = event.x
        self.start_y = event.y
        # create rectangle if not yet exist
        #if not self.rect:
        self.rect = self.canvas.create_rectangle(self.x, self.y, 1, 1, fill="")

    def on_move_press(self, event):
        self.curX, self.curY = (event.x, event.y)
        # expand rectangle as you drag the mouse
	x_dist = abs(self.start_x - self.curX)
	y_dist = x_dist * 3 / 4
	self.curY = self.start_y + y_dist if self.curY > self.start_y else self.start_y - y_dist
        self.canvas.coords(self.rect, self.start_x, self.start_y, self.curX, self.curY)

    def on_button_release(self, event):
        pass

    def right_click(self, event):
        if self.rect:
            self.canvas.delete(self.rect)
            self.rect=None

    def cropImage(self):
        if not self.rect:
            return

        width, height = self.image.size
        offsetX = (self.master.winfo_screenwidth()/2) - (width / 2)
        offsetY = ((self.master.winfo_screenheight()-200)/2) - (height / 2)
        self.toBeCropped = self.originalImage

        if self.imageType=='Cropped':
            self.toBeCropped = self.croppedImage

        if self.imageType=='Rotated':
            rWidth, rHeight = self.rotateImage.size
            if rWidth != width:
                offsetX = offsetX - (rWidth - width) / 2
                offsetY = offsetY - (rHeight - height) / 2
                self.toBeCropped = self.rotateImage
            else:
                self.toBeCropped = self.image

        if self.image:
            #print(self.toBeCropped.size)
            #print('{0} {1}'.format(offsetX, offsetY))
            if self.imageType == 'Standard':
                self.croppedImage = self.toBeCropped.crop((int((self.start_x-offsetX)*self.w_mult), int((self.start_y-offsetY)*self.h_mult),
                                    int((self.curX-offsetX)*self.w_mult), int((self.curY-offsetY)*self.h_mult)))
            else:
                self.croppedImage = self.toBeCropped.crop((self.start_x-offsetX, self.start_y-offsetY,self.curX-offsetX, self.curY-offsetY))
            self.croppedImage = self.croppedImage.resize((1288,964), Im.BICUBIC)
            #print('{0} {1} {2} {3}'.format(self.start_x, self.start_y, self.curX, self.curY))
            self.image_tk = ImageTk.PhotoImage(self.croppedImage)
            # create the label with the embedded image
            self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
            self.imageType = 'Cropped'
            self.canvas.delete(self.rect)

    def undoCrop(self):
        if self.croppedImage:
            self.rotateImage=None
            self.croppedImage=None
            self.rotateValue.set(0)
            self.image_tk = ImageTk.PhotoImage(self.image)
            # create the label with the embedded image
            self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
            self.imageType = 'Standard'

    # create the buttons for the GUI and attach the corresponding functions
    def createWidgets(self):
        self.cropButton = Button(self, width=10, height=1, text="CROP", command=self.cropImage, state=DISABLED)
        self.cropButton.grid(row=1, column=6)
        self.undo = Button(self, width=10, heigh=1, text="UNDO CROP", command=self.undoCrop, state=DISABLED)
        self.undo.grid(row=2, column=6)
        #self.select = Button(self, width=10, height=1, text="SELECT", command=self.drawRectangle)
        #self.select.grid(row=1, column=6)
        # create the button that allows us to submit the information
        self.submit = Button(self, width=20, height=1, text="SUBMIT", fg="green", command=self.submitInfo, state=DISABLED)
        self.submit.grid(row=1, column=4, columnspan=2)

        self.rotateValue = IntVar()
        self.rotateValue.set(0)
        self.refValue = 0
        self.rotateLabel = Label(self, text="Counter Clockwise >>")
        self.rotateLabel.grid(row=2, column=4, columnspan=2)
        self.rotateScale = Scale(self, from_=0, to=360, orient=HORIZONTAL, width=10, length=150,
                            sliderlength=15, variable=self.rotateValue, state=DISABLED)
        self.rotateScale.grid(row=3, column=4, columnspan=2)

        # create the button that allows us to quit the program
        self.quit = Button(self, width=10, height=1, text="QUIT", fg="red", command=self.quit)
        # display the button on the GUI
        self.quit.grid(row=6, column=6)

        self.tColorLabel = Label(self, text="Background Color")
        self.tColorLabel.grid(row=1,column=1)

        self.tColorContent = StringVar()
        self.tColorContent.set("white")
        self.targetColor = OptionMenu(self, self.tColorContent, "white", "black", "gray", "red", "blue",
                            "green", "yellow", "purple", "brown", "orange")
        self.targetColor.grid(row=2,column=1)

        self.tShapeLabel = Label(self, text="Shape")
        self.tShapeLabel.grid(row=3,column=1)

        self.tShapeContent = StringVar()
        self.tShapeContent.set("circle")
        self.targetShape = OptionMenu(self, self.tShapeContent, "circle", "semicircle", "quarter_circle", "triangle", "square", "rectangle", "trapezoid",
                            "pentagon", "hexagon", "heptagon", "octagon", "star", "cross" )
        self.targetShape.grid(row=4,column=1)

        self.alphaColorLabel = Label(self, text="Alphanumeric Color")
        self.alphaColorLabel.grid(row=1,column=2)

        self.lColorContent = StringVar()
        self.lColorContent.set("gray")
        self.alphaColor = OptionMenu(self, self.lColorContent, "white", "black", "gray", "red", "blue", "green", "yellow",
                            "purple", "brown", "orange")
        self.alphaColor.grid(row=2,column=2)

        self.letterLabel = Label(self, text="Alphanumeric")
        self.letterLabel.grid(row=3,column=2)

        self.letterContent = StringVar()
        self.letter = Entry(self, width=20, textvariable=self.letterContent, state=DISABLED)
        self.letter.grid(row=4,column=2)

        self.typeLabel = Label(self, text="Target Type")
        self.typeLabel.grid(row=3, column=3)

        self.typeContent = StringVar()
        self.typeContent.set("standard")
        self.typeMenu = OptionMenu(self, self.typeContent, "standard", "emergent")
        self.typeMenu.grid(row=4,column=3)

        # create a label to explain the functionality of the radio buttons
        self.targetLabel = Label(self, text="Select Target Directory")
        # display the label on the GUI
        self.targetLabel.grid(row=1,column=3)

        self.targetButton = Button(self, width=18, height=1, text="BROWSE", command=self.browseDirectory)
        self.targetButton.grid(row=2, column=3)

        Label(self, text=" ").grid(row=5, column=1)
        self.descriptionLabel = Label(self, text="Emergent Description:")
        self.descriptionLabel.grid(row=6, column=1)

        self.descriptionContent = StringVar()
        self.descriptionField = Entry(self, textvariable=self.descriptionContent, state=DISABLED)
        self.descriptionField.grid(row=6, column=2)


    def loadImage(self, event):
	if self.red_rect:
            self.canvas.delete(self.red_rect)

        filename='{}/{}'.format(self.targetDir,self.images[int(self.red_pos[0]*self.columns+self.red_pos[1])])
        try:
            self.image = Im.open(filename)
        except OSError:
            return
        self.canvas.unbind('<Button 1>')
        self.master.unbind('<Up>')
        self.master.unbind("<Down>")
        self.master.unbind("<Left>")
        self.master.unbind("<Right>")

        
        self.savedImages=[]
        self.submit.configure(state=NORMAL)
        self.cropButton.configure(state=NORMAL)
        self.undo.configure(state=NORMAL)
        self.rotateScale.configure(state=NORMAL)
        self.letter.configure(state=NORMAL)
        self.descriptionField.configure(state=NORMAL)

        self.canvas.bind("<ButtonPress-1>",self.on_button_press)
        self.canvas.bind("<ButtonPress-3>", self.right_click)
        self.canvas.bind("<B1-Motion>", self.on_move_press)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release)
        self.image = Im.open(filename)
        self.originalImage = self.image
        width, height = self.image.size
        self.w_mult = float(width) / 1288
        self.h_mult = float(height) / 964
        # resizes the image so that all are the same size
        self.image = self.image.resize((1288, 964), Im.BICUBIC)
        # converts the PIL image to a tk image
        self.image_tk = ImageTk.PhotoImage(self.image)
        # create the label with the embedded image
        self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
        self.imageType = 'Standard'
	self.rotateJob = self.after(1000, self.sampleRotate)
        self.averagePositionVals()

    def browseDirectory(self):
        self.targetDir = tkFileDialog.askdirectory()
        self.loadFiles()

    def averagePositionVals(self):
        self.vals = [0,0]
        values = open('{}/target_{}_locations.txt'.format(self.paramDir, self.targetDir[-1]), 'rb')
        count = 0
        image_no = self.red_pos[0] * self.columns + self.red_pos[1]
        m=0
        for line in values:
            if(line!='\n'):
                split_vals = line.split(',')
                self.vals[0] += float(split_vals[2])
                self.vals[1] += float(split_vals[3])
                count += 1
                if(m==image_no):
                    orientation = float(split_vals[4])
                m+=1
            else:
              break

        self.vals = list(map(lambda x: x / count, self.vals))
        self.vals.append(orientation)

    def resetObjects(self):
	if self.rotateJob:
            self.master.after_cancel(self.rotateJob)
            self.rotateJob=None
        self.rotateValue.set(0)
	self.refValue=0
	if self.croppedImage:
	    self.croppedImage=None
        if self.image:
            self.image_tk=None
            self.image=None
	if self.rotateImage:
	    self.rotateImage=None

	


    def loadFiles(self, event=None):
        self.resetObjects()

        try:
            files = os.listdir(self.targetDir)
        except OSError:
            return
        self.cropButton.configure(state=DISABLED)
        self.undo.configure(state=DISABLED)
        self.rotateScale.configure(state=DISABLED)
        self.letter.configure(state=DISABLED)
        self.descriptionField.configure(state=DISABLED)

        self.canvas.bind("<Button-1>", self.move_red_rect)
        self.canvas.bind("<Double-Button-1>", self.loadImage)
        self.canvas.bind("<Button-2>", self.loadFiles)
        self.master.bind("<Up>", self.move_up)
        self.master.bind("<Down>", self.move_down)
        self.master.bind("<Left>", self.move_left)
        self.master.bind("<Right>", self.move_right)

        self.images = [x for x in files if '.jpg' in x]
        self.savedImages = []
        self.paramDir = os.path.join(os.path.dirname(os.path.dirname(self.targetDir)),'target_locations_sorted')
        try:
            locations_files = os.listdir(self.paramDir)
        except OSError:
            return
        self.columns = ceil(sqrt(len(self.images)))
        self.rows = ceil(len(self.images) / self.columns)
        width=self.master.winfo_screenwidth()
        height=self.master.winfo_screenheight()-200
        j=0
        k=0
        self.column_size = int(width/self.columns)
        self.row_size = int(height/self.rows)
        for i in range(len(self.images)):
            image = Im.open('{}/{}'.format(self.targetDir,self.images[i]))
            image = image.resize((self.column_size, self.row_size))
            # converts the PIL image to a tk image
            image_tk = ImageTk.PhotoImage(image)
            self.savedImages.append(image_tk)
            self.canvas.create_image(j*(self.column_size)+self.column_size/2,k*self.row_size+self.row_size/2, image=image_tk)
            j+=1
            if(j==self.columns):
                j=0
                k+=1
        self.red_rect = self.canvas.create_rectangle(0, 0, self.column_size, self.row_size, fill='',outline='red')
        self.red_pos = (0,0)


    def move_up(self, event):
        if not self.red_rect:
            return
        if self.red_pos[0] == 0:
            if(((self.rows-1)*self.columns+self.red_pos[1]) > (len(self.images)-1)):
                self.red_pos = (self.rows-2, self.red_pos[1])
            else:
                self.red_pos = (self.rows-1, self.red_pos[1])
        else:
            self.red_pos = (self.red_pos[0]-1, self.red_pos[1])
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size,
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')
    def move_down(self, event):
        if not self.red_rect:
            return
        if self.red_pos[0] == self.rows-1:
            self.red_pos = (0, self.red_pos[1])
        else:
            if(((self.red_pos[0]+1)*self.columns+self.red_pos[1]) > (len(self.images)-1)):
                self.red_pos = (0, self.red_pos[1])
            else:
                self.red_pos = (self.red_pos[0]+1, self.red_pos[1])
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size,
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')

    def move_left(self, event):
        if not self.red_rect:
            return
        if self.red_pos[1] == 0:
            if((self.red_pos[0]*self.columns+(self.columns-1)) > (len(self.images)-1)):
                self.red_pos = (self.red_pos[0], (len(self.images) % self.rows)-1)
            else:
                self.red_pos = (self.red_pos[0], self.columns-1)
        else:
            self.red_pos = (self.red_pos[0], self.red_pos[1]-1)
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size,
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')
    def move_right(self, event):
        if not self.red_rect:
            return
        if self.red_pos[1] == self.columns-1:
            self.red_pos = (self.red_pos[0], 0)
        else:
            if(self.red_pos[0]*self.columns+self.red_pos[1]+1) > (len(self.images)-1):
                self.red_pos = (self.red_pos[0], 0);
            else:
                self.red_pos = (self.red_pos[0], self.red_pos[1]+1)
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size,
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')

    def move_red_rect(self, event):
        if not self.red_rect:
            return
        column = int(event.x / self.column_size)
        row = (event.y / self.row_size)
        if((row*self.columns+column) > (len(self.images)-1)):
            return
        self.red_pos = (row, column)
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size,
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')

    # initialize the application
    def __init__(self, master=None):
        self.master = master
        self.i = 0
        # auto discard delay time in milliseconds
        self.delay = 2000
        # create the image file and initialize it to none
        self.image = None
        self.displayImage = None
        self.rotateImage = None
        self.croppedImage = None
        self.rotateJob = None
        self.imageType = ''
	self.originalImage =None																																								

        self.pub = rospy.Publisher('plans', interopImages, queue_size =  10)
        self.bridge = CvBridge()
        self.msg = interopImages()

        rospy.init_node('death_star', anonymous=True)

        # create the frame
        Frame.__init__(self, master)
        # pack it up
        self.pack()

        self.x = self.y = 0
        self.canvas = Canvas(self, height=self.master.winfo_screenheight()-200, width=self.master.winfo_screenwidth(), cursor="cross")
        self.canvas.grid(row=0, column=1, columnspan=6)
        self.rect=None

        # add all the buttons
        self.createWidgets()
        self.master.grab_set()
        self.master.grab_release()

# create the application and run it
root = Tk()
#root.resizable(width=False, height=False)
width, height = root.winfo_screenwidth(), root.winfo_screenheight()
app = Application(master=root)
app.mainloop()
root.destroy()
