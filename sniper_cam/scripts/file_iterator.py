import os
import glob
import cv2

global image_number

image_number = 0

image_path = os.path.expanduser('~') + "/Desktop/vision_files/all_images/*.jpg"

#get a list of all the image_files
file_list = glob.glob(image_path)

# sort the files to be in chronological order
file_list.sort(key=os.path.getmtime)

while True:
    #load in the current image
    #print image_number
    filename = file_list[image_number]
    stuff = filename.strip().split('/')
    image_name = stuff[-1]  #get the last part of the file path
    image = cv2.imread(os.path.expanduser('~') + "/Desktop/vision_files/all_images/" + image_name)
    cv2.imshow('image', image)
    key = cv2.waitKey(1000)
    if key == 32:
        image_number += 1
    elif key == 81 and image_number > 0:
        image_number -= 1
    # elif cv2.waitKey(10) == 81 and image_number > 0:
        # image_number -= 1
    else:
        pass


# def click(event, x, y, flags, param):
#     # if user clicks on target in the image frame
#     if event == cv2.EVENT_LBUTTONDOWN:
#         image_and_line_number += 1
#
#     elif event == cv2.EVENT_RBUTTONDOWN and image_and_line_number > 0:
#         image_and_line_number -= 1
#
#     else:
#         pass
#
# def display_image():
#
#
# global image_and_line_number = 0
#
# cv2.namedWindow('window')
# cv2.setMouseCallback('window', click)
#
# image_path = os.path.expanduser('~') + "/Desktop/vision_files/target_images/all_images/*.jpg"
#
# data_path = os.path.expanduser('~') + "/Desktop/vision_files/state_data_all_images.txt"
#
#
#
# #get a list of all the image_files
# file_list = glob.glob(image_path)
#
# # sort the files to be in chronological order
# file_list.sort(key=os.path.getmtime)






# for filename in file_list:
#     #get the last part of the filename
#     stuff = filename.strip().split('/')
#     image_name = stuff[7][:19]  #get the first 19 characters of the 7th thing in stuff
#     #print image_name
#
#     #open and parse the text file
#     for line in file(data_path, 'r'):
#         for num in line.strip().split(','):
#             #print num
#             if num == image_name:
#                 print num
#                 break
#
#     image = cv2.imread(filename)
#     cv2.imshow('image', image)
#     cv2.waitKey(2000)
#
# cv2.destroyAllWindows()
