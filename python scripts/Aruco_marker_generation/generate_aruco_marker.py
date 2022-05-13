import numpy as np
import argparse
from utils import ARUCO_DICT
import cv2
import sys

full_fig = np.ones((200, 5,1))
gap = np.ones((200,5,1))
for i in range(1, 6):
	type = "DICT_5X5_100"
	id = i
	size = 200

	if ARUCO_DICT.get(type, None) is None:
		print("ArUCo tag type ", type, " is not supported")
		sys.exit(0)

	arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[type])

	print("Generating ArUCo tag of type ",type, "with ID ", id)
	tag_size = size
	tag = np.zeros((tag_size, tag_size, 1), dtype="uint8")
	cv2.aruco.drawMarker(arucoDict, id, tag_size, tag, 1)


	full_fig = np.hstack((full_fig,tag))
	full_fig = np.hstack((full_fig, gap))
	cv2.destroyAllWindows()

full_fig_2 = np.ones((200, 5,1))
gap = np.ones((200,5,1))
for i in range(6, 11):
	type = "DICT_5X5_100"
	id = i
	size = 200

	if ARUCO_DICT.get(type, None) is None:
		print("ArUCo tag type ", type, " is not supported")
		sys.exit(0)

	arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[type])

	print("Generating ArUCo tag of type ",type, "with ID ", id)
	tag_size = size
	tag = np.zeros((tag_size, tag_size, 1), dtype="uint8")
	cv2.aruco.drawMarker(arucoDict, id, tag_size, tag, 1)


	full_fig_2 = np.hstack((full_fig_2,tag))
	full_fig_2 = np.hstack((full_fig_2, gap))
	cv2.waitKey(0)
	cv2.destroyAllWindows()

gap = np.ones((10, 1030,1))
final_img = np.vstack((full_fig,gap, full_fig_2 ))
tag_name = input("Name of the image")
tag_name = tag_name+".png"

cv2.imwrite(tag_name, final_img)
cv2.imshow("ArUCo Tag", final_img)
cv2.waitKey(0)