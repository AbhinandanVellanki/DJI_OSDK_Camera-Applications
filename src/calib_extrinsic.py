
#!/usr/bin/env python

import cv2
import numpy as np
import os
import glob

#To calibrate two camera with each other and produce the relative extrinsic camera matrix

def calibrate_camera(camera_name, img_directory):
	# obtained from https://learnopencv.com/camera-calibration-using-opencv/

	# Defining the dimensions of checkerboard
	CHECKERBOARD = (6,8)
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	# Creating vector to store vectors of 3D points for each checkerboard image
	objpoints = []
	# Creating vector to store vectors of 2D points for each checkerboard image
	imgpoints = [] 


	# Defining the world coordinates for 3D points
	objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
	objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
	prev_img_shape = None

	# Extracting path of individual image stored in a given directory
	images_path = './'+img_directory+'/*.png'
	print("Reading images from :"+images_path)

	images = glob.glob(images_path)
	for fname in images:
	    img = cv2.imread(fname)
	    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	    # Find the chess board corners
	    # If desired number of corners are found in the image then ret = true
	    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
	    
	    """
	    If desired number of corner are detected,
	    we refine the pixel coordinates and display 
	    them on the images of checker board
	    """
	    if ret == True:
	        objpoints.append(objp)
	        # refining pixel coordinates for given 2d points.
	        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
	        
	        imgpoints.append(corners2)

	        # Draw and display the corners
	        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
	    
	    #comment out below lines if you don't want to inspect the points on the images
	    cv2.imshow(fname,img)
	    cv2.waitKey(0)

	cv2.destroyAllWindows()

	h,w = img.shape[:2]

	"""
	Performing camera calibration by 
	passing the value of known 3D points (objpoints)
	and corresponding pixel coordinates of the 
	detected corners (imgpoints)
	"""
	print("obtained points, calibrating camera: "+camera_name+"...")
	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

	# print("Camera matrix : \n")
	# print(mtx)
	# print("dist : \n")
	# print(dist)
	# print("rvecs : \n")
	# print(rvecs)
	# print("tvecs : \n")
	# print(tvecs)
	return ret, mtx, dist, rvecs, tvecs


def average_vecs(rvecs, tvecs):
	#calculate mean rvec and tvec 
	prev = rvecs[0]

	for item in rvecs:
		if item.all() == prev.all():
			continue
		else:
			item_a = np.array(item)
			avg_ar= np.array(np.array(prev), item_a)
			avg = np.mean(avg_ar, axis=0)
			prev = list(avg)

	avg_rvec = prev
	prev = tvecs[0]

	for item in tvecs:
		if item.all() == prev.all():
			continue
		else:
			item_a = np.array(item)
			avg_ar = np.array(np.array(prev), item_a)
			avg = np.mean(avg_ar, axis=0)
			prev = list(avg)

	avg_tvec = prev
	
	return avg_rvec, avg_tvec

def calc_rot_mat(avg_rvec):
	rot_mat, _ = cv2.Rodrigues(avg_rvec)
	return rot_mat

def write_file(filename, mtx, dist, rvecs, tvecs, avg_rvec, avg_tvec, rot_mat):

	#create and write file to store the camera calibration results

	print("Writing to file: "+filename+"...")
	f = open(filename,"w+")
	f.write("Camera Matrix: \n")
	f.write(str(mtx)+"\n")
	f.write("\n Distortion Coeffs: \n")
	f.write(str(dist)+"\n")
	print("Wrote intrinsic parameters!")

	print("Obtained "+str(len(rvecs))+" rotation vectors...")
	f.write("\n Rotation Vectors: \n")
	for item in rvecs:
		f.write(str(item)+"\n")
	f.write("\n Number of Rotation Vectors: "+str(len(rvecs))+"\n")
	print("Written to file!")

	print("Obtained "+str(len(tvecs))+" translation vectors...")
	f.write("\n Translation Vectors: \n")
	for item in tvecs:
		f.write(str(item)+"\n")
	f.write("\n Number of Translation Vectors: "+str(len(tvecs))+"\n")
	print("Written to file!")

	print("Writing average rotation and translation vectors...")
	f.write("\n Average rotation vector: \n")
	f.write(str(avg_rvec)+"\n")
	f.write("\n Average translation vector: \n")
	f.write(str(avg_tvec)+"\n")

	print("Writing rotation matrix...")
	f.write("\nRotation Matrix: \n")
	f.write(str(rot_mat)+"\n")

	print("Done writing to file "+filename+"!")
	f.close()

def calc_relative(rot_mat_1, rot_vec_1, avg_tvec_1, rot_mat_2, rot_vec_2, avg_tvec_2):
	rot_mat_w_1 = np.array(rot_mat_1)
	rot_mat_w_2 = np.array(rot_mat_2)

	tra_mat_w_1 = np.array(avg_tvec_1)
	tra_mat_w_2 = np.array(avg_tvec_2)

	tra_mat_1_w = tra_mat_w_1 * -1

	rot_mat_1_w = np.linalg.inv(rot_mat_w_1)
	rel_rot_mat_1_2 = np.matmul(rot_mat_1_w, rot_mat_w_2)

	rot_vec_1_w = np.array(rot_vec_1) * -1
	rot_vec_w_2 = np.array(rot_vec_2)
	rel_rot_vec_1_2 = np.cross(rot_vec_1_w.T, rot_vec_w_2.T)

	rel_t_mat_1_2 = np.add(tra_mat_1_w, np.matmul(rot_mat_1_w, tra_mat_w_2))
	#rel_t_mat1_2 = np.subtract(avg_tvec_1, avg_tvec_2)

	return rel_rot_vec_1_2.T, rel_rot_mat_1_2, rel_t_mat_1_2

def write_relative(rot_vec, rot_vec_m, rot_mat, tran_mat, camera1, camera2):
	filename = camera1+"_to_"+camera2+".txt"
	f = open(filename, "w+")
	f.write("Relative rotation vector (vector method)"+camera1+"_to_"+camera2+" : \n")
	f.write(str(rot_vec)+"\n")
	f.write("Relative rotation vector (matrix method)"+camera1+"_to_"+camera2+" : \n")
	f.write(str(rot_vec_m)+"\n")
	f.write("\nRelative rotation matrix "+camera1+"_to_"+camera2+" : \n")
	f.write(str(rot_mat)+"\n")
	f.write("\nRelative translation matrix "+camera1+"_to_"+camera2+" : \n")
	f.write(str(tran_mat)+"\n")
	f.close()
	print("Wrote relative tranformation matrices from "+camera1+" to "+camera2+" to "+filename)

def main(camera1, camera2, imgs1, imgs2):

	#calibrate camera 1
	ret1, mtx1, dist1, rvecs1, tvecs1 = calibrate_camera(camera1, imgs1)
	if ret1:
		avg_rvec_1, avg_tvec_1 = average_vecs(rvecs1, tvecs1)
		rot_mat_1 = calc_rot_mat(avg_rvec_1)
		write_file("calib_"+camera1+".txt", mtx1, dist1, rvecs1, tvecs1, avg_rvec_1, avg_tvec_1, rot_mat_1)

	#calibrate camera2
	ret2, mtx2, dist2, rvecs2, tvecs2 = calibrate_camera(camera2, imgs2)
	if ret2:
		avg_rvec_2, avg_tvec_2 = average_vecs(rvecs2, tvecs2)
		rot_mat_2 = calc_rot_mat(avg_rvec_2)
		write_file("calib_"+camera2+".txt", mtx2, dist2, rvecs2, tvecs2, avg_rvec_2, avg_tvec_2, rot_mat_2)

	rel_rot_vec_2_1, rel_rot_2_1, rel_tra_2_1 = calc_relative(rot_mat_2, avg_rvec_2, avg_tvec_2, rot_mat_1, avg_rvec_1, avg_tvec_1)
	rel_rot_vec_2_1_m = calc_rot_mat(rel_rot_2_1)
	write_relative(rel_rot_vec_2_1, rel_rot_vec_2_1_m, rel_rot_2_1, rel_tra_2_1, camera2, camera1)

	
if __name__ == '__main__':
	camera1 = "z30" #name of camera 1
	imgs1 = "z30_calib_imgs" #directory of images from camera 1

	camera2 = "l_stereo" #camera 2
	imgs2 = "l_stereo_calib_imgs" #directory of images from camera 2

	main(camera1, camera2, imgs1, imgs2)

