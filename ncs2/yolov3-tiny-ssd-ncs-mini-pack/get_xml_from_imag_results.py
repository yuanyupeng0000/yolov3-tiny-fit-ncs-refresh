import os, cv2
def Visualize(img_full_path_name, img_data, results):
	img_cp = img_data
	detectedNum = len(results)
	if detectedNum > 0:
            for i in range(detectedNum):
                txt = results[i].name
                left = results[i].left
                top = results[i].top
                right = results[i].right
                bottom = results[i].bottom
	return img_cp

