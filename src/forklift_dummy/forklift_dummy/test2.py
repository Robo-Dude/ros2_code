import cv2 

def main(image1, image2):

    # image_paths=[image1, image2] 
    # initialized a list of images 
    imgs = [image1, image2] 
    
    # for i in range(len(imgs)): 
    #     # imgs.append(cv2.imread(image_paths[i])) 
    #     imgs[i]=cv2.resize(imgs[i],(0,0),fx=0.4,fy=0.4) 
    #     # this is optional if your input images isn't too large 
    #     # you don't need to scale down the image 
    #     # in my case the input images are of dimensions 3000x1200 
    #     # and due to this the resultant image won't fit the screen 
    #     # scaling down the images  
    # # showing the original pictures 
    cv2.imshow('1',imgs[0]) 
    cv2.imshow('2',imgs[1]) 
    # cv2.imshow('3',imgs[2]) 
    
    stitchy=cv2.Stitcher.create() 
    (dummy,output)=stitchy.stitch(imgs) 
   
    return output
    
   