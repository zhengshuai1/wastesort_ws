import torch
import numpy as np
import  cv2
# imgsz = 640
# device = 'cuda:0'
# img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
# gn = torch.tensor(img.shape)[[1, 0, 1, 0]]
# print(gn.numpy())
# x = torch.tensor([[1, 2, 3], [4, 5, 6]])
# print(x)
# a=np.ones([3,3])
# b=np.array([1, 2, 3])
# c=np.dot(a,b)
# print(c)
# print(c.transpose().shape)
# im1 = cv2.imread('./mydata/pcdall2r.png')
# im2 = cv2.imread('./mydata/00_Color.png')
# im3 = cv2.imread('./mydata/00_Depth.png')
# # h, w = im.shape[0:2]
# print(im1.shape,im2.shape,im3.shape)

a = np.array([0, 1, 2, 3])
b = a.tolist()
b.extend([0,1])
print(b)