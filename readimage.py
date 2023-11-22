from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt

def read_image(image):
    img = Image.open(image)

    img = ImageOps.grayscale(img)

    np_image = np.array(img)
    np_img = ~np_image
    np_image[np_img > 0] = 1
    #plt.set_cmap('binary')
    #plt.imshow(np_img)

    np.save('cspace.npy', np_img)

    #grid = np.load('cspace.npy')
    #plt.imshow(grid, cmap = 'binary')
    #plt.tight_layout()