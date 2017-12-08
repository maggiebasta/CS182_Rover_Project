from PIL import Image

# import heightmap png
im = Image.open("terrain.png") 
pix = im.load()
print im.size 

hmap = []
zrange = []

# parse the heightmap pixels
for i in range(129):
    for j in range(129):
        hmap.append((i, j, pix[i*3.969,j*3.969]))
        zrange.append(pix[i*3.969,j*3.969])


abshmap = map(lambda x: ((x[0] -64.5)*-1, (x[1] -64.5), x[2]), hmap)
print abshmap