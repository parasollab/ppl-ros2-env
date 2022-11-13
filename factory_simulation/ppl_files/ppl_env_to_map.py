import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import math
import os.path
import argparse
import pywavefront
import math

# Some variables
#
ix,iy = -1,-1
x1 = [0,0,0,0]
y1 = [0,0,0,0]

def parse_2d_obj(filename,r,t,ax):

  f = open(filename,'r')
  lines = f.readlines()

  obj_sets = lines[0].split(' ')
  num_vertices = None
  for v in obj_sets:
    if v == '':
      continue
    if num_vertices == None:
      num_vertices = int(v)
    else:
      num_vertices = int(v)
      break

  # print(num_vertices)

  vertices = []
  edges = []

  for index,line in enumerate(lines[2:-1]):
    if len(line.split()) < 1:
      continue
    if len(vertices) < num_vertices:
      # print('VERTEX',index,line)
      coord = [float(x) for x in line.replace('\n','').split()]

      # print(coord)
      coord = r.apply(coord)
      # print(coord)

      for i,c in enumerate(coord):
        coord[i] = c + t[i]
      # print(coord)

      vertices.append(coord)

    else:
      # print('EDGE',index,line)
      verts = [int(x) for x in line.replace('\n','').split()]
      edges.append((verts[0],verts[1]))
      edges.append((verts[1],verts[2]))

  for edge in edges:
    v1 = vertices[edge[0]-1]
    v2 = vertices[edge[1]-1]



    x = [v1[0],v2[0]]
    y = [v1[1],v2[1]]

    print(x,y)

    ax.plot(x,y,color='black',linewidth='3')

def parse_wavefront(filename,r,t,ax,center_mesh=True):
  obj = pywavefront.Wavefront(filename,collect_faces=True)
  vertices = obj.vertices
  faces = obj.mesh_list[0].faces

  if center_mesh:
    x = 0
    y = 0
    z = 0
    for v in vertices:
      x += v[0]
      y += v[1]
      z += v[2]
    x = x/len(vertices)
    y = y/len(vertices)
    z = z/len(vertices)
    t[0] -= x
    t[1] -= y
    t[2] -= z

  new_vertices = []
  for coord in vertices:
    coord = r.apply(coord)

    for i,c in enumerate(coord):
      coord[i] = c + t[i]
    new_vertices.append(coord)


  for face in faces:
    p1 = new_vertices[face[0]]
    p2 = new_vertices[face[1]]
    p3 = new_vertices[face[2]]

    x = [p1[0],p2[0],p3[0],p1[0]]
    y = [p1[1],p2[1],p3[1],p1[1]]

    ax.plot(x,y,color='black',linewidth='3')

def env_to_png(env_file,png_file):

    f = open(env_file,'r')
    lines = f.readlines()

    settings = {
        'boundary':[],
        'objects':[]
    }

    for index,line in enumerate(lines):
        if 'Boundary Box2D' in line:
            boundary = line.split('Boundary Box2D')[-1].replace(' ','').replace('[','').replace(']','').replace('\n','')
            dimensions = boundary.split(';')

            for d in dimensions:
                range = d.split(':')
                min,max = float(range[0]),float(range[1])
                settings['boundary'].append((min,max))

        elif 'Boundary Box' in line:
            boundary = line.split('Boundary Box')[-1].replace(' ','').replace('[','').replace(']','').replace('\n','')
            dimensions = boundary.split(';')

            for d in dimensions:
                range = d.split(':')
                min,max = float(range[0]),float(range[1])
                settings['boundary'].append((min,max))

        elif 'Passive' in line:
            obj = lines[index+1]
            values = obj.replace('\n','').split(' ')
            coordinates = []

            for v in values[1:]:
              if v == '':
                continue
              coordinates.append(float(v))

            obj_settings = {
                'filename':values[0],
                'coordinates':coordinates
            }
            settings['objects'].append(obj_settings)

    # Set boundary of image
    size_x = settings['boundary'][0][1] - settings['boundary'][0][0]
    size_y = settings['boundary'][1][1] - settings['boundary'][1][0]

    fig, ax = plt.subplots(figsize=(size_x,size_y))

    plt.xlim([settings['boundary'][0][0],settings['boundary'][0][1]])
    plt.ylim([settings['boundary'][1][0],settings['boundary'][1][1]])
    ax.axes.xaxis.set_visible(False)
    ax.axes.yaxis.set_visible(False)

    ax.spines["top"].set_linewidth(4)
    ax.spines["bottom"].set_linewidth(4)
    ax.spines["left"].set_linewidth(4)
    ax.spines["right"].set_linewidth(4)

    # Parse points and lines in object files

    for obj in settings['objects']:

      rotation = [math.radians(x) for x in obj['coordinates'][-3:]]

      r = R.from_rotvec(rotation)
      t = obj['coordinates'][0:3]

      filename = obj['filename']

      suffix = filename.split('.')[-1]

      if suffix == 'g':
        parse_2d_obj(filename,r,t,ax)
      elif suffix == 'obj':
        parse_wavefront(filename,r,t,ax)
      else:
        raise Exception('File type' + suffix + 'not recognized.')

    plt.savefig(png_file)

def convert_to_binary(png_file, binary_file):
    # Read the image file
    img = cv2.imread(png_file)

    ret, bw_img = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)

    # Converting to its binary form
    bw = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)

    # Save image
    cv2.imwrite(binary_file, bw_img)

def make_ros_map(file_name):

    print("You will need to choose the x coordinates horizontal with respect to each other")
    print("Double Click the first x point to scale")
    #
    # Read in the image
    #
    image = cv2.imread(file_name)
    #
    font = cv2.FONT_HERSHEY_SIMPLEX
    #
    # mouse callback function
    # This allows me to point and
    # it prompts me from the command line
    #
    def draw_point(event,x,y,flags,param):
      global ix,iy,x1,y1n,sx,sy
      if event == cv2.EVENT_LBUTTONDBLCLK:
        ix,iy = x,y
        print(ix,iy)

        # Draws the point with lines around it so you can see it
        image[iy,ix]=(0,0,255)
        cv2.line(image,(ix+2,iy),(ix+10,iy),(0,0,255),1)
        cv2.line(image,(ix-2,iy),(ix-10,iy),(0,0,255),1)
        cv2.line(image,(ix,iy+2),(ix,iy+10),(0,0,255),1)
        cv2.line(image,(ix,iy-2),(ix,iy-10),(0,0,255),1)

        # This is for the 4 mouse clicks and the x and y lengths
        if x1[0] == 0:
          x1[0]=ix
          y1[0]=iy
          print('Double click a second x point')

        elif (x1[0] != 0 and x1[1] == 0):
          x1[1]=ix
          y1[1]=iy
          prompt = '> '
          print("What is the x distance in meters between the 2 points?")
          deltax = float(input(prompt))
          dx = math.sqrt((x1[1]-x1[0])**2 + (y1[1]-y1[0])**2)*.05
          sx = deltax / dx
          print("You will need to choose the y coordinates vertical with respect to each other")
          print('Double Click a y point')

        elif (x1[1] != 0 and x1[2] == 0):
          x1[2]=ix
          y1[2]=iy
          print('Double click a second y point')

        else:
          prompt = '> '
          print("What is the y distance in meters between the 2 points?")
          deltay = float(input(prompt))
          x1[3]=ix
          y1[3]=iy
          dy = math.sqrt((x1[3]-x1[2])**2 + (y1[3]-y1[2])**2)*.05
          sy = deltay/dy
          print(sx, sy)
          res = cv2.resize(image, None, fx=sx, fy=sy, interpolation = cv2.INTER_CUBIC)
          # Convert to grey
          res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
          cv2.imwrite("KEC_BuildingCorrected.pgm", res );
          cv2.imshow("Image2", res)

          prompt = '> '
          print("What is the name of the new map?")
          mapName = input(prompt)

          prompt = '> '
          print("Where is the desired location of the map and yaml file?")
          mapLocation = input(prompt)
          completeFileNameMap = os.path.join(mapLocation, mapName +".pgm")
          completeFileNameYaml = os.path.join(mapLocation, mapName +".yaml")
          yaml = open(completeFileNameYaml, "w")
          cv2.imwrite(completeFileNameMap, res );
            #
            # Write some information into the file
            #
          yaml.write("image: " + mapLocation + "/" + mapName + ".pgm\n")
          yaml.write("resolution: 0.050000\n")
          yaml.write("origin: [" + str(-1) + "," +  str(-1) + ", 0.000000]\n")
          yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196")
          yaml.close()
          exit()

    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('image',draw_point)
    #
    #  Waiting for a Esc hit to quit and close everything
    #
    while(1):
      cv2.imshow('image',image)
      k = cv2.waitKey(20) & 0xFF
      if k == 27:
        break
      elif k == ord('a'):
        print('Done')
    cv2.destroyAllWindows()

def convert_env_to_map(filename):
    print("TODO")


if __name__ == '__main__':

    # Parse args for names

    parser = argparse.ArgumentParser(description='Read in filenames')
    parser.add_argument('--env_file',help='PPL environment file.')
    parser.add_argument('--png_file',help='PNG filename to store intermediate representations as.')
    args = parser.parse_args()

    env_file = args.env_file
    png_file = args.png_file
    binary_file = png_file.replace('.png','') + '_binary.png'

    env_to_png(env_file,png_file)
    convert_to_binary(png_file,binary_file)
    make_ros_map(binary_file)

