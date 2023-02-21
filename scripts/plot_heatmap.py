import math
import os
import numpy as np
import json
import matplotlib.pyplot as plt
import matplotlib.colors as colors

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/"


gps_pos_file = data_dir + "pos_data_log_heatmap.json"

gzb_gps_info = {}
with open(gps_pos_file, "r") as fs:
    gzb_gps_info = json.load(fs)
gzb_true_pos = np.array(gzb_gps_info['true_pos'])
gzb_gps_pos = np.array(gzb_gps_info['gps_pos'])

#Calculating the size of the heat map based on the lat lon values
scale = 5
size_x_min = (int)((np.abs(gzb_true_pos[:,0].min()))/scale)
size_y_min = (int)((np.abs(gzb_true_pos[:,1].min()))/scale)
size_x = (int)((np.abs(gzb_true_pos[:,0].max()))/scale + size_x_min)
size_y = (int)((np.abs(gzb_true_pos[:,1].max()))/scale + size_y_min)
heat_map = np.zeros((size_y+1,size_x+1))
for i in range(gzb_true_pos.shape[0]):
    y_grid = (int)(gzb_true_pos[i,1]/scale + size_y_min)
    x_grid = (int)(gzb_true_pos[i,0]/scale + size_x_min)
    error = np.linalg.norm(gzb_true_pos[i,:] - gzb_gps_pos[i,:])
    if not math.isnan(error):
        if error > 40:
            error = 40
        heat_map[y_grid,x_grid] = error
        heat_map[y_grid,x_grid] = 0
    else:
        heat_map[y_grid,x_grid] = 40

from matplotlib.pyplot import cm
import matplotlib.colors
cmap = colors.LinearSegmentedColormap.from_list("", ["green","yellow","orange","red"])
plt.imshow(heat_map, cmap=cmap, interpolation='bilinear')
plt.colorbar()
plt.gca().invert_yaxis()
plt.axis('off')
#plt.savefig('ComparisonPlot.png', dpi=500)
plt.show()

# plt.figure(figsize=(10,10))
# speed = np.zeros(500)
# for i in range (1,500):
#     speed[i]= np.sqrt((grd_truth['lat'][i] - grd_truth['lat'][i-1])**2 + (grd_truth['lon'][i] - grd_truth['lon'][i-1])**2)
# plt1 = plt.plot(speed, label = 'Speed')
# print(len(grd_truth['lat']),len(gps_log['lat']))
# error = np.zeros(500)
# for i in range (500):
#     error[i]= np.sqrt((grd_truth['lat'][i] - gps_log['lat'][i])**2 + (grd_truth['lon'][i] - gps_log['lon'][i])**2)
# plt2 = plt.gca().twinx().plot(error, color = 'r', label = 'Error in GPS position')
# lns = plt1 + plt2
# labels = [l.get_label() for l in lns]
# plt.legend(lns, labels, loc=0)
# plt.show()

# plt.figure(figsize=(10,10))
# speed = np.zeros(len(aff_pos['true_x']))
# for i in range (1,len(aff_pos['true_x'])):
#     speed[i]= np.sqrt((aff_pos['true_x'][i] - aff_pos['true_x'][i-1])**2 + (aff_pos['true_y'][i] - aff_pos['true_y'][i-1])**2)
# plt1 = plt.plot(speed, label = 'Speed')

# error = np.zeros(len(aff_pos['true_x']))
# for i in range (len(aff_pos['true_x'])):
#     error[i]= np.sqrt((aff_pos['true_x'][i] - aff_pos['aff_x'][i])**2 + (aff_pos['true_y'][i] - aff_pos['aff_y'][i])**2)
# plt2 = plt.gca().twinx().plot(error, color = 'r', label = 'Error in GPS position')
# lns = plt1 + plt2
# labels = [l.get_label() for l in lns]
# plt.legend(lns, labels, loc=0)
# plt.show()