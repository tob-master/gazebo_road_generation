# The German Traffic Sign Recognition Benchmark
#
# sample code for reading the traffic sign images and the
# corresponding labels
#
# example:
#            
# trainImages, trainLabels = readTrafficSigns('GTSRB/Training')
# print len(trainLabels), len(trainImages)
# plt.imshow(trainImages[42])
# plt.show()
#
# have fun, Christian

# CSV FORMAT
# Filename        ;Width;Height;Roi.X1;Roi.Y1;Roi.X2;Roi.Y2;ClassId 
# 00002_00006.ppm ; 30  ; 30   ; 5    ; 5    ; 25   ; 25   ; 0

# DAT FORMAT
# [filename]   [# of objects] [[x   y   width height] [... 2nd object] ...]
# img/img2.jpg  2               100 200 50    50        50 30 25 25

import matplotlib.pyplot as plt
import csv


test_dat = 'image_training.dat'

# function for reading the images
# arguments: path to the traffic sign data, for example './GTSRB/Training'
# returns: list of images, list of corresponding labels 
def readTrafficSigns(rootpath):
    '''Reads traffic sign data for German Traffic Sign Recognition Benchmark.

    Arguments: path to the traffic sign data, for example './GTSRB/Training'
    Returns:   list of images, list of corresponding labels'''
    images = [] # images
    labels = [] # corresponding labels
    # loop over all 42 classes
    for c in range(1,2):
        prefix = rootpath + '/' + format(c, '05d') + '/' # subdirectory for class
        gtFile = open(prefix + 'GT-'+ format(c, '05d') + '.csv') # annotations file
        gtReader = csv.reader(gtFile, delimiter=';') # csv parser for annotations file
        gtReader.next() # skip header
        # loop over all images in current annotations file
        for row in gtReader:
            images.append(plt.imread(prefix + row[0])) # the 1th column is the filename
            labels.append(row[7]) # the 8th column is the label

	    #jpg conversion
	    new_format = row[0][:-3] +'jpg'

            with open(test_dat,'a') as f:
            	f.write('pos_img/' + format(c, '05d') + '/' + new_format + ' ' + '1' + ' ' + str(row[3]) + ' ' + str(row[4]) + ' ' + str(int(row[5])-int(row[3])) + ' ' + str(int(row[6])-int(row[4])) +'\n')
        gtFile.close()
    return images, labels

#trainImages, trainLabels = readTrafficSigns('/home/tb/NEWHAAAR')
#print len(trainLabels), len(trainImages)
#plt.imshow(trainImages[29])
#plt.show()

#with open("Output.txt", "w") as text_file:
#    text_file.write("Purchase Amount: {}".format(TotalAmount))

readTrafficSigns('/home/tb/Final_HaarTraining/Traffic_Signs')

                        
