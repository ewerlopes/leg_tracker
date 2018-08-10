#!/usr/bin/python

import sys
import csv
import numpy as np

def printUsage():
    print('Usage: %s spatial_log optitrack_log out_filename'%(sys.argv[0]))
    sys.exit(1)

def parseSpatial(filename):
    time_dict = {}
    data = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        for row in csvreader:
            if len(row) < 1 :
                continue
            if row[0] == '#':
                continue

            data.append([float(r) for r in row])

            timestamp = float(row[0])
            if timestamp in time_dict:
                time_dict[timestamp].append(len(data) - 1)
            else:
                time_dict[timestamp] = [len(data) - 1]
    return time_dict, data




def parseOptitrackData(filename):
    time_dict = {}
    data = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        for row in csvreader:
            if len(row) < 1 :
                continue

            data.append([float(r) for r in row])

            timestamp = float(row[0])
            if timestamp in time_dict:
                time_dict[timestamp].append(len(data) - 1)
            else:
                time_dict[timestamp] = [len(data) - 1]
    return time_dict, data


def closestStamp(data, stamp):
    return stamp if stamp in data else min(data.keys(), key=lambda k: abs(k - stamp))

def assignGT(rows, gt):
    gt = np.array([gt[1], gt[2]])
    distances = [np.linalg.norm(np.array([row[4], row[5]]) - gt) for row in rows] # -3 -2
    idx = distances.index(min(distances))

    # for i in range(len(rows)):
    #     print rows[i][4], rows[i][5], distances[i]


    rows[idx][-1] = 1
    return rows


def appendZero(data):
    for i in range(len(data)):
        data[i].append(0)

def joinData(spatialData, optitrackData):
    spatialStamps, features = spatialData
    optitrackStamps, data = optitrackData

    outData = features
    appendZero(outData)

    for stamp, idx in optitrackStamps.iteritems():
        closestSpatialStamp = closestStamp(spatialStamps, stamp)
        indexes = spatialStamps[closestSpatialStamp]

        feature = [features[i] for i in indexes]
        tmp = assignGT(feature, data[idx[0]])

        for i in range(len(indexes)):
            outData[indexes[i]] = tmp[i]


    return outData


# NOTE no more info about when ending a window, but can be recovered looking at time of cluster, when become 0 ended window!
def saveToCsv(data, filename):
    with open(filename, 'w') as out_f:
        w = csv.writer(out_f, delimiter=',')
        w.writerows(data)


def main():
    if len(sys.argv) < 4:
        printUsage()

    spatialFile   = sys.argv[1]
    optitrackFile = sys.argv[2]
    outFilename   = sys.argv[3]


    spatialData = parseSpatial(spatialFile)
    optitrackData = parseOptitrackData(optitrackFile)

    joinedData = joinData(spatialData, optitrackData)

    saveToCsv(joinedData, outFilename)



if __name__ == '__main__':
    main()
