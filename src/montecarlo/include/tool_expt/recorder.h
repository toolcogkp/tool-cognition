/**
 * @file recorder.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * logging class to save and export data to csv file to check on other platforms such as Matlab.
 * @version 0.1
 * @date 2019-04-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef RECORDER_H
#define RECORDER_H

// standard lib
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include <vector>

// definitions
#include <tool_expt/definitions.h>

/// Flag to start recording and export to csv
// #define RECORD_START

using namespace std;

/**
 * @brief structure for data to record. Add more data entry here.
 * 
 */
struct dataset
{
    /*list data type to record */ 
    // float Ka[3];
    
    int colFlag;
    int smallFlag;
    
    double score;
    int num_visit;

    int index[7];

    string label;
    string type;
};

/**
 * @brief class to export data into csv file
 * 
 */
class Recorder
{
public:
    /**
     * @brief Construct a new Recorder object
     * 
     */
    Recorder();
    /**
     * @brief Destroy the Recorder object
     * 
     */
    virtual ~Recorder();

    /**
     * @brief save the data into the records
     * 
     * @param d data to save
     */
    void saveData(dataset d);

    /**
     * @brief export out to csv file
     * 
     */
    void save2File();

    /**
     * @brief clear all the records and free up memory
     * 
     */
    void clearRecords();

    /**
     * @brief Set the Name of object
     * 
     * @param ss name
     */
    void setName(string ss){ name = ss;}

    /**
     * @brief show current pointer address
     * 
     */
    void showAddress();

private:
    int fileidx;
    int size;
    int idx;
    string name;

    struct dataset *records;
};

#endif //RECORDER_H
