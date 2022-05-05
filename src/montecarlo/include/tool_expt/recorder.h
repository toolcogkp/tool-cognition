#ifndef RECORDER_H
#define RECORDER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include <vector>

#include <tool_expt/definitions.h>
// #define RECORD_START

using namespace std;

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

class Recorder
{
public:
    Recorder();
    virtual ~Recorder();

    void saveData(dataset d);
    void save2File();
    void clearRecords();

    void setName(string ss){ name = ss;}

    void showAddress();

private:
    int fileidx;
    int size;
    int idx;
    string name;

    struct dataset *records;
};

#endif //RECORDER_H
