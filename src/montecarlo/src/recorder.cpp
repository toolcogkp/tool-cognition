#include <tool_expt/recorder.h>
#include <tool_expt/definitions.h>

using namespace std;

Recorder::Recorder()
{
    name = "mcts";
    fileidx = 0;
    idx = 0;
    size = N_PLAYS * 9;   // N_plays * num of layers

    //allocate memory to record
    // records = (struct dataset*) malloc( size * sizeof( struct dataset) );
    
    // records = (dataset*) calloc( size, sizeof( dataset) );
    records = new dataset[size];
    cout << "memory allocated!!!" << endl;
    // memset(records, 0, size * sizeof(struct dataset));
}

Recorder::~Recorder()
{
    // free(records);
    // free( (size_t*)records - 1);

    cout << "delete memory!!!" << endl;
    delete[] records;
}

void Recorder::showAddress()
{
    cout << "records address: " << records << endl;
}

void Recorder::clearRecords()
{
    cout << "clearing memory" << endl;
    // memset(records, 0, size * sizeof(struct dataset));
    
    // free(records);
    // records = (dataset*) calloc( size, sizeof(dataset) );
    records = new dataset[size];
    cout << "memory reallocated" << endl;
    idx = 0;
}

void Recorder::saveData(dataset d)
{
    cout << "save data to records" << endl;
    // showAddress();

    if(idx < size)
    {
        // cout << "idx: " << idx << endl;
        // cout << "size: " << size << endl;
        // cout << "record struct size: " << sizeof(records) << endl;
        // cout << "record size: " << ((size_t*)records)[-1] << endl;
       
        *(records+idx) = d;
    
        cout << "saved DONE! at index: " << idx << endl;
        idx++;
    }
    else
    {
        cout << "memory size of record exceeded -> not saved!" << endl;
    }
}

void Recorder::save2File()
{
    cout << "saving records to file" << endl;

    ofstream myfile;

    std::string out_string;
    std::stringstream ss;
    /*save data location */
    // ss << "/home/rainbow/Documents/" << name << "_records" << fileidx << ".csv";
    // sftp://olivia@192.168.1.103/home/olivia/Documents
    ss << "/home/olivia/Documents/" << name << "_records_" << fileidx << ".csv";
    out_string = ss.str();

    myfile.open(out_string.c_str(), ios::out);

    if(!myfile.is_open())
    {
        cout<< "RECORDER: FAILED TO OPEN FILE @: " << out_string << endl;
        return;
    }

    /*give table header name */
    // myfile << "Ka_x, Ka_y, Ka_z, Fr_x, Fr_y, Fr_z, Fu_x, Fu_y, Fu_z, Fthres_x, Fthres_y, Fthres_z, Fcmd_x, Fcmd_y, Fcmd_z, Pos_x, Pos_y, Pos_z, Tar_x, Tar_y, Tar_z, Con_x, Con_y, Con_z\n";
    myfile << "label, ";
    for (int i=0; i < 7; i++)
    {
        myfile << "layer_" << to_string(i) <<", ";
    }
    myfile << "type, collision, small_seg, score, num_of_visits\n";

    for(int j = 0; j < idx; j++)
    {
        //write to each row in table
        // myfile << (records+j)->Ka[0] << ", ";
        // myfile << (records+j)->Ka[1] << ", ";
        // myfile << (records+j)->Ka[2] << ", ";

        myfile << (records+j)->label << ", ";

        for(int k = 0; k < 7; k++)
        {
            if( (records+j)->index[k] < 0 )
                myfile << ", ";
            else
                myfile << (records+j)->index[k] << ", ";
        }

        myfile << (records+j)->type << ", ";
        myfile << (records+j)->colFlag << ", ";
        myfile << (records+j)->smallFlag << ", ";

        myfile << (records+j)->score << ", ";
        myfile << (records+j)->num_visit;

        myfile << "\n";
    }

    myfile << "done!\n";
    myfile.close();

    cout << "DONE! records in Documents as csv" << endl;
    fileidx++;
}
