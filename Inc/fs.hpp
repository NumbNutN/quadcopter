#include "os.h"
#include <string.h>

#include <vector>
#include <stack>

using namespace std;
class fs{

public:
    static uint8_t idx;
    
private:

    using f_write = int(*)(const void *, size_t);

    struct dev_t{
        char* name;
        f_write write;
    };

    /* registered dev */
    vector<dev_t> devList;
    /* opened dev */
    vector<dev_t> openedList;

    dev_t _stdinD;
    dev_t _stdoutD;
    dev_t _stderrD;

public:

    fs(dev_t stdinD,dev_t stdoutD,dev_t stderrD) :_stdinD(stdinD),_stdoutD(stdoutD),_stderrD(stderrD){
        registerDevice(stdinD.name, NULL);
        registerDevice(stdoutD.name,stdoutD.write);
        registerDevice(stderrD.name, stderrD.write);
    }

    void exec(){
        open(_stdinD.name);
        open(_stdoutD.name);
        open(_stderrD.name);
    }

    int open(const char * path){
        for(auto dev:devList){
            if(strcmp(dev.name,path) == 0){
                openedList.push_back(dev);
                return fs::idx++;
            }
        }
        return -1;
    }
    
    int write(int file, const void * ptr, size_t len){
        return openedList[file].write(ptr,len);
    }

    void registerDevice(char* name,f_write write){
        devList.push_back({name,write});
    }

};