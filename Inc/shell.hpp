#include "os.h"
#include <string.h>

#include <vector>
#include <stack>

#define SHELL_BUFFER_SIZE 128
#define MAXARGNUM 5
#define ARGSIZE 10

using namespace std;
class sh{

    using entry_t = int(*)(int argc,char** argv);

    struct cmd_t{
        char* name;
        entry_t entry;
    };

private:
    /* command string store */
    char _buffer[SHELL_BUFFER_SIZE];
    /* arguments number */
    int argc;
    /* arguments vector */
    char* argv[MAXARGNUM];

    vector<struct cmd_t> cmdList;

public:
    void parse(){
        argc = 0;
        char* tmp = strtok(_buffer," ");
        for(;argc < MAXARGNUM && tmp;++argc){
            argv[argc]=tmp;
            tmp = strtok(NULL," ");
        }
    }

    void run(){
        char* cmd = argv[0];
        for(auto cmdOpt : cmdList){
            if(strcmp(cmd, cmdOpt.name) == 0){
                cmdOpt.entry(argc,argv);
                return;
            }
        }
        printf("unregister command\n");
    }

    void registerCmd(char* name,entry_t emtry){
        cmdList.push_back({name,emtry});
    }

    char* getCmdBuffer(){
        return _buffer;
    }
};