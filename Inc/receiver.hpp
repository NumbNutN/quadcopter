#include <vector>

#include "stdint.h"

class receiver{

private:

    using handler_t = void(*)(float sig);
    static const uint8_t channel_num = 8;
    std::vector<handler_t>_sig_handler_list = std::vector<handler_t>(channel_num,nullptr);

    //接受信号区
    float _signal_receive_buf[channel_num];

public:
    enum{
        LATERAL_MOVE = 0,
        FORWARD_BACK = 1,
        THRUTTLE_CHANNEL = 2,
        DIRECTION = 3,
        UNLOCK = 6
    };

    void register_handler(uint8_t channel,handler_t handler){
        _sig_handler_list[channel] = handler;
    }

    void run(){
        for(int idx=0;idx<channel_num;++idx){
            if(_sig_handler_list[idx] == (void*)0)continue;
            _sig_handler_list[idx](_signal_receive_buf[idx]);
        }
    }

    float* getReceiverBuf(){
        return _signal_receive_buf;
    }
};