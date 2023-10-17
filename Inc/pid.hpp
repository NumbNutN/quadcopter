#include <utility>

// class pid{

// private:
//     float kp,ki,kd;

//     float _last_var = 0;
//     float _target;
//     float _accumated_error;
//     std::pair<float,float> _cur_pair;

//     using f_t = std::pair<float,float>(*)(void);
//     using callback_t = void(*)(float y);
//     using f_item = float(*)();

//     f_t get_cur_pair;
//     callback_t callback;
//     f_item get_differentiate = []()->float{return _cur_pair.first - _last_var};


// public:
//     pid(float kp,float ki,float kd,f_t get_cur_pair,callback_t callback,f_item get_differentiate) :kp(kp),ki(ki),kd(kd),
//                 get_cur_pair(get_cur_pair),
//                 callback(callback),
//                 get_differentiate(get_differentiate){}

//     void run(){
//         _cur_pair = get_cur_pair();
//         _accumated_error += _cur_pair.second;
//         float out = kp* (_target - _cur_pair.second) + ki*_accumated_error + kd*get_differentiate();
//         _last_var = _cur_pair.first;
//     }

//     void setTarget(float target){
//         _target = target;
//         _accumated_error = 0;
//         _last_var = 0;
//     }

// };

class pid{

private:
    float kp,ki,kd;

    float _last_var;
    float _cur_var;
    float _target;
    float _accumated_error;

    bool _tar_is_set = false;

    using f_t = float(*)();
    using callback_t = void(*)(float y);
    using f_item = float(*)(pid*);

    f_t get_cur_pair;
    callback_t callback;
    f_item get_differentiate;


public:
    pid(float kp,float ki,float kd,
        f_t get_cur_pair,
        callback_t callback,
        f_item get_differentiate = [](pid* obj)->float{return obj->getVarDelta();}) :kp(kp),ki(ki),kd(kd),
                get_cur_pair(get_cur_pair),
                callback(callback),
                get_differentiate(get_differentiate){}

    void run(){
        if(_tar_is_set){
            _cur_var = get_cur_pair();
            _accumated_error += _cur_var;
            float out = kp* (_target - _cur_var) + ki*_accumated_error + kd*get_differentiate(this);
            callback(out);
        }
    }

    void setTarget(float target){
        _target = target;
        _accumated_error = 0;
        _cur_var = 0;
        _last_var = 0;
        _tar_is_set = true;
    }

    void setPID(float kp,float ki,float kd){
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    float getCurError(){
        return _target - _cur_var;
    }

    float getVarDelta(){
        return _cur_var - _last_var;
    }

};