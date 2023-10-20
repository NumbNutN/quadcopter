#pragma once

#include "os.h"

class pid{

    using value_type = float;
    using f_t = float(*const)();
    using callback_t = void(*)(float y);
private:

    float _kp,_ki,_kd;

    float _last_output_signal;
    float _cur_output_signal;
    float _reference_signal;
    float _accumated_error;
    float control_signal;
    bool _tar_is_set = false;
    int8_t _label = '?';

    f_t get_cur_val;
    callback_t callback;
    f_t get_differentiate = nullptr;

    float _builtin_get_differentiate(){
        return _cur_output_signal - _last_output_signal;
    }

public:
    pid(float kp,float ki,float kd,
        f_t get_cur_val,
        callback_t callback,
        f_t get_differentiate = nullptr) :_kp(kp),_ki(ki),_kd(kd),
                get_cur_val(get_cur_val),
                callback(callback),
                get_differentiate(get_differentiate){}

    void run(){
        if(_tar_is_set){
            _cur_output_signal = get_cur_val();
            _accumated_error += _cur_output_signal;
            control_signal = _kp* (_reference_signal - _cur_output_signal) + _ki*_accumated_error + _kd* (get_differentiate?get_differentiate():_builtin_get_differentiate());
            callback(control_signal);
        }
    }

    void setLabel(int8_t label){
        _label = label;
    }

    void setTarget(float target){
        _reference_signal = target;
        _accumated_error = 0;
        _cur_output_signal = 0;
        _last_output_signal = 0;
        _tar_is_set = true;
    }

    void setParam(float kp,float ki,float kd){
        this->_kp = kp;
        this->_ki = ki;
        this->_kd = kd;
    }

    float getCurError() const {
        return _reference_signal - _cur_output_signal;
    }

    float getOutput() const {
        return control_signal;
    }

    int8_t getLabel() const {
        return _label;
    }

};