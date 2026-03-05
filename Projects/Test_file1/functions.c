#include "functions.h"


int grayTo_int(bool Enc_A, bool Enc_B){
    if(!Enc_A && !Enc_B){
        return 0;
    }
    else if(!Enc_A && Enc_B){
        return 1;
    }
    else if(Enc_A && Enc_B){
        return 2;
    }
    else{
        return 3;
    }
}