//
// Created by matej on 08/11/2020.
//

#ifndef HEATINGSYSTEMV5_INDICATOR_H
#define HEATINGSYSTEMV5_INDICATOR_H


#include <stdint.h>

class Indicator {

public:

private:

    uint8_t seq1_L[2] = {0,1};
    uint32_t seq1_T[2] = {1000, 1000};

    uint8_t seq2_L[4] = {0,1,0,1};
    uint32_t seq2_T[4] = {1000, 1000, 500, 500};

    uint8_t *seq_L_list[2] = {seq1_L, seq2_L};
    uint32_t *seq_T_list[2] = {seq1_T, seq2_T};

};


#endif //HEATINGSYSTEMV5_INDICATOR_H
