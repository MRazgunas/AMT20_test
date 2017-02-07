#include "ch.h"
#include "hal.h"
#include "string.h"

#include "filters.h"

#define MEDIAN_FILT_SIZE 3

uint16_t median_filter(uint16_t meas) { //only one thread can call (i think)
    static uint16_t meas_buff[MEDIAN_FILT_SIZE] = {0};
    static uint8_t data_pointer = 0; //Implement circular array

    meas_buff[data_pointer++] = meas;
    if(data_pointer == MEDIAN_FILT_SIZE) {
        data_pointer = 0;
    }
    uint16_t tmp_buff[MEDIAN_FILT_SIZE];
    memcpy(tmp_buff, meas_buff, sizeof(uint16_t) * MEDIAN_FILT_SIZE);

    //TODO: improve sorting algorithm
    //Sort buffer
    for(uint8_t i = 0; i < MEDIAN_FILT_SIZE-1; i++) {
        for(uint8_t j = i; j < MEDIAN_FILT_SIZE; j++) {
            if(tmp_buff[i] > tmp_buff[j]) {
                uint16_t tmp = tmp_buff[i];
                tmp_buff[i] = tmp_buff[j];
                tmp_buff[j] = tmp;
            }
        }
    }

    return tmp_buff[MEDIAN_FILT_SIZE/2];
}

uint16_t median_filter_psu(uint16_t meas) { //only one thread can call (i think)
    static uint16_t meas_buff[MEDIAN_FILT_SIZE] = {0};
    static uint8_t data_pointer = 0; //Implement circular array

    meas_buff[data_pointer++] = meas;
    if(data_pointer == MEDIAN_FILT_SIZE) {
        data_pointer = 0;
    }
    uint16_t tmp_buff[MEDIAN_FILT_SIZE];
    memcpy(tmp_buff, meas_buff, sizeof(uint16_t) * MEDIAN_FILT_SIZE);

    //TODO: improve sorting algorithm
    //Sort buffer
    for(uint8_t i = 0; i < MEDIAN_FILT_SIZE-1; i++) {
        for(uint8_t j = i; j < MEDIAN_FILT_SIZE; j++) {
            if(tmp_buff[i] > tmp_buff[j]) {
                uint16_t tmp = tmp_buff[i];
                tmp_buff[i] = tmp_buff[j];
                tmp_buff[j] = tmp;
            }
        }
    }

    return tmp_buff[MEDIAN_FILT_SIZE/2];
}
