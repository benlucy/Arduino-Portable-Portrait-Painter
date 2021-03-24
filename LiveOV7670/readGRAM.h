//
// Created by @SouravSh25 on 22.09.2020.
//

#ifndef READ_GRAM_H
#define READ_GRAM_H

#include "Arduino.h"

extern uint32_t GRAM_buffer[];

void readID();
void writeCmd(uint8_t cmd);
void initiate_readGRAM();
void readGRAM_Row(uint32_t *dataBuff, unsigned char Row_number);
void readGRAM_Column(uint32_t *dataBuff, uint8_t Column_number);


#endif //READ_GRAM_H
