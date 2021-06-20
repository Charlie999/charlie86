#ifndef CHARLIE86_REGS_H
#define CHARLIE86_REGS_H

#include "types.h"

#define REGISTER_AL 0x00
#define REGISTER_AH 0x04
#define REGISTER_AX 0x08

#define REGISTER_CL 0x01
#define REGISTER_CH 0x05
#define REGISTER_CX 0x09

#define REGISTER_DL 0x02
#define REGISTER_DH 0x06
#define REGISTER_DX 0x0A

#define REGISTER_BL 0x03
#define REGISTER_BH 0x07
#define REGISTER_BX 0x0B

#define REGISTER_SP 0x0C
#define REGISTER_BP 0x0D
#define REGISTER_SI 0x0E
#define REGISTER_DI 0x0F

#define REGISTER_CS 0x10
#define REGISTER_DS 0x11
#define REGISTER_ES 0x12
#define REGISTER_SS 0x13

#ifndef NOREGFUNCS

uint8_t sr2regid_[] {
    REGISTER_ES, REGISTER_CS, REGISTER_SS, REGISTER_DS
};

uint8_t sr2regid(uint8_t srid) {
    return sr2regid_[srid];
}

uint8_t inc2regid_[] {
    REGISTER_AX, REGISTER_CX, REGISTER_DX, REGISTER_BX, REGISTER_SP, REGISTER_BP, REGISTER_SI, REGISTER_DI
};

uint8_t inc2regid(uint8_t incid) {
    return inc2regid_[incid];
}

const char* getregname(uint8_t id) {
    switch (id) {
        case REGISTER_AL:
            return "AL";
        case REGISTER_AH:
            return "AH";
        case REGISTER_AX:
            return "AX";
        case REGISTER_BL:
            return "BL";
        case REGISTER_BH:
            return "BH";
        case REGISTER_BX:
            return "BX";
        case REGISTER_CL:
            return "CL";
        case REGISTER_CH:
            return "CH";
        case REGISTER_CX:
            return "CX";
        case REGISTER_DL:
            return "DL";
        case REGISTER_DH:
            return "DH";
        case REGISTER_DX:
            return "DX";
        case REGISTER_SP:
            return "SP";
        case REGISTER_BP:
            return "BP";
        case REGISTER_SI:
            return "SI";
        case REGISTER_DI:
            return "DI";
        case REGISTER_CS:
            return "CS";
        case REGISTER_DS:
            return "DS";
        case REGISTER_SS:
            return "SS";
        case REGISTER_ES:
            return "ES";
        default:
            return "(unk)";
    }
}

#endif

#endif
