#ifndef CHARLIE86_RM_H
#define CHARLIE86_RM_H
#include "types.h"
#include "regs.h"
#include "common.h"

#define RM_DIRECT 0b110

uint16_t rm2loc(uint8_t rm) {
    uint16_t loc = 0;
    switch (rm) {
        case 0:
            loc = getregval(REGISTER_BX) + getregval(REGISTER_SI);
            break;
        case 1:
            loc = getregval(REGISTER_BX) + getregval(REGISTER_DI);
            break;
        case 2:
            loc = getregval(REGISTER_BP) + getregval(REGISTER_SI);
            break;
        case 3:
            loc = getregval(REGISTER_BP) + getregval(REGISTER_DI);
            break;
        case 4:
            loc = getregval(REGISTER_SI);
            break;
        case 5:
            loc = getregval(REGISTER_DI);
            break;
        case 6:
            loc = getregval(REGISTER_BP);
            break;
        case 7:
            loc = getregval(REGISTER_BX);
            break;
    }
    return loc;
}

const char* rmnames[] = {
        "[BX+SI]", "[BX+DI]",
        "[BP+SI]", "[BP+DI]",
        "[SI]", "[DI]", "[BP]", "[BX]"
};

const char* rmnames_nb[] = {
        "BX+SI", "BX+DI",
        "BP+SI", "BP+DI",
        "SI", "DI", "BP", "BX"
};

const char* getrmname(uint8_t rm) {
    return rmnames[rm];
}

const char* getrmnamenb(uint8_t rm) {
    return rmnames_nb[rm];
}

uint8_t rmt1[] {
    REGISTER_SI, REGISTER_DI, REGISTER_BP, REGISTER_BX
};

uint16_t getrmdispval(uint8_t rm, sint16_t disp) {
    if (rm >= 4) {
        return getregval(rmt1[rm] + disp);
    } else if (rm == 0) {
        return getregval(REGISTER_BX) + getregval(REGISTER_SI) + disp;
    } else if (rm == 1) {
        return getregval(REGISTER_BX) + getregval(REGISTER_DI) + disp;
    } else if (rm == 2) {
        return getregval(REGISTER_BP) + getregval(REGISTER_SI) + disp;
    } else if (rm == 3) {
        return getregval(REGISTER_BP) + getregval(REGISTER_DI) + disp;
    }

    return 0;
}

typedef struct __mrstruct_t {
    uint8_t mod;
    uint8_t reg;
    uint8_t rm;
    uint8_t w;
    uint8_t d;
    uint8_t orig;
} mrfield;

mrfield parsemr(uint16_t addr, uint8_t ipc) {
    uint8_t mr = getmemory()[addr];

    mrfield ret;

    ret.mod = (mr & 0b11000000) >> 6;
    ret.reg = (mr & 0b00111000) >> 3;
    ret.rm = (mr  & 0b00000111);
    ret.w = ipc&1;
    ret.d = (ipc & 0b10) >> 1;

    ret.orig = mr;

    return ret;
}

#endif //CHARLIE86_RM_H
