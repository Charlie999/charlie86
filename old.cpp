#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>

#include "common.h"
#include "regs.h"
#include "mr.h"
#include "rm.h"
#include "video.h"

#define VIRTUAL_MEMORY_SIZE (1024 * 720) // 720K of virtual memory for now

static uint8_t* virt_memory = NULL;

static uint16_t IP = 0;
static uint16_t regs[12]; // AX,BX,CX,DX SP,BP,SI,DI CS,DS,ES,SS

static int FLAG_ZERO = 0;
static int FLAG_INTERRUPT = 1;
static int FLAG_DIRECTION = 0;
static int FLAG_CARRY = 0;

static int SEGREG_OVERRIDE = -1;

static int running = 1; // if the CPU is running, i.e executing instructions

static std::string printed = std::string();

uint8_t * getmemory() { return virt_memory; }

uint16_t getregval(uint8_t regid);
void setregval(uint8_t regid, uint16_t val);

void interrupt(uint8_t intid) {
    if (!FLAG_INTERRUPT)
        return;

    if (intid == 0x10) { // BIOS display services
        if (getregval(REGISTER_AH) == 0x0E) {
            printed.push_back(getregval(REGISTER_AL));
            printf("VM BIOS print char: %c [vbuf=%s]\n", getregval(REGISTER_AL), printed.c_str());
        } else if (getregval(REGISTER_AH) == 0) {
            printf("VM change video mode to %d\n",getregval(REGISTER_AL));
            switch (getregval(REGISTER_AL)) {
                case 0x13:
                    initvideo(320,200,0xA000);
                    break;
                default:
                    printf("WARNING: Unimplemented video mode %d\n",getregval(REGISTER_AH));
            }
        }
    } else if (intid == 0xFF) { // 0:0 jump callback
        printf("0000:0000 has been jumped to. Halting.\n");
    } else if (intid == 0xFE) { // sooper secret prints for in-VM things like the boot ROM
        printf("VM: %s\n", virt_memory + getregval(REGISTER_SI));
        setregval(REGISTER_SI, 0);
    } else if (intid == 0x1A) {
        setregval(REGISTER_DX, 0);
    }
}

void setregval(uint8_t regid, uint16_t val) {
    if (regid < 4) {
        //printf("SET LOW REG %s[%s] TO %x\n",getregname(regid), getregname(regid+8), val&0x00FF);
        regs[regid] &= 0xFF00;
        regs[regid] |= val & 0x00FF;
    } else if (regid < 8) {
        //printf("SET HIGH REG %s[%s] TO %02x\n",getregname(regid), getregname(regid+4), (val&0xFF));
        regs[regid - 4] &= 0x00FF;
        regs[regid - 4] |= (val & 0xFF) << 8;
    } else {
        //printf("SET REG %s[%d] TO %x\n",getregname(regid), regid - 8, val);
        regs[regid - 8] = val;
    }
}

uint16_t getregval(uint8_t regid) {
    if (regid < 4) {
        return regs[regid] & 0xFF;
    } else if (regid < 8) {
        return (regs[regid - 4] & 0xFF00) >> 8;
    } else {
        return regs[regid - 8];
    }
}

void regdump() {
    printf("[");
    for (int i=0;i<0x8;i++) {
        printf("%s=0x%04X ", getregname(i + 8), getregval(i + 8));
    }
    printf("IP=0x%04X CS=%X DS=%X ES=%X SS=%X] ",IP,
           getregval(REGISTER_CS), getregval(REGISTER_DS), getregval(REGISTER_ES), getregval(REGISTER_SS));
}

void loadsector(uint16_t sid, uint16_t destination, uint16_t count, const char* file) {
    std::ifstream input( file, std::ios::binary );
    std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(input), {});
    if (buffer.size() / 512 < count || buffer.size() < (sid*512 + count*512)) {
        fprintf(stderr, "Warning: file %s doesn't contain enough data to read %d sectors (fz=%d, rq=%d). Aborting.\n", file, count, buffer.size(), (sid*512 + count*512));
        input.close();
        return;
    }
    memcpy(virt_memory + destination, buffer.data() + (sid*512), count * 512);
    printf("Loaded %d sectors from %s [sector %d] to 0x%X\n", count, file, sid, destination);
}

void jump(uint16_t addr) {
    IP = addr;
}

uint32_t segcalc(uint16_t seg, uint16_t off) {
    uint32_t ret = seg;
    ret = ret << 4;
    ret += off;
    return ret;
}

uint32_t pushw(uint16_t val) {
    uint32_t stackptr = segcalc(getregval(REGISTER_SS), getregval(REGISTER_SP));
    setregval(REGISTER_SP, getregval(REGISTER_SP) - 2);
    *((uint16_t*)virt_memory+stackptr) = val;
    return stackptr;
}

uint16_t popw() {
    setregval(REGISTER_SP, getregval(REGISTER_SP) + 2);
    uint32_t stackptr = segcalc(getregval(REGISTER_SS), getregval(REGISTER_SP));
    return *((uint16_t*)virt_memory+stackptr);
}

// this is a lazy, likely buggy 8086 emulator.
// i will probably rewrite this in the future
int main() {
    printf("Starting charlie86\n");
    printf("Registers: [");
    for (int i=0;i<0xF;i++) {
        regs[i] = 0;
        printf("%s=%X ", getregname(i), regs[i]);
    }
    printf("IP=%X]\n",IP);

    virt_memory = (unsigned char*) malloc(VIRTUAL_MEMORY_SIZE);
    memset(virt_memory, 0, VIRTUAL_MEMORY_SIZE);
    memset(virt_memory + 0xA000, 40, 320*240);

    printf("Allocated %d bytes of virtual memory\n", VIRTUAL_MEMORY_SIZE);

    printf("Loading bootsector..\n");
    //loadsector(0, 0x7C00,1, "../disk/INVADERS.BIN");
    loadsector(0, 0x7C00, 1, "../disk/BOOTSEC.BIN");
    loadsector(0, 0x500, 1, "../disk/BOOTROM.BIN"); // load the ROM/(BIOS i guess) at 0x500

    jump(0x500);

    // This is now handled in-VM (in the ROM)
    //if (virt_memory[0x7DFE] != 0x55 || virt_memory[0x7DFF] != 0xAA) {
    //    fprintf(stderr, "Warning: Magic number 55AA not present in bootsector! [%x%x]\n", virt_memory[0x7DFE], virt_memory[0x7DFF]);
    //}

    while (running) {
        uint8_t ipc = *(virt_memory+IP);
        IP++;

        int rec = false;
        if ((ipc & (uint8_t)0xF0) == 0xB0) { // MOV *mostly*
            rec = true;

            uint8_t regid = ipc & (uint8_t)0xF;
            char* regname = (char*) getregname(regid);

            uint16_t d = 0;

            int wide = 1;
            if (regid >= 8){
                d = (virt_memory[IP]) + (virt_memory[IP+1] << (uint8_t)8);
                IP+=2;
            } else {
                d = virt_memory[IP];
                IP++;
                wide = 0;
            }

            setregval(regid, d);

            regdump();
            if (wide)
                printf("MOV %s, 0x%04x [wide]\n",regname,d);
            else
                printf("MOV %s, 0x%02x\n",regname,d);
        } else if ((ipc & 0xF0) == 0xA0 && (ipc & 0x0F) < 3) { // MOV A(L/X)
            rec = true;

            uint8_t regid = (ipc & 0x0F) << 3; // either reg 0 (al) or reg 8 (ax)

            uint16_t a = (virt_memory[IP]) + (virt_memory[IP+1] << (uint8_t)8);
            IP+=2;

            uint16_t d = 0;
            int wide = 1;
            if (regid >= 8){
                d = (virt_memory[a+1]) + (virt_memory[a] << (uint8_t)8);
            } else {
                d = virt_memory[a];
                wide = 0;
            }

            setregval(regid, d);

            regdump();
            if (wide)
                printf("MOV %s, [0x%04x] [wide]\n",getregname(regid),a);
            else
                printf("MOV %s, [0x%04x]\n",getregname(regid),a);
        } else if (ipc == 0x8C || ipc == 0x8E) { // MOV sr, mrw and vice versa
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = 1;
            uint8_t d = (ipc & 0b10) >> 1;

            uint8_t regid1, regid2;

            if (!d) {
                regid1 = (reg + (w << 3));
                regid2 = sr2regid(rm);
            } else {
                regid1 = (rm + (w << 3));
                regid2 = sr2regid(reg);
            }

            uint16_t loc;

            switch (mod) {
                case MR_RM1:
                    loc = rm2loc(rm);
                    *(virt_memory + loc) = (uint16_t)getregval(regid1);

                    regdump();
                    printf("MOV %s, %s\n", getrmname(rm), getregname(regid1));
                    break;
                case MR_RM2_DISP8:
                case MR_RM2_DISP16:
                    printf("FIXME: Unimplemented MOV memory access mode %d\n",mod);
                    running = false;
                    break;
                case MR_2REG:
                    setregval(regid2, getregval(regid1));
                    if (getregval(regid2) == 0)
                        FLAG_ZERO = 1;
                    regdump();
                    printf("MOV %s, %s\n", getregname(regid2), getregname(regid1));
            }
        } else if (ipc == 0x89 || ipc == 0x8B) { // MOV WORD rw, mrw and vice versa
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = 1;
            uint8_t d = (ipc & 0b10) >> 1;

            uint8_t regid1, regid2;

            regid1 = (reg + (w << 3));
            regid2 = (rm + (w << 3));

            uint16_t loc;
            sint8_t l1;
            sint16_t l2;

            switch (mod) {
                case MR_RM1:
                    loc = rm2loc(rm);
                    if (!d) {
                        *(virt_memory + loc) = getregval(regid1);

                        regdump();
                        printf("MOV %s, %s\n", getrmname(rm), getregname(regid1));
                    } else {
                        setregval(regid1, (uint16_t) *(virt_memory + loc));

                        regdump();
                        printf("MOV %s, %s\n", getregname(regid1), getrmname(rm));
                    }
                    break;
                case MR_RM2_DISP8:
                    l1 = (sint8_t)(virt_memory[IP]);
                    IP++;

                    *(virt_memory + getrmdispval(rm, l1)) = getregval(regid1);

                    regdump();
                    printf("MOV [%s%s%d], %s\n", getrmnamenb(rm), (l1<0)?"":"+",l1, getregname(regid1));
                    break;
                case MR_RM2_DISP16:
                    l2 = (sint16_t)(virt_memory[IP] + (virt_memory[IP+1]<<8));
                    IP+=2;

                    *(virt_memory + getrmdispval(rm, l2)) = getregval(regid1);

                    regdump();
                    printf("MOV [%s%s%d], %s\n", getrmnamenb(rm), (l2<0)?"":"+",l2, getregname(regid1));
                    break;
                case MR_2REG:
                    setregval(regid2, getregval(regid1));
                    if (getregval(regid2) == 0)
                        FLAG_ZERO = 1;
                    regdump();
                    printf("MOV %s, %s\n", getregname(regid2), getregname(regid1));
            }
        } else if (ipc == 0x88 || ipc == 0x8A) { // MOV BYTE rw, mrw and vice versa
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = 0;
            uint8_t d = (ipc & 0b10) >> 1;

            uint8_t regid1, regid2;

            regid1 = (reg + (w << 3));
            regid2 = (rm + (w << 3));

            uint16_t loc;

            switch (mod) {
                case MR_RM1:
                    loc = rm2loc(rm);
                    if (!d) {
                        *(virt_memory + loc) = getregval(regid1);

                        regdump();
                        printf("MOV %s, %s\n", getrmname(rm), getregname(regid1));
                    } else {
                        setregval(regid1, (uint16_t) *(virt_memory + loc));

                        regdump();
                        printf("MOV %s, %s\n", getregname(regid1), getrmname(rm));
                    }
                    break;
                case MR_RM2_DISP8:
                case MR_RM2_DISP16:
                    printf("FIXME: Unimplemented MOV memory access mode %d\n",mod);
                    running = false;
                    break;
                case MR_2REG:
                    setregval(regid2, getregval(regid1));
                    if (getregval(regid2) == 0)
                        FLAG_ZERO = 1;
                    regdump();
                    printf("MOV %s, %s\n", getregname(regid2), getregname(regid1));
            }
        } else if (ipc == 0xA2 || ipc == 0xA3) { // MOV [x], AL
            rec=1;

            uint8_t regid = REGISTER_AL + (ipc==0xA3?8:0);
            uint16_t dest = virt_memory[IP] + (virt_memory[IP+1]<<8);
            IP+=2;

            if (ipc==0xA3) {
                *(virt_memory+dest) = getregval(REGISTER_AX);
            } else {
                virt_memory[dest] = getregval(REGISTER_AL)&0xFF;
            }

            regdump();
            printf("MOV [0x%X], %s\n",dest, getregname(regid));
        } else if (ipc == 0xC7) { // mov word [reg], val
            rec=1;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = 0;
            uint8_t d = (ipc & 0b10) >> 1;

            switch (mod) {
                case MR_RM1:
                    if (rm == RM_DIRECT) {
                        uint16_t dest = virt_memory[IP] + (virt_memory[IP+1]<<8);
                        IP+=2;
                        uint16_t term = virt_memory[IP] + (virt_memory[IP+1]<<8);
                        IP+=2;

                        *(virt_memory + dest) = term;

                        regdump();
                        printf("MOV [0x%X], %X\n",dest,term);

                        break;
                    } else {
                        uint16_t loc = rm2loc(rm);

                        uint16_t term = virt_memory[IP] + (virt_memory[IP+1]<<8);

                        *(virt_memory + loc) = virt_memory[IP];
                        *(virt_memory + loc + 1) = virt_memory[IP+1];

                        IP+=2;

                        regdump();
                        printf("MOV %s, %X\n",getrmname(rm),term);

                        break;
                    }
                case MR_RM2_DISP8:
                case MR_RM2_DISP16:
                case MR_2REG:
                    printf("FIXME: [C7] Unimplemented MOV memory access mode %d\n",mod);
                    running = false;
                    break;
            }
        }

        if (ipc == 0x3C || ipc == 0x3D) { // CMP A(L/X)
            rec = true;

            uint8_t regid = (ipc == 0x3D) << 3; // either reg 0 (al) or reg 8 (ax)

            uint16_t d = 0;
            int wide = 1;
            if (regid >= 8){
                d = (virt_memory[IP]) + (virt_memory[IP+1] << (uint8_t)8);
                IP+=2;
            } else {
                d = virt_memory[IP];
                IP++;
                wide = 0;
            }

            FLAG_ZERO = getregval(regid) == d;

            regdump();
            if (wide)
                printf("CMP %s, 0x%04x [wide]\n",getregname(regid),d);
            else
                printf("CMP %s, 0x%02x\n",getregname(regid),d);
        } else if ((ipc == 0x80 || ipc == 0x83) && ((virt_memory[IP])&0b111000)==0b111000) { // CMP rm[w/b] ib
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = ipc & 1;
            uint8_t wi = (ipc == 0x81);

            uint8_t regid1 = reg + (w<<3);
            uint8_t regid2 = rm + (w<<3);

            uint16_t loc;

            uint16_t op1 = 0xFFFF;
            uint16_t op2 = 0xFFFE;

            switch (mod) {
                case MR_RM1:
                    loc = rm2loc(rm);
                    op1 = (uint16_t)*(virt_memory + loc);
                    if (w) {
                        op2 = ((virt_memory[IP] << 8) + virt_memory[IP]);
                        IP+=2;
                    } else {
                        op2 = virt_memory[IP];
                        IP++;
                    }
                    regdump();
                    printf("CMP %s, 0x%X\n",getrmname(rm), op2);
                    break;
                case MR_RM2_DISP8:
                case MR_RM2_DISP16:
                    printf("FIXME: [80,83] Unimplemented CMP memory access mode %d\n",mod);
                    running = false;
                    break;
                case MR_2REG:
                    op2 = getregval(regid1);
                    op1 = getregval(regid2);
                    if (op1 == op2)
                        FLAG_ZERO = 1;
                    regdump();
                    printf("CMP %s, %s\n", getregname(regid1), getregname(regid2));
                    break;
            }
        } else if (ipc == 0x81 && ((virt_memory[IP])&0b111000)==0b111000) { // CMP word REG, VAL
            rec=true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = ipc & 1;
            uint8_t wi = (ipc == 0x81);

            uint8_t regid1 = rm + (w<<3);

            uint16_t val = ((virt_memory[IP+1] << 8) + virt_memory[IP]);
            IP+=2;

            regdump();
            printf("CMP %s, 0x%X\n", getregname(regid1), val);

        }

        if (ipc == 0xAC) { // LODSB
            rec = true;

            setregval(REGISTER_AL, *(virt_memory + getregval(REGISTER_SI)));
            if (FLAG_DIRECTION)
                setregval(REGISTER_SI, getregval(REGISTER_SI) - 1);
            else
                setregval(REGISTER_SI, getregval(REGISTER_SI) + 1);

            regdump();
            printf("LODSB\n");
        } else if (ipc == 0xAD) { // LODSW
            rec = true;

            setregval(REGISTER_AX, *(virt_memory + getregval(REGISTER_SI)));
            if (FLAG_DIRECTION)
                setregval(REGISTER_SI, getregval(REGISTER_SI) - 1);
            else
                setregval(REGISTER_SI, getregval(REGISTER_SI) + 1);

            regdump();
            printf("LODSW\n");
        }

        if (ipc == 0x08 || ipc == 0x09) { // OR
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = ipc & 1;

            uint8_t regid = -1;

            switch (mod) {
                case MR_RM1:
                case MR_RM2_DISP8:
                case MR_RM2_DISP16:
                    printf("FIXME: Unimplemented OR memory access mode\n");
                    rec = false; // let's just ignore this for now :)
                    continue;
                case MR_2REG:
                    regid = reg + (w<<3);
                    setregval(regid, getregval(regid) | getregval(regid));
                    if (getregval(regid) == 0)
                        FLAG_ZERO = 1;
                default:
                    rec = false; // can't happen so probably a good idea to raise an error
                    continue;
            }

            regdump();
            printf("OR mod=%x, reg=%s, rm=%x\n", mod, getregname(regid), rm);
        }

        if (ipc == 0x00 || ipc == 0x01) { // ADD
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = ipc & 1;

            uint8_t regid1 = reg + (w<<3);
            uint8_t regid2 = rm + (w<<3);

            uint16_t loc;

            switch (mod) {
                case MR_RM1:
                    loc = rm2loc(rm);
                    *(virt_memory + loc) = getregval(regid1) + (uint16_t)*(virt_memory + loc);

                    regdump();
                    printf("ADD %s, %s\n", getrmname(rm), getregname(regid1));
                    break;
                case MR_RM2_DISP8:
                case MR_RM2_DISP16:
                    printf("FIXME: Unimplemented ADD memory access mode %d\n",mod);
                    running = false;
                    break;
                case MR_2REG:
                    setregval(regid2, getregval(regid1) + getregval(regid2));
                    if (getregval(regid2) == 0)
                        FLAG_ZERO = 1;
                    regdump();
                    printf("ADD %s, %s\n", getregname(regid2), getregname(regid1));
            }
        } else if ((ipc == 0x80 || ipc == 0x83) && ((virt_memory[IP])&0b111000)==0) { // ADD rmw, ib
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t il = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = ipc & 1;

            uint8_t regid2 = rm + (w<<3);

            uint16_t loc;

            uint16_t amt = 0;

            switch (il) {
                case 3:
                    amt += virt_memory[IP];
                    IP++;
                    break;
                case 4:
                    amt += (virt_memory[IP]<<8) + virt_memory[IP];
                    IP+=2;
                    break;
                case 5:
                    printf("FIXME: Unimplemented ADD IL mode\n");
                    running = false;
                    break;
            }

            setregval(regid2, amt + getregval(regid2));
            if (getregval(regid2) == 0)
                FLAG_ZERO = 1;
            regdump();
            printf("ADD %s, %d\n", getregname(regid2), amt);
        } else if (ipc == 0x04 || ipc == 0x05) {
            uint8_t wide = ipc&1;

            uint16_t v;

            if (wide) {
                v = (virt_memory[IP+1]<<8) + virt_memory[IP];
                IP+=2;
                setregval(REGISTER_AX, getregval(REGISTER_AX) + v);
            } else {
                v = virt_memory[IP];
                setregval(REGISTER_AL, getregval(REGISTER_AL) + v);
                IP++;
            }

            regdump();
            printf("ADD %s, 0x%04X\n", getregname(REGISTER_AL + (wide<<3)), v);

            rec = true;
        }

        if (ipc == 0x30 || ipc == 0x31) { // XOR
            rec = true;

            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = ipc & 1;

            uint8_t regid1 = reg + (w<<3);
            uint8_t regid2 = rm + (w<<3);

            uint16_t loc;

            switch (mod) {
                case MR_RM1:
                    loc = rm2loc(rm);
                    *(virt_memory + loc) = getregval(regid1) ^ (uint16_t)*(virt_memory + loc);

                    regdump();
                    printf("XOR %s, %s\n", getrmname(rm), getregname(regid1));
                    break;
                case MR_RM2_DISP8:
                case MR_RM2_DISP16:
                    printf("FIXME: Unimplemented OR memory access mode %d\n",mod);
                    running = false;
                    break;
                case MR_2REG:
                    setregval(regid2, getregval(regid1) ^ getregval(regid2));
                    if (getregval(regid2) == 0)
                        FLAG_ZERO = 1;
                    regdump();
                    printf("XOR %s, %s\n", getregname(regid2), getregname(regid1));
            }
        }

        if (ipc == 0x74) { // JZ
            rec=true;

            sint8_t rel = (sint8_t)virt_memory[IP];
            IP++;

            if (FLAG_ZERO)
                jump(IP + rel);
            int jumped = FLAG_ZERO;
            FLAG_ZERO = 0;

            regdump();
            printf("JZ 0x%04X",IP);
            if (jumped)
                printf(" [jumped]");
            else
                printf(" [not jumped]");
            printf("\n");
        } else if (ipc == 0x75) { // JNZ
            rec=true;

            sint8_t rel = (sint8_t)virt_memory[IP];
            IP++;

            if (!FLAG_ZERO)
                jump(IP + rel);
            int jumped = !FLAG_ZERO;

            regdump();
            printf("JNZ 0x%04X",IP);
            if (jumped)
                printf(" [jumped]");
            else
                printf(" [not jumped]");
            printf("\n");
        }

        if (ipc == 0xCC || ipc == 0xCD) { // INT
            rec=true;

            uint8_t intid = 0xFF;

            if (ipc==0xCC) {
                intid = 3;
            } else {
                intid = virt_memory[IP];
                IP++;
            }

            regdump();
            printf("INT 0x%02X\n",intid);

            interrupt(intid);
        }

        if (ipc == 0xEB) { // JUMP SHORT 8
            rec = true;

            sint8_t rel = (sint8_t)virt_memory[IP];
            IP++;

            jump(IP + rel);

            regdump();
            printf("JMP 0x%04X [near, 8, %02X]\n",IP, rel);
        }

        if (ipc == 0xE9) { // JUMP SHORT 16
            rec = true;

            sint16_t rel = (sint16_t)(virt_memory[IP] + (virt_memory[IP+1] << 8));
            IP+=2;

            int neg = rel<0;

            if (rel < 0 && (rel+IP) < 0)
                jump(0xFFFF - rel);
            else
                jump(IP + rel);

            regdump();
            printf("JMP 0x%04X [near, 16, %c%04X]\n",IP, neg?'-':'+', (uint16_t)rel);
        }

        if (ipc == 0xEA) { // JUMP FAR
            rec = true;

            uint16_t off = (virt_memory[IP] + (virt_memory[IP + 1] << 8));
            IP+=2;

            uint16_t seg = virt_memory[IP] + (virt_memory[IP + 1] << 8);
            setregval(REGISTER_CS, seg);
            IP+=2;

            jump(off);

            regdump();
            printf("JMP 0x%04X [far, 16, %04X:%04X]\n",IP, seg, off);
        }

        if (ipc == 0xFA) { // CLI
            regdump();
            printf("CLI\n");

            rec = true;
            FLAG_INTERRUPT = 0;
        }

        if (ipc == 0xF4) { // HLT
            regdump();
            printf("HLT\n");

            rec = true;
            running = false;
            break;
        }

        if (ipc == 0x90) { // NOP
            regdump();
            printf("NOP\n");

            rec = true;
        }

        if (ipc == 0xFC) { // CLD
            regdump();
            printf("CLD\n");

            FLAG_DIRECTION = 0;
            rec = true;
        }

        if (ipc == 0xF3) { // REP

            uint8_t next = virt_memory[IP];
            IP++;

            uint8_t wide;
            uint8_t d;
            uint32_t start;

            int reps = 0;

            if (next==0xAA || next==0xAB)
                start = segcalc(getregval(REGISTER_ES),getregval(REGISTER_DI));

            while (getregval(REGISTER_CX) > 0) {
                reps++;
                switch (next) {
                    case 0xAA:
                    case 0xAB: // REP STOS[B/W]
                        wide = next & 1;
                        d = (next & 0b10) >> 1;

                        if (wide) {
                            (*(virt_memory + segcalc(getregval(REGISTER_ES),getregval(REGISTER_DI)))) = getregval(REGISTER_AX);
                        } else {
                            (*(virt_memory + segcalc(getregval(REGISTER_ES),getregval(REGISTER_DI)))) = (uint8_t) getregval(REGISTER_AL);
                        }

                        if (d) {
                            setregval(REGISTER_DI, getregval(REGISTER_DI + wide + 1));
                        } else {
                            setregval(REGISTER_DI, getregval(REGISTER_DI - (wide + 1)));
                        }

                        break;
                    default:
                        printf("FIXME: Not implemented REP operation %02X\n", next);
                        rec = false;
                        break;
                }

                setregval(REGISTER_CX, getregval(REGISTER_CX) - 1);
            }

            if (next == 0xAA || next == 0xAB) {
                regdump();
                printf("REP STOS [%d(%X) reps, %X start, %X end]\n",reps,reps,start,segcalc(getregval(REGISTER_ES),getregval(REGISTER_DI)));
            } else {
                regdump();
                printf("REP (unk) [%d(%X) reps]\n",reps,reps);
            }

            rec = true;
        }

        if (ipc >= 0x40 && ipc <= 0x47) { // INC [reg]
            uint8_t regid = inc2regid(ipc - 0x40);

            setregval(regid, getregval(regid) + 1);

            regdump();
            printf("INC %s\n", getregname(regid));

            rec = true;
        } else if (ipc == 0xFE || ipc == 0xFF) {
            uint8_t mr = virt_memory[IP];
            IP++;

            uint8_t mod = (mr & 0b11000000) >> 6;
            uint8_t reg = (mr & 0b00111000) >> 3;
            uint8_t rm = (mr  & 0b00000111);
            uint8_t w = ipc & 1;

            uint8_t regid1 = reg + (w<<3);
            uint8_t regid2 = rm + (w<<3);

            setregval(regid2, getregval(regid2)+1);

            regdump();
            printf("INC %s\n",getregname(regid2));

            rec=true;
        }

        if (ipc == 0xAA || ipc == 0xAB) { // STOS[B/W]
            uint8_t wide = ipc & 1;
            uint8_t d = (ipc & 0b10) >> 1;

            if (wide) {
                (*(virt_memory + segcalc(getregval(REGISTER_ES),getregval(REGISTER_DI)))) = getregval(REGISTER_AX);
            } else {
                (*(virt_memory + segcalc(getregval(REGISTER_ES),getregval(REGISTER_DI)))) = (uint8_t) getregval(REGISTER_AL);
            }

            uint16_t bdi = getregval(REGISTER_DI);

            if (!d) {
                setregval(REGISTER_DI, getregval(REGISTER_DI) + (wide + 1));
            } else {
                setregval(REGISTER_DI, getregval(REGISTER_DI) - (wide + 1));
            }

            rec = true;
            regdump();
            printf("STOSW %04X => %X\n",getregval(REGISTER_AX),segcalc(getregval(REGISTER_ES),bdi));
        }

        if (ipc >= 0x91 && ipc <= 0x97) { // XCHG AX,[reg]
            uint8_t regid = inc2regid(ipc - 0x90);

            uint16_t ro = getregval(REGISTER_AX);
            setregval(REGISTER_AX, getregval(regid));
            setregval(regid, ro);

            regdump();
            printf("XCHG AX,%s\n", getregname(regid));

            rec = true;
        }

        if (ipc == 0xE2) { // LOOP
            sint8_t dest = (sint8_t)virt_memory[IP];
            IP++;

            uint8_t jumped = 0;
            if (getregval(REGISTER_CX) > 0) {
                setregval(REGISTER_CX, getregval(REGISTER_CX) - 1);
                jump(IP + dest);
                jumped = 1;
            }

            regdump();
            printf("LOOP %04X [%s]\n",IP,jumped?"jumped":"not jumped");

            rec = true;
        }

        if (ipc >= 0x50 && ipc <= 0x57) { // PUSH
            rec=true;
            uint8_t regid = (ipc-0x50) + 8;

            pushw(getregval(regid));

            regdump();
            printf("PUSH %s\n", getregname(regid));
        }

        if (ipc >= 0x58 && ipc <= 0x5F) { // POP
            rec=true;
            uint8_t regid = (ipc-0x58) + 8;

            setregval(regid, popw());

            regdump();
            printf("POP %s\n", getregname(regid));
        }

        if (ipc == 0x9C) { // PUSHF
            rec = true;

            uint16_t stored_flags = FLAG_ZERO;
            stored_flags |= (FLAG_INTERRUPT << 1);
            stored_flags |= (FLAG_DIRECTION << 2);
            stored_flags |= (FLAG_CARRY << 3);

            pushw(stored_flags);

            regdump();
            printf("PUSHF\n");
        }

        if (ipc == 0xE8) { // CALL
            rec=true;

            sint16_t rel = *(virt_memory+IP);
            IP+=2;
            pushw(IP);

            jump(IP+rel);

            regdump();
            printf("CALL 0x%04X\n", IP);
        }

        if (ipc == 0xC3) { // RET
            rec = true;

            jump(popw());

            regdump();
            printf("RET\n");
        }

        if (ipc == 0xD7) { // XLAT
            rec=true;

            uint8_t * table;

            uint8_t SEGREG = (SEGREG_OVERRIDE>-1)?SEGREG_OVERRIDE:REGISTER_DS;

            table = virt_memory + segcalc(getregval(SEGREG), getregval(REGISTER_BX));

            uint8_t idx = getregval(REGISTER_AL);
            setregval(REGISTER_AL, table[idx]);

            regdump();
            printf("XLAT %02X => %02X [%X]\n",idx,getregval(REGISTER_AL),table-virt_memory);
        }

        if (ipc == 0xF8) { // CLC
            rec=1;
            FLAG_CARRY = 0;
            regdump();
            printf("CLC\n");
        }

        SEGREG_OVERRIDE = -1;

        if (ipc == 0x2E) { // CS prefix
            rec=true;

            SEGREG_OVERRIDE = REGISTER_CS;

            regdump();
            printf("CS PRE\n");
        }

        // if instruction not "recognised", i.e. not executed
        if (!rec) {
            regdump();
            printf("Invalid instruction 0x%02x\n",ipc);
            running = false;
        }
    }

    printf("Halted [vbuf=%s]\n",printed.c_str());

    printf("Press a key to end\n");
    getchar();

    killvideo();
    free(virt_memory);
    return 0;
}
